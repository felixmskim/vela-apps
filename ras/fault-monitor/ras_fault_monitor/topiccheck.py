# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""Topic-level anomaly verdicts — built on the premise that **which sensors will run is
unknown**.

Hardcoding "an IMU should read near 9.8 m/s²" only works in a demo. For a drone in free
fall 48 m/s² is normal, and in a crash test it is the expected value. The physically
normal range of a sensor value depends on **the application's intent**, which the daemon
has no way to know.

So verdicts are graded by evidence. The top holds with no configuration at all; further
down the claims get stronger but need grounds.

  Tier 0 `structural`      Type-independent: publication stopped, period shifted, size
                           shifted. No message class required (raw CDR bytes suffice),
                           so custom `.msg` types are covered too.
  Tier 1 `self_declared`   A violation of **the contract the message declares about
                           itself**. No configuration needed: NaN/Inf, a bad
                           `header.stamp`, variable-array structure mismatch,
                           `LaserScan.ranges ∉ [range_min, range_max]`,
                           `NavSatFix.status == STATUS_NO_FIX`, an empty payload.
  Tier 2 `baseline`        Departure from an observed baseline. It can say "different
                           from usual" and nothing more — never "physically wrong".
  Tier 3 `declared_limit`  A physical limit supplied by the integrator. The only tier
                           that claims something is wrong.

This module **never imports rclpy**. It holds pure verdict functions and state, while
the bridge node (`ros2topic_node.py`) does the actual subscribing. That way the verdict
logic can be verified on the host without QEMU, and the daemon core is unaffected if the
bridge dies.

The Tier 2 trap
---------------
If the system is already broken during warmup, that state gets learned as normal. So a
Tier 0/1 violation during warmup **refuses adoption of the baseline**.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .fault import Fault

#: evidence-grade tags — a log consumer has to be able to tell how strong a claim is.
TIER_STRUCTURAL = "anomaly:structural"
TIER_SELF_DECLARED = "anomaly:self_declared"
TIER_BASELINE = "anomaly:baseline_deviation"
TIER_DECLARED = "anomaly:declared_limit"

#: STATUS_NO_FIX from `sensor_msgs/msg/NavSatStatus`
NAVSAT_NO_FIX = -1


def _is_bad_float(value) -> bool:
    return isinstance(value, float) and (math.isnan(value) or math.isinf(value))


# -- Tier 1: the contract the message declares about itself ------------------

def check_self_declared(type_name: str, fields: dict) -> list[tuple[str, str]]:
    """A list of `(code, description)`, limited to what needs no configuration.

    `fields` is the message flattened into a dict (built and passed in by the bridge).
    Dispatch is by type name, but **an unknown type still gets the common checks** —
    NaN/Inf and stamp make a message invalid whatever it is.
    """
    out: list[tuple[str, str]] = []

    # common (1) NaN/Inf — invalid for any physical quantity
    for key, value in fields.items():
        if _is_bad_float(value):
            out.append(("nan_inf", f"{key} = {value}"))
        elif isinstance(value, (list, tuple)):
            bad = sum(1 for v in value if _is_bad_float(v))
            if bad and bad == len(value):
                out.append(("all_nan_inf", f"all {bad} entries of {key} are NaN/Inf"))
            elif bad:
                out.append(("nan_inf_partial", f"{bad}/{len(value)} entries of {key}"))

    # common (2) timestamp
    stamp = fields.get("header.stamp_sec")
    if stamp is not None and stamp == 0:
        out.append(("zero_stamp",
                    "header.stamp is 0 — the publisher never filled in the time"))

    # LaserScan: the message carries its own valid range
    if type_name.endswith("LaserScan"):
        lo, hi = fields.get("range_min"), fields.get("range_max")
        ranges = fields.get("ranges")
        if isinstance(ranges, (list, tuple)):
            if not ranges:      # came as a list and is empty = definitely empty payload
                out.append(("empty_payload", "ranges is empty"))
            elif isinstance(lo, float) and isinstance(hi, float) and hi > lo:
                bad = [r for r in ranges
                       if isinstance(r, float) and not math.isnan(r)
                       and not math.isinf(r) and not (lo <= r <= hi)]
                if bad:
                    out.append(("out_of_declared_range",
                                f"{len(bad)} entries of ranges outside "
                                f"[{lo}, {hi}] (message self-declared)"))
        inc = fields.get("angle_increment")
        amin, amax = fields.get("angle_min"), fields.get("angle_max")
        if (isinstance(ranges, (list, tuple)) and ranges
                and all(isinstance(v, float) for v in (inc, amin, amax))
                and inc not in (0.0, None)):
            expect = int(round((amax - amin) / inc)) + 1
            if abs(expect - len(ranges)) > 1:
                out.append(("array_size_mismatch",
                            f"{len(ranges)} ranges vs {expect} computed from angles"))

    # Range: min_range/max_range are self-declared
    if type_name.endswith("/Range"):
        lo, hi, val = (fields.get("min_range"), fields.get("max_range"),
                       fields.get("range"))
        if all(isinstance(v, float) for v in (lo, hi, val)) and hi > lo:
            if not (lo <= val <= hi):
                out.append(("out_of_declared_range",
                            f"range {val} ∉ [{lo}, {hi}] (message self-declared)"))

    # NavSatFix: status reports the absence of a fix on its own
    if type_name.endswith("NavSatFix"):
        if fields.get("status.status") == NAVSAT_NO_FIX:
            out.append(("no_fix", "NavSatFix.status = STATUS_NO_FIX"))

    # PointCloud2 / Image: size consistency.
    # `data_len` **absent (unknown)** is distinguished from **0 (empty)**. Calling a
    # missing field "empty" produces a false verdict that pollutes warmup and ends up
    # refusing the baseline too (measured: it disabled the Image topic entirely).
    if type_name.endswith("PointCloud2"):
        w, h, step = (fields.get("width"), fields.get("height"),
                      fields.get("point_step"))
        n = fields.get("data_len")
        if n == 0:
            out.append(("empty_payload", "data is empty"))
        elif all(isinstance(v, int) for v in (w, h, step, n)) and w * h * step != n:
            out.append(("array_size_mismatch",
                        f"width*height*point_step={w * h * step} vs data={n}"))
    if type_name.endswith("/Image"):
        h, step, n = fields.get("height"), fields.get("step"), fields.get("data_len")
        if n == 0:
            out.append(("empty_payload", "data is empty"))
        elif all(isinstance(v, int) for v in (h, step, n)) and h * step != n:
            out.append(("array_size_mismatch", f"height*step={h * step} vs data={n}"))

    # BatteryState: the health enum reports the failure on its own
    if type_name.endswith("BatteryState"):
        health = fields.get("power_supply_health")
        # 1=GOOD is fine, other diagnostic values are problems (0=UNKNOWN is deferred)
        if isinstance(health, int) and health not in (0, 1):
            out.append(("battery_health", f"power_supply_health = {health}"))
    return out


# -- Tier 0/2: per-topic state -----------------------------------------------

@dataclass
class TopicState:
    """Observed state of one topic. The baseline grows here."""

    topic: str
    type_name: str = ""
    count: int = 0
    last_wall: float = 0.0
    intervals: list = field(default_factory=list)     # recent publication intervals
    sizes: list = field(default_factory=list)         # recent message sizes
    baseline_period: float = 0.0
    baseline_size: float = 0.0
    baseline_locked: bool = False
    baseline_rejected: str = ""                       # why adoption was last refused
    baseline_attempts: int = 0
    violations_during_warmup: int = 0
    warmup_base: int = 0          # the count when this warmup window opened
    deadline_reported: bool = False
    #: code -> {"n": occurrences, "first": wall, "last_report": wall}
    #: A violating publisher violates on *every* message, so an unsuppressed verdict
    #: is emitted at the topic's publication rate. Measured: one 4 Hz talker produced
    #: 9,617 identical records and buried every other event in the log.
    active: dict = field(default_factory=dict)

    def observe(self, wall: float, size: int, warmup_n: int, keep: int) -> None:
        if self.last_wall:
            gap = wall - self.last_wall
            if gap > 0:
                self.intervals.append(gap)
                del self.intervals[:-keep]
        self.last_wall = wall
        self.count += 1
        self.sizes.append(size)
        del self.sizes[:-keep]
        self.deadline_reported = False

        if (not self.baseline_locked
                and self.count - self.warmup_base >= warmup_n and self.intervals):
            self.baseline_attempts += 1
            if self.violations_during_warmup:
                # Something was already wrong during warmup. Learning that state as
                # normal makes every later verdict meaningless. But this is **not a
                # permanent refusal**: the monitor may simply have attached while the app
                # was already misbehaving, and that must not cost the topic its Tier 0/2
                # checks forever. A new window opens and a clean warmup is awaited.
                self.baseline_rejected = (
                    f"{self.violations_during_warmup} Tier 0/1 violations during "
                    f"warmup #{self.baseline_attempts} — awaiting retry")
                self.violations_during_warmup = 0
                self.warmup_base = self.count
                self.intervals.clear()
                return
            ordered = sorted(self.intervals)
            self.baseline_period = ordered[len(ordered) // 2]     # median
            self.baseline_size = sum(self.sizes) / len(self.sizes)
            self.baseline_locked = True
            self.baseline_rejected = ""      # the retry succeeded

    @property
    def has_baseline(self) -> bool:
        return self.baseline_locked and not self.baseline_rejected


class TopicMonitor:
    """Per-topic verdicts, as pure functions with no rclpy."""

    def __init__(self, node_name: str = "topic_monitor", warmup_n: int = 20,
                 keep: int = 50, deadline_factor: float = 5.0,
                 size_tolerance: float = 4.0, min_deadline_s: float = 1.0,
                 repeat_after_s: float = 300.0):
        self.node_name = node_name
        self.warmup_n = warmup_n
        self.keep = keep
        self.deadline_factor = deadline_factor
        self.size_tolerance = size_tolerance
        self.min_deadline_s = min_deadline_s
        self.repeat_after_s = repeat_after_s
        self.topics: dict[str, TopicState] = {}

    def _state(self, topic: str, type_name: str = "") -> TopicState:
        st = self.topics.get(topic)
        if st is None:
            st = TopicState(topic=topic, type_name=type_name)
            self.topics[topic] = st
        elif type_name and not st.type_name:
            st.type_name = type_name
        return st

    # -- message arrival ----------------------------------------------------
    def on_message(self, topic: str, type_name: str, wall: float, size: int,
                   fields: dict | None = None) -> list[Fault]:
        """One message -> the verdicts it produces."""
        st = self._state(topic, type_name)
        verdicts: list[dict] = []

        # Tier 1 — only when the fields are visible (in raw mode fields is None)
        if fields:
            for code, why in check_self_declared(type_name, fields):
                verdicts.append({
                    "tier": TIER_SELF_DECLARED, "code": code, "why": why,
                    "fault_type": ("SENSOR_LOGIC_FAULT" if code != "empty_payload"
                                   else "SENSOR_HW_FAULT"),
                    "severity": "WARN", "expected": 0.0, "actual": 0.0})

        # Tier 2 — size shift (only with a baseline)
        if st.has_baseline and st.baseline_size > 0:
            ratio = size / st.baseline_size
            if ratio > self.size_tolerance or ratio < 1.0 / self.size_tolerance:
                verdicts.append({
                    "tier": TIER_BASELINE, "code": "size_deviation",
                    "why": f"size {size}B against a {st.baseline_size:.0f}B baseline",
                    "fault_type": "SENSOR_LOGIC_FAULT", "severity": "WARN",
                    "expected": st.baseline_size, "actual": float(size)})

        # Warmup accounting counts **every** verdict, not just the emitted ones.
        # Counting emissions instead would let a suppressed run of violations look
        # like a clean warmup, and the baseline would be learned from bad data.
        if verdicts and not st.baseline_locked:
            st.violations_during_warmup += len(verdicts)

        faults: list[Fault] = []
        for v in verdicts:
            report, folded = self._gate(st, v["code"], wall)
            if not report:
                continue
            f = self._fault(st, v["tier"], v["code"], v["why"],
                            fault_type=v["fault_type"], severity=v["severity"],
                            expected=v["expected"], actual=v["actual"])
            if folded:
                f.detail["occurrences"] = folded
                f.tags.append("still_violating")
            faults.append(f)
        faults.extend(self._clear_gone(st, {v["code"] for v in verdicts}))

        st.observe(wall, size, self.warmup_n, self.keep)
        return faults

    # -- deadline checks (the point is when no message arrives) --------------
    def check_deadlines(self, now: float) -> list[Fault]:
        """Detects publication stopping. **A message not arriving** is the single most
        important signal."""
        faults: list[Fault] = []
        for st in self.topics.values():
            if not st.has_baseline or st.deadline_reported or not st.last_wall:
                continue
            deadline = max(st.baseline_period * self.deadline_factor,
                           self.min_deadline_s)
            silence = now - st.last_wall
            if silence > deadline:
                st.deadline_reported = True
                faults.append(self._fault(
                    st, TIER_STRUCTURAL, "comm_deadline",
                    f"nothing published for {silence:.1f}s (baseline period "
                    f"{st.baseline_period:.3f}s, allowance {deadline:.1f}s)",
                    fault_type="DDS_FAULT", severity="ERROR",
                    expected=deadline, actual=silence))
        return faults

    # -- repeat suppression -------------------------------------------------
    def _gate(self, st: TopicState, code: str, wall: float) -> tuple[bool, int]:
        """Report a repeating condition once, then only on a heartbeat.

        Returns `(report_now, folded_occurrences)`. The same principle the health
        checks follow: a condition that persists is a *state*, and re-emitting it every
        message drowns the new events that matter. The heartbeat exists so that a
        violation lasting hours does not become invisible.
        """
        ent = st.active.get(code)
        if ent is None:
            st.active[code] = {"n": 1, "first": wall, "last_report": wall}
            return True, 0
        ent["n"] += 1
        if wall - ent["last_report"] >= self.repeat_after_s:
            folded = ent["n"]
            ent["last_report"] = wall
            ent["n"] = 0
            return True, folded
        return False, 0

    def _clear_gone(self, st: TopicState, still: set) -> list[Fault]:
        """A condition that stopped firing is recorded as cleared.

        Without this, a log reader cannot tell "still broken" from "fixed itself", and
        the response to a fault can never be judged.
        """
        out: list[Fault] = []
        for code in [c for c in st.active if c not in still]:
            ent = st.active.pop(code)
            f = self._fault(st, TIER_STRUCTURAL, code,
                            f"cleared: {code} no longer violating",
                            fault_type="SENSOR_LOGIC_FAULT", severity="INFO")
            f.detail["occurrences"] = ent["n"]
            f.tags.append("recovered")
            out.append(f)
        return out

    # -- helpers ------------------------------------------------------------
    def _fault(self, st: TopicState, tier: str, code: str, why: str,
               fault_type: str, severity: str,
               expected: float = 0.0, actual: float = 0.0) -> Fault:
        return Fault(
            fault_type=fault_type, severity=severity,
            source_layer="application", recoverable=True,
            description=f"{st.topic}: {why}",
            tags=["ros2", "topic", tier, f"code:{code}",
                  f"topic:{st.topic}", f"type:{st.type_name}"],
            node_name=self.node_name, topic_name=st.topic,
            expected_value=expected, actual_value=actual,
            detail={"topic": st.topic, "type": st.type_name, "code": code,
                    "tier": tier.split(":", 1)[1],
                    "baseline_period_s": st.baseline_period,
                    "baseline_rejected": st.baseline_rejected,
                    "observed_count": st.count})

    def status(self) -> dict:
        return {t: {"type": s.type_name, "count": s.count,
                    "baseline_period_s": round(s.baseline_period, 4),
                    "baseline_rejected": s.baseline_rejected,
                    "baseline_attempts": s.baseline_attempts,
                    "has_baseline": s.has_baseline}
                for t, s in sorted(self.topics.items())}
