# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""The configuration surface — the one place an integrator injects application knowledge.

Principle
---------
**Monitoring holds with no configuration.** The defaults are safe in general and
configuration only adds precision. Making a config file mandatory makes "just start it
and see" impossible, and then nobody starts it.

What configuration unlocks (things impossible in principle without it)
---------------------------------------------------------------------
1. **Threshold tuning** — 90% disk on a logging box does not mean what 90% means on a
   drone.
2. **Naming the critical nodes** — the daemon cannot tell `/diagnostics` dying apart from
   `/cmd_vel` dying. That is not knowledge observation can supply.
3. **Registering a recovery mechanism** — how a node is brought back. Some nodes hold
   state and get worse when restarted.
4. **Tier 3 physical limits** — the only grounds on which "48 m/s² on the IMU is wrong"
   can be claimed. For a drone in free fall it is normal, so only the integrator can
   make that call.
5. **Whether core loss is tolerable** — with a real-time deadline, detaching a core is
   itself the failure. An app-independent action still has app-visible consequences.

Dependency
----------
YAML is read when `PyYAML` is present, JSON otherwise. An embedded image may not carry
PyYAML, and failing to read the policy for that reason would be unacceptable.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field

from .health import Threshold

DEFAULT_PATHS = ("/etc/ras/policy.yaml", "/etc/ras/policy.yml",
                 "/etc/ras/policy.json")


@dataclass
class NodePolicy:
    """Application knowledge about one node."""

    name: str
    critical: bool = False
    restart: str = ""          # recovery command; empty = never restart
    stateful: bool = False     # holding state makes a restart harmful
    max_restarts: int = 3      # more restarts than this counts as a loop

    @property
    def restartable(self) -> bool:
        return bool(self.restart) and not self.stateful


@dataclass
class TopicPolicy:
    """Expectations about one topic; the grounds for a Tier 3 verdict."""

    name: str
    required: bool = False
    min_hz: float = 0.0
    limits: dict = field(default_factory=dict)   # {"linear_acceleration.z": [8, 12]}

    def violated_limits(self, fields: dict) -> list[tuple[str, str]]:
        """A configured physical limit violated. **The only verdict that claims
        something is wrong.**"""
        out = []
        for path, bounds in self.limits.items():
            value = fields.get(path)
            if not isinstance(value, (int, float)):
                continue
            try:
                lo, hi = float(bounds[0]), float(bounds[1])
            except (TypeError, ValueError, IndexError):
                continue
            if not (lo <= value <= hi):
                out.append(("declared_limit",
                            f"{path} = {value} ∉ [{lo}, {hi}] (configured limit)"))
        return out


@dataclass
class Policy:
    """The whole policy. No field is mandatory."""

    thresholds: Threshold = field(default_factory=Threshold)
    nodes: dict = field(default_factory=dict)     # name -> NodePolicy
    topics: dict = field(default_factory=dict)    # name -> TopicPolicy
    core_loss_allowed: bool = True
    min_cores: int = 2
    armed: bool | None = None                     # None = follow the CLI value
    source: str = ""                              # where it was read from
    errors: list = field(default_factory=list)    # parse problems (never swallowed)

    # -- lookup -------------------------------------------------------------
    def node(self, name: str) -> NodePolicy | None:
        return self.nodes.get(name)

    def topic(self, name: str) -> TopicPolicy | None:
        return self.topics.get(name)

    def is_critical(self, node_name: str) -> bool:
        """An unlisted node is **not assumed critical**.

        Treating the unknown as critical makes every node death CRITICAL and the severity
        scale stops meaning anything. A node marked critical, in contrast, has definite
        grounds behind it.
        """
        p = self.nodes.get(node_name)
        return bool(p and p.critical)

    def recovery_for(self, node_name: str) -> str:
        p = self.nodes.get(node_name)
        return p.restart if (p and p.restartable) else ""

    def required_topics(self) -> list:
        return sorted(n for n, p in self.topics.items() if p.required)

    def summary(self) -> dict:
        return {
            "source": self.source or "(none — defaults)",
            "nodes": len(self.nodes),
            "critical_nodes": sorted(n for n, p in self.nodes.items() if p.critical),
            "restartable_nodes": sorted(n for n, p in self.nodes.items()
                                        if p.restartable),
            "topics": len(self.topics),
            "required_topics": self.required_topics(),
            "topics_with_limits": sorted(n for n, p in self.topics.items()
                                         if p.limits),
            "core_loss_allowed": self.core_loss_allowed,
            "min_cores": self.min_cores,
            "errors": list(self.errors),
        }


def _load_text(path: str) -> tuple[dict, list]:
    """YAML first, JSON otherwise. Failures come back as a reason list, not an
    exception."""
    errors: list = []
    try:
        with open(path, "r", encoding="utf-8") as fp:
            text = fp.read()
    except OSError as exc:
        return {}, [f"{path}: {exc}"]

    try:
        import yaml            # noqa: PLC0415
        data = yaml.safe_load(text)
        return (data or {}), errors
    except ImportError:
        errors.append("no PyYAML — trying to parse as JSON")
    except Exception as exc:    # yaml.YAMLError and friends
        return {}, [f"{path}: YAML parse failed, {exc}"]

    try:
        return (json.loads(text) or {}), errors
    except (json.JSONDecodeError, ValueError) as exc:
        return {}, errors + [f"{path}: JSON parse failed, {exc}"]


def parse_policy(data: dict, source: str = "") -> Policy:
    """dict -> :class:`Policy`. Unknown keys are ignored, with a reason recorded."""
    pol = Policy(source=source)
    if not isinstance(data, dict):
        pol.errors.append("top level is not a mapping — using defaults")
        return pol

    th = data.get("thresholds") or {}
    if isinstance(th, dict):
        known = Threshold.__dataclass_fields__
        for key, value in th.items():
            if key not in known:
                pol.errors.append(f"thresholds.{key}: unknown entry")
                continue
            try:
                setattr(pol.thresholds, key, type(getattr(pol.thresholds, key))(value))
            except (TypeError, ValueError):
                pol.errors.append(f"thresholds.{key}: cannot parse value ({value!r})")

    for entry in data.get("nodes") or []:
        if isinstance(entry, str):
            pol.nodes[entry] = NodePolicy(name=entry, critical=True)
            continue
        if not isinstance(entry, dict) or not entry.get("name"):
            pol.errors.append(f"nodes: ignoring an entry with no name ({entry!r})")
            continue
        name = str(entry["name"])
        pol.nodes[name] = NodePolicy(
            name=name,
            critical=bool(entry.get("critical", True)),
            restart=str(entry.get("restart") or ""),
            stateful=bool(entry.get("stateful", False)),
            max_restarts=int(entry.get("max_restarts", 3) or 3))

    topics = data.get("topics") or {}
    if isinstance(topics, dict):
        for name, spec in topics.items():
            spec = spec if isinstance(spec, dict) else {}
            limits = spec.get("limits") or {}
            pol.topics[str(name)] = TopicPolicy(
                name=str(name),
                required=bool(spec.get("required", False)),
                min_hz=float(spec.get("min_hz", 0.0) or 0.0),
                limits=limits if isinstance(limits, dict) else {})
    elif topics:
        pol.errors.append("topics: must be a mapping")

    tol = data.get("tolerances") or {}
    if isinstance(tol, dict):
        pol.core_loss_allowed = bool(tol.get("core_loss", True))
        try:
            pol.min_cores = int(tol.get("min_cores", 2))
        except (TypeError, ValueError):
            pol.errors.append("tolerances.min_cores: cannot parse value")
    if "armed" in data:
        pol.armed = bool(data["armed"])
    return pol


def load_policy(path: str = "") -> Policy:
    """Read the configuration. **With none present it runs quietly on the defaults.**

    Parse errors are not swallowed: they land in `errors` and go into the startup log —
    a threshold silently ignored because of one typo leaves nobody able to find the cause
    later.
    """
    candidates = [path] if path else list(DEFAULT_PATHS)
    for candidate in candidates:
        if candidate and os.path.exists(candidate):
            data, errors = _load_text(candidate)
            pol = parse_policy(data, source=candidate)
            pol.errors = errors + pol.errors
            return pol
    pol = Policy()
    if path:
        pol.errors.append(f"{path}: no such file — using defaults")
    return pol


#: The example shipped with the deployment. Not one entry is mandatory.
EXAMPLE_YAML = """\
# ras-fault-monitor policy — every entry is optional.
# General threat monitoring works exactly the same without this file.

thresholds:
  disk_used_warn: 0.80        # earlier on a box that logs heavily
  mem_available_warn: 0.15
  temp_warn_c: 70.0           # lower it for a sealed enclosure

# Which nodes are critical — knowledge observation cannot supply.
nodes:
  - name: cmd_vel_mux
    critical: true
    restart: systemctl restart nav-cmd-mux
  - name: imu_node
    critical: true
    stateful: true            # holds calibration -> never restarted
  - name: diagnostics_agg
    critical: false

# Topic expectations. min_hz and limits are the grounds for a Tier 3 verdict.
topics:
  /cmd_vel:
    required: true
    min_hz: 10
  /sensor/imu:
    min_hz: 50
    limits:
      # This alone can claim something is physically wrong.
      # Widen the range for an airframe that goes into free fall.
      linear_acceleration.z: [8.0, 12.0]

tolerances:
  core_loss: true             # true when there is no real-time deadline
  min_cores: 4
"""
