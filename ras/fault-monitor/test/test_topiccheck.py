# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""Topic anomaly verdicts, on the premise that which sensors will run is unknown.

The point is that **the evidence grades never blur**. "the message violated its own
contract" (certain) and "this differs from usual" (statistical) are different claims, and
dressing the latter as the former makes the verdict a lie.
"""

import math

from ras_fault_monitor.topiccheck import (
    NAVSAT_NO_FIX, TIER_BASELINE, TIER_SELF_DECLARED, TIER_STRUCTURAL,
    TopicMonitor, check_self_declared,
)

LASER = "sensor_msgs/msg/LaserScan"
IMU = "sensor_msgs/msg/Imu"
NAVSAT = "sensor_msgs/msg/NavSatFix"
PC2 = "sensor_msgs/msg/PointCloud2"
IMAGE = "sensor_msgs/msg/Image"
CUSTOM = "my_pkg/msg/Weird"


def codes(result):
    return {c for c, _ in result}


# --- Tier 1: common checks that hold without knowing the type ---------------

def test_nan_is_invalid_for_any_type():
    """A NaN physical quantity is invalid regardless of the app."""
    assert "nan_inf" in codes(check_self_declared(
        CUSTOM, {"whatever": float("nan")}))
    assert "nan_inf" in codes(check_self_declared(
        IMU, {"linear_acceleration.z": float("inf")}))


def test_all_nan_array_distinguished_from_partial():
    all_bad = check_self_declared(LASER, {"ranges": [float("nan")] * 4})
    part = check_self_declared(LASER, {"ranges": [1.0, float("nan"), 2.0]})
    assert "all_nan_inf" in codes(all_bad)
    assert "nan_inf_partial" in codes(part)


def test_zero_timestamp_flagged():
    assert "zero_stamp" in codes(check_self_declared(IMU, {"header.stamp_sec": 0}))


def test_healthy_message_yields_nothing():
    assert check_self_declared(IMU, {"header.stamp_sec": 1700,
                                     "linear_acceleration.z": 9.81}) == []


# --- Tier 1: the range the message declares about itself --------------------

def test_laserscan_uses_its_own_declared_range():
    """No sensor knowledge needed — range_min/range_max live in the message."""
    got = check_self_declared(LASER, {
        "range_min": 0.1, "range_max": 10.0,
        "ranges": [0.5, 99.0, 2.0],           # 99.0 is outside the self-declared range
    })
    assert "out_of_declared_range" in codes(got)


def test_laserscan_within_declared_range_is_quiet():
    assert check_self_declared(LASER, {
        "range_min": 0.1, "range_max": 10.0, "ranges": [0.5, 9.9]}) == []


def test_laserscan_inf_is_not_range_violation():
    """In LaserScan, inf is the "no measurement" convention, not a range violation."""
    got = check_self_declared(LASER, {
        "range_min": 0.1, "range_max": 10.0, "ranges": [float("inf"), 1.0]})
    assert "out_of_declared_range" not in codes(got)


def test_empty_pointcloud_is_hardware_fault():
    """The "empty point cloud" of deck scenario 1."""
    got = check_self_declared(PC2, {"width": 0, "height": 0,
                                    "point_step": 16, "data_len": 0})
    assert "empty_payload" in codes(got)


def test_pointcloud_size_inconsistency():
    got = check_self_declared(PC2, {"width": 10, "height": 1,
                                    "point_step": 16, "data_len": 99})
    assert "array_size_mismatch" in codes(got)


def test_image_size_inconsistency():
    got = check_self_declared(IMAGE, {"height": 480, "step": 1920,
                                      "data_len": 1000})
    assert "array_size_mismatch" in codes(got)


def test_navsat_no_fix_is_self_declared():
    got = check_self_declared(NAVSAT, {"status.status": NAVSAT_NO_FIX})
    assert "no_fix" in codes(got)
    assert check_self_declared(NAVSAT, {"status.status": 0}) == []


def test_laserscan_array_size_vs_angles():
    got = check_self_declared(LASER, {
        "range_min": 0.0, "range_max": 100.0,
        "angle_min": 0.0, "angle_max": 1.0, "angle_increment": 0.1,
        "ranges": [1.0] * 3})                 # the angles imply 11 entries
    assert "array_size_mismatch" in codes(got)


def test_battery_health_enum():
    assert "battery_health" in codes(check_self_declared(
        "sensor_msgs/msg/BatteryState", {"power_supply_health": 3}))
    # 0=UNKNOWN defers the verdict, 1=GOOD is fine
    assert check_self_declared("sensor_msgs/msg/BatteryState",
                               {"power_supply_health": 0}) == []
    assert check_self_declared("sensor_msgs/msg/BatteryState",
                               {"power_supply_health": 1}) == []


def test_unknown_type_still_gets_common_checks():
    """A custom message still gets the NaN check."""
    got = check_self_declared(CUSTOM, {"x": float("nan"), "header.stamp_sec": 0})
    assert codes(got) == {"nan_inf", "zero_stamp"}


# --- Tier 0: publication stopped (no message arriving is the point) --------

def _warm(mon, topic, type_name, period, n, t0=1000.0, size=100):
    """Feed enough normal publications for a baseline to lock."""
    t = t0
    for _ in range(n):
        mon.on_message(topic, type_name, t, size, {"header.stamp_sec": int(t)})
        t += period
    return t


def test_deadline_miss_after_silence():
    """Deck scenario 2 — the camera stopped publishing."""
    mon = TopicMonitor(warmup_n=10, deadline_factor=5.0, min_deadline_s=0.0)
    t = _warm(mon, "/sensor/camera", IMAGE, 0.2, 12)
    assert not mon.check_deadlines(t + 0.5)          # still within the allowance
    faults = mon.check_deadlines(t + 3.0)            # past 0.2*5=1.0s
    assert len(faults) == 1
    f = faults[0]
    assert f.fault_type == "DDS_FAULT"
    assert TIER_STRUCTURAL in f.tags
    assert "code:comm_deadline" in f.tags
    assert f.topic_name == "/sensor/camera"


def test_deadline_reported_once_until_next_message():
    mon = TopicMonitor(warmup_n=5, deadline_factor=2.0, min_deadline_s=0.0)
    t = _warm(mon, "/a", IMU, 0.1, 6)
    assert len(mon.check_deadlines(t + 10)) == 1
    assert len(mon.check_deadlines(t + 20)) == 0     # no duplicate report
    mon.on_message("/a", IMU, t + 21, 100, {"header.stamp_sec": 1})
    assert len(mon.check_deadlines(t + 40)) == 1     # a recurrence reports again


def test_no_baseline_means_no_deadline_claim():
    """Too few observations means the period is unknown, so no deadline is claimed."""
    mon = TopicMonitor(warmup_n=50)
    _warm(mon, "/a", IMU, 0.1, 5)
    assert mon.check_deadlines(1e9) == []


def test_min_deadline_prevents_absurd_thresholds():
    """Faulting a 1 ms topic after 5 ms of silence turns the log into garbage."""
    mon = TopicMonitor(warmup_n=5, deadline_factor=5.0, min_deadline_s=1.0)
    t = _warm(mon, "/fast", IMU, 0.001, 6)
    assert mon.check_deadlines(t + 0.5) == []       # the floor is 1s, not 0.005s
    assert len(mon.check_deadlines(t + 2.0)) == 1


# --- Tier 2: the baseline and its trap --------------------------------------

def test_size_deviation_is_baseline_tier_not_self_declared():
    """Recording "differs from usual" as "the message violated" would be a lie."""
    mon = TopicMonitor(warmup_n=10, size_tolerance=2.0)
    _warm(mon, "/a", IMU, 0.1, 12, size=100)
    faults = mon.on_message("/a", IMU, 2000.0, 5000, {"header.stamp_sec": 1})
    dev = [f for f in faults if "code:size_deviation" in f.tags]
    assert len(dev) == 1
    assert TIER_BASELINE in dev[0].tags
    assert TIER_SELF_DECLARED not in dev[0].tags
    assert dev[0].actual_value == 5000.0


def test_baseline_rejected_when_warmup_had_violations():
    """Learning a broken warmup as normal makes every later verdict meaningless."""
    mon = TopicMonitor(warmup_n=5)
    t = 1000.0
    for _ in range(6):
        mon.on_message("/bad", IMU, t, 100, {"linear_acceleration.z": float("nan")})
        t += 0.1
    st = mon.topics["/bad"]
    assert st.has_baseline is False
    assert st.baseline_attempts >= 1
    assert "Tier 0/1 violations" in st.baseline_rejected
    # with no baseline, no deadline is claimed either
    assert mon.check_deadlines(t + 100) == []


def test_clean_warmup_locks_baseline():
    mon = TopicMonitor(warmup_n=10)
    _warm(mon, "/ok", IMU, 0.25, 12)
    st = mon.topics["/ok"]
    assert st.has_baseline is True
    assert math.isclose(st.baseline_period, 0.25, rel_tol=0.05)


def test_raw_mode_skips_tier1_but_keeps_tier0():
    """With an unknown type (fields=None) no verdict is made on absent evidence."""
    mon = TopicMonitor(warmup_n=5, deadline_factor=2.0, min_deadline_s=0.0)
    t = 1000.0
    for _ in range(6):
        assert mon.on_message("/custom", CUSTOM, t, 50, None) == []
        t += 0.1
    assert len(mon.check_deadlines(t + 5)) == 1     # the period is still watched


def test_status_reports_baseline_state():
    mon = TopicMonitor(warmup_n=5)
    _warm(mon, "/a", IMU, 0.1, 6)
    st = mon.status()["/a"]
    assert st["has_baseline"] is True and st["type"] == IMU


def test_only_existing_fault_types():
    from ras_fault_monitor.fault import FAULT_TYPES
    mon = TopicMonitor(warmup_n=5, deadline_factor=2.0, min_deadline_s=0.0,
                       size_tolerance=1.5)
    t = _warm(mon, "/a", LASER, 0.1, 6)
    produced = mon.on_message("/a", LASER, t, 9999, {
        "range_min": 0.0, "range_max": 1.0, "ranges": [5.0],
        "header.stamp_sec": 0})
    produced += mon.check_deadlines(t + 100)
    assert produced
    for f in produced:
        assert f.fault_type in FAULT_TYPES, f.fault_type


# --- "unknown" vs "0" (regression for a measured false verdict) -------------

def test_missing_data_len_is_not_claimed_empty():
    """Calling a missing field (unknown) "empty" pollutes warmup."""
    assert check_self_declared(IMAGE, {"header.stamp_sec": 1700}) == []
    assert check_self_declared(PC2, {"header.stamp_sec": 1700}) == []


def test_explicit_zero_data_len_is_empty():
    assert "empty_payload" in codes(check_self_declared(IMAGE, {"data_len": 0}))
    assert "empty_payload" in codes(check_self_declared(PC2, {"data_len": 0}))


def test_image_warmup_not_poisoned_by_missing_fields():
    """This false verdict got every Image topic baseline refused."""
    mon = TopicMonitor(warmup_n=10, deadline_factor=5.0, min_deadline_s=0.0)
    _warm(mon, "/sensor/camera", IMAGE, 0.2, 12)
    assert mon.topics["/sensor/camera"].has_baseline is True


# --- baseline refusal must not be permanent (found in measurement) ----------

def test_rejected_baseline_is_retried_after_clean_window():
    """Attaching while the app was already misbehaving must not blind it forever."""
    mon = TopicMonitor(warmup_n=5, deadline_factor=2.0, min_deadline_s=0.0)
    t = 1000.0
    # (1) a dirty warmup — it must be refused
    for _ in range(6):
        mon.on_message("/t", IMU, t, 100, {"linear_acceleration.z": float("nan")})
        t += 0.1
    st = mon.topics["/t"]
    assert st.has_baseline is False
    assert st.baseline_attempts == 1
    assert "awaiting retry" in st.baseline_rejected

    # (2) once it runs clean, the baseline must lock again. The last message of the
    #     dirty phase leaves one violation in the next window, so adoption waits for
    #     **one fully clean window** — which is the intended strictness.
    for _ in range(12):
        mon.on_message("/t", IMU, t, 100, {"header.stamp_sec": int(t)})
        t += 0.2
    assert st.has_baseline is True
    assert st.baseline_attempts >= 2
    assert st.baseline_rejected == ""
    assert math.isclose(st.baseline_period, 0.2, rel_tol=0.2)
    # with a baseline in hand, a deadline claim becomes possible
    assert len(mon.check_deadlines(t + 10)) == 1


def test_repeated_dirty_warmup_keeps_retrying():
    mon = TopicMonitor(warmup_n=3)
    t = 1000.0
    for _ in range(9):
        mon.on_message("/t", IMU, t, 100, {"linear_acceleration.z": float("nan")})
        t += 0.1
    st = mon.topics["/t"]
    assert st.baseline_attempts >= 2      # it keeps retrying
    assert st.has_baseline is False


# --- repeat suppression (measured: 9,617 identical records buried the log) ---

def _bad_scan(n=5):
    return {"range_min": 0.1, "range_max": 10.0, "ranges": [99.0] * n}


def test_repeating_violation_is_reported_once():
    """A violating publisher violates on every message; the log must not.

    Measured on the VM: one 4 Hz talker left 9,617 identical records and every other
    event in the file became unfindable.
    """
    mon = TopicMonitor(warmup_n=3)
    t = 1000.0
    first = mon.on_message("/scan", LASER, t, 100, _bad_scan())
    assert len(first) == 1
    for _ in range(40):
        t += 0.25
        assert mon.on_message("/scan", LASER, t, 100, _bad_scan()) == []


def test_still_violating_reports_again_on_the_heartbeat():
    """A violation lasting hours must not become invisible."""
    mon = TopicMonitor(warmup_n=3, repeat_after_s=60.0)
    t = 1000.0
    assert len(mon.on_message("/scan", LASER, t, 100, _bad_scan())) == 1
    for _ in range(10):                      # inside the heartbeat window
        t += 1.0
        assert mon.on_message("/scan", LASER, t, 100, _bad_scan()) == []
    again = mon.on_message("/scan", LASER, t + 60.0, 100, _bad_scan())
    assert len(again) == 1
    assert "still_violating" in again[0].tags
    assert again[0].detail["occurrences"] >= 10      # what was folded is stated


def test_cleared_violation_is_recorded():
    """"Still broken" and "fixed itself" must be distinguishable in the log."""
    mon = TopicMonitor(warmup_n=3)
    t = 1000.0
    mon.on_message("/scan", LASER, t, 100, _bad_scan())
    for _ in range(5):
        t += 0.25
        mon.on_message("/scan", LASER, t, 100, _bad_scan())
    good = {"range_min": 0.1, "range_max": 10.0, "ranges": [1.0] * 5}
    cleared = mon.on_message("/scan", LASER, t + 0.25, 100, good)
    assert len(cleared) == 1
    assert "recovered" in cleared[0].tags
    assert cleared[0].severity == "INFO"
    assert cleared[0].detail["code"] == "out_of_declared_range"


def test_suppressed_violations_still_poison_the_warmup():
    """The trap: counting emissions instead of verdicts.

    Suppression must not make a violating warmup look clean — the baseline would then
    be learned from broken data and every later verdict would rest on it.
    """
    mon = TopicMonitor(warmup_n=5)
    t = 1000.0
    for _ in range(6):
        mon.on_message("/scan", LASER, t, 100, _bad_scan())
        t += 0.2
    st = mon.topics["/scan"]
    assert st.has_baseline is False
    assert "Tier 0/1 violations" in st.baseline_rejected
