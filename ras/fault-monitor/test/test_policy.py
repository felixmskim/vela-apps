# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""The configuration surface — the one place an integrator injects application knowledge.

Its most important property: **it works with no configuration.** Making a config file
mandatory makes "just start it and see" impossible, and then nobody starts it.
"""

import json

from ras_fault_monitor.policy import (
    EXAMPLE_YAML, NodePolicy, Policy, TopicPolicy, load_policy, parse_policy,
)


# --- works with no configuration --------------------------------------------

def test_absent_policy_yields_safe_defaults(tmp_path):
    pol = load_policy(str(tmp_path / "nope.yaml"))
    assert isinstance(pol, Policy)
    assert pol.nodes == {} and pol.topics == {}
    assert pol.thresholds.disk_used_warn == 0.85      # the general default
    assert pol.errors                                  # the reason is recorded


def test_unknown_node_is_not_assumed_critical():
    """Treating the unknown as critical makes every node death CRITICAL and kills the
    severity scale."""
    pol = Policy()
    assert pol.is_critical("whatever") is False
    assert pol.recovery_for("whatever") == ""


# --- parsing ----------------------------------------------------------------

def test_threshold_override():
    pol = parse_policy({"thresholds": {"disk_used_warn": 0.7,
                                       "temp_warn_c": 65}})
    assert pol.thresholds.disk_used_warn == 0.7
    assert pol.thresholds.temp_warn_c == 65.0
    assert pol.thresholds.disk_used_crit == 0.95       # untouched values stay default


def test_unknown_threshold_key_is_surfaced():
    """A threshold ignored over one typo, with nothing shown, leaves the cause
    unfindable."""
    pol = parse_policy({"thresholds": {"disk_used_wrn": 0.7}})
    assert any("disk_used_wrn" in e for e in pol.errors)


def test_bad_threshold_value_is_surfaced():
    pol = parse_policy({"thresholds": {"disk_used_warn": "lots"}})
    assert any("disk_used_warn" in e for e in pol.errors)
    assert pol.thresholds.disk_used_warn == 0.85       # the default is kept


def test_node_shorthand_string_means_critical():
    pol = parse_policy({"nodes": ["cmd_vel_mux"]})
    assert pol.is_critical("cmd_vel_mux") is True


def test_stateful_node_is_not_restartable():
    """A node that holds state gets worse when restarted."""
    pol = parse_policy({"nodes": [
        {"name": "imu_node", "stateful": True, "restart": "systemctl restart imu"}]})
    assert pol.is_critical("imu_node") is True
    assert pol.recovery_for("imu_node") == ""          # no restart mechanism is handed out


def test_restartable_node_exposes_command():
    pol = parse_policy({"nodes": [
        {"name": "mux", "restart": "systemctl restart nav-mux"}]})
    assert pol.recovery_for("mux") == "systemctl restart nav-mux"


def test_node_without_name_is_skipped_with_reason():
    pol = parse_policy({"nodes": [{"critical": True}]})
    assert pol.nodes == {} and any("name" in e for e in pol.errors)


def test_topics_required_and_limits():
    pol = parse_policy({"topics": {
        "/cmd_vel": {"required": True, "min_hz": 10},
        "/sensor/imu": {"limits": {"linear_acceleration.z": [8.0, 12.0]}}}})
    assert pol.required_topics() == ["/cmd_vel"]
    assert pol.topic("/sensor/imu").limits


def test_topics_wrong_shape_is_surfaced():
    pol = parse_policy({"topics": ["/a", "/b"]})
    assert pol.topics == {} and any("mapping" in e for e in pol.errors)


def test_tolerances():
    pol = parse_policy({"tolerances": {"core_loss": False, "min_cores": 6}})
    assert pol.core_loss_allowed is False and pol.min_cores == 6


def test_non_mapping_top_level_falls_back():
    pol = parse_policy(["not", "a", "map"])
    assert pol.nodes == {} and pol.errors


# --- Tier 3 verdicts — the only ones claiming something is wrong ------------

def test_declared_limit_violation():
    tp = TopicPolicy(name="/imu", limits={"linear_acceleration.z": [8.0, 12.0]})
    got = tp.violated_limits({"linear_acceleration.z": 48.0})
    assert got and got[0][0] == "declared_limit"
    assert "48.0" in got[0][1]


def test_within_declared_limit_is_quiet():
    tp = TopicPolicy(name="/imu", limits={"linear_acceleration.z": [8.0, 12.0]})
    assert tp.violated_limits({"linear_acceleration.z": 9.81}) == []


def test_missing_field_makes_no_limit_claim():
    """With no value, no verdict."""
    tp = TopicPolicy(name="/imu", limits={"a.b": [0, 1]})
    assert tp.violated_limits({"other": 5}) == []


def test_malformed_bounds_ignored():
    tp = TopicPolicy(name="/imu", limits={"x": ["low", "high"], "y": [1]})
    assert tp.violated_limits({"x": 5.0, "y": 5.0}) == []


# --- file reading -----------------------------------------------------------

def test_json_policy_loaded(tmp_path):
    path = tmp_path / "policy.json"
    path.write_text(json.dumps({"nodes": [{"name": "a", "critical": True}],
                                "tolerances": {"min_cores": 5}}))
    pol = load_policy(str(path))
    assert pol.is_critical("a") and pol.min_cores == 5
    assert pol.source == str(path)


def test_malformed_file_does_not_raise(tmp_path):
    path = tmp_path / "policy.json"
    path.write_text("{ this is not valid")
    pol = load_policy(str(path))
    assert pol.errors and pol.nodes == {}        # keeps running on the defaults


def test_example_yaml_is_parseable_as_documented():
    """An example that does not parse makes the documentation a lie."""
    try:
        import yaml
    except ImportError:
        return
    pol = parse_policy(yaml.safe_load(EXAMPLE_YAML), source="example")
    assert pol.is_critical("cmd_vel_mux")
    assert pol.recovery_for("cmd_vel_mux")
    assert pol.recovery_for("imu_node") == ""    # stateful
    assert pol.is_critical("diagnostics_agg") is False
    assert pol.required_topics() == ["/cmd_vel"]
    assert pol.topic("/sensor/imu").limits["linear_acceleration.z"] == [8.0, 12.0]
    assert pol.thresholds.disk_used_warn == 0.80
    assert pol.min_cores == 4
    assert pol.errors == []                      # the example must carry no typo


def test_summary_shape():
    pol = parse_policy({"nodes": [{"name": "a", "restart": "x"}],
                        "topics": {"/t": {"required": True}}})
    s = pol.summary()
    assert s["critical_nodes"] == ["a"] and s["restartable_nodes"] == ["a"]
    assert s["required_topics"] == ["/t"]
