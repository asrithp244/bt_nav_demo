"""
Unit tests for scenarios.yaml structure.
Validates that all required fields are present and values are in sane ranges.
No ROS 2 runtime required — pure Python YAML parsing.
"""
import os
import yaml
import pytest


SCENARIOS_PATH = os.path.join(
    os.path.dirname(__file__),
    "..", "config", "scenarios.yaml"
)


@pytest.fixture(scope="module")
def scenarios():
    with open(SCENARIOS_PATH, "r") as f:
        data = yaml.safe_load(f)
    return data["scenarios"]


def test_scenarios_loaded(scenarios):
    assert len(scenarios) >= 8, "Expected at least 8 scenarios"


def test_each_scenario_has_required_fields(scenarios):
    required = ["id", "goal_pose", "timeout_sec", "success_threshold_m"]
    for s in scenarios:
        for field in required:
            assert field in s, f"Scenario '{s.get('id', '?')}' missing '{field}'"


def test_ids_are_unique(scenarios):
    ids = [s["id"] for s in scenarios]
    assert len(ids) == len(set(ids)), "Duplicate scenario IDs detected"


def test_goal_poses_have_x_y(scenarios):
    for s in scenarios:
        gp = s["goal_pose"]
        assert "x" in gp and "y" in gp, \
            f"Scenario '{s['id']}' goal_pose missing x or y"


def test_timeout_is_positive(scenarios):
    for s in scenarios:
        assert s["timeout_sec"] > 0, \
            f"Scenario '{s['id']}' has non-positive timeout"


def test_success_threshold_is_positive(scenarios):
    for s in scenarios:
        assert s["success_threshold_m"] > 0, \
            f"Scenario '{s['id']}' has non-positive success_threshold_m"


def test_obstacles_have_required_fields(scenarios):
    for s in scenarios:
        for i, obs in enumerate(s.get("obstacles", [])):
            assert "pose" in obs, \
                f"Scenario '{s['id']}' obstacle[{i}] missing 'pose'"
            assert "size" in obs, \
                f"Scenario '{s['id']}' obstacle[{i}] missing 'size'"
            assert "x" in obs["pose"] and "y" in obs["pose"], \
                f"Scenario '{s['id']}' obstacle[{i}] pose missing x or y"
            assert "w" in obs["size"] and "h" in obs["size"], \
                f"Scenario '{s['id']}' obstacle[{i}] size missing w or h"


def test_obstacle_sizes_are_positive(scenarios):
    for s in scenarios:
        for i, obs in enumerate(s.get("obstacles", [])):
            assert obs["size"]["w"] > 0, \
                f"Scenario '{s['id']}' obstacle[{i}] size.w <= 0"
            assert obs["size"]["h"] > 0, \
                f"Scenario '{s['id']}' obstacle[{i}] size.h <= 0"


def test_unreachable_scenario_exists(scenarios):
    """s009 must exist to test graceful failure handling."""
    ids = [s["id"] for s in scenarios]
    assert any("unreachable" in sid for sid in ids), \
        "No goal_unreachable scenario found — required for failure regression"


def test_baseline_scenario_has_no_obstacles(scenarios):
    """s010 (clear baseline) must have zero obstacles."""
    baselines = [s for s in scenarios if "baseline" in s["id"]]
    assert baselines, "No baseline (no-obstacle) scenario found"
    for b in baselines:
        assert len(b.get("obstacles", [])) == 0, \
            f"Baseline scenario '{b['id']}' should have no obstacles"
