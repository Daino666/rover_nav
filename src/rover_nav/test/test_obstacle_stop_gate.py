"""Regression tests for the forward-obstacle stop-gate in cmd_vel_arbiter.

The gate's whole value is that it fails CLOSED, so most of what is worth
testing here is the ways it must refuse to drive -- a detector that never
started, one that went silent, and a single clear frame in the middle of a
real obstacle.
"""

import importlib.util
import sys
from pathlib import Path

import pytest
import rclpy


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

MODULE_PATH = SCRIPTS / "cmd_vel_arbiter.py"
SPEC = importlib.util.spec_from_file_location("rover_cmd_vel_arbiter", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class _Bool:
    """Stand-in for std_msgs/Bool -- the callback only reads .data."""

    def __init__(self, data):
        self.data = data


NOW = 1000.0


@pytest.fixture
def arbiter(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    rclpy.init()
    node = MODULE.CmdVelArbiter()

    # Satisfy every gate ahead of the obstacle check, so a refusal in these
    # tests can only have come from the obstacle gate itself.
    node.conflict = False
    node.estopped = False
    node.run_started = True
    node.waypoints = [[1.0, 0.0]]
    node.car_pos = [0.0, 0.0]
    node.car_yaw = 0.0
    node.last_odom_time = NOW
    node.drive_enabled = True
    node.drive_enabled_seen = True
    try:
        yield node
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_holds_until_a_detector_is_actually_seen(arbiter):
    """No detector must never read as 'clear' -- this is the fail-closed case."""
    allowed, reason = arbiter._safety_gate(NOW)
    assert allowed is False
    assert "no obstacle detector" in reason
    # and the reason has to say how to proceed deliberately
    assert "obstacle_stop_enabled" in reason


def test_clear_detector_allows_driving(arbiter):
    arbiter._on_obstacle(_Bool(False))
    arbiter.last_obstacle_time = NOW
    arbiter.obstacle_clear_since = NOW - arbiter.obstacle_clear_s
    arbiter._on_obstacle(_Bool(False))
    arbiter.last_obstacle_time = NOW
    assert arbiter._safety_gate(NOW) == (True, "")


def test_obstacle_blocks_immediately(arbiter):
    arbiter._on_obstacle(_Bool(True))
    arbiter.last_obstacle_time = NOW
    allowed, reason = arbiter._safety_gate(NOW)
    assert allowed is False
    assert reason == "obstacle ahead"


def test_silent_detector_goes_stale_rather_than_clear(arbiter):
    """A detector that crashes mid-run must brake, not free the rover."""
    arbiter._on_obstacle(_Bool(False))
    arbiter.obstacle_blocked = False
    arbiter.last_obstacle_time = NOW

    stale = NOW + arbiter.obstacle_timeout_s + 0.01
    allowed, reason = arbiter._safety_gate(stale)
    assert allowed is False
    assert "stale" in reason


def test_one_clear_frame_does_not_release_the_brake(arbiter):
    """The detector is per-frame with no temporal filter; a dropout mid-rock
    must not lurch the rover forward."""
    arbiter._on_obstacle(_Bool(True))
    assert arbiter.obstacle_blocked is True

    arbiter._on_obstacle(_Bool(False))     # single clear frame, immediately after
    assert arbiter.obstacle_blocked is True, "released on one clear frame"

    # Only a sustained run of clear frames may release it.
    arbiter.obstacle_clear_since -= arbiter.obstacle_clear_s
    arbiter._on_obstacle(_Bool(False))
    assert arbiter.obstacle_blocked is False


def test_reblocking_resets_the_clear_timer(arbiter):
    arbiter._on_obstacle(_Bool(False))
    arbiter._on_obstacle(_Bool(True))
    assert arbiter.obstacle_clear_since is None
    assert arbiter.obstacle_blocked is True


def test_disabled_gate_never_blocks(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    rclpy.init()
    node = MODULE.CmdVelArbiter()
    try:
        node.obstacle_stop_enabled = False
        # Even with a latched obstacle and no detector heartbeat at all.
        node.obstacle_blocked = True
        node.obstacle_seen = False
        assert node._obstacle_gate(NOW) == (True, "")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
