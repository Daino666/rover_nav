"""Regression tests for the planner-facing /obstacles/array output.

The detector works natively in camera_depth_optical_frame (X right, Y DOWN,
Z forward); cmd_vel_arbiter.py plans in odom (X forward, Y left, Z up). Getting
that remap wrong is silent -- obstacles simply appear in the wrong place -- so
it is pinned down here rather than eyeballed in RViz.
"""

import importlib.util
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time as TimeMsg


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

MODULE_PATH = SCRIPTS / "Rover_path_controller" / "pcl_obstacle_detector.py"
SPEC = importlib.util.spec_from_file_location("rover_pcl_obstacle_detector", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


# camera_depth_optical_frame -> base/odom, i.e. rpy(-90, 0, -90):
#   optical +Z (forward) -> +X,  optical +X (right) -> -Y,  optical +Y (down) -> -Z
OPTICAL_TO_BASE_QUAT = (-0.5, 0.5, -0.5, 0.5)


def _transform(quat=OPTICAL_TO_BASE_QUAT, xyz=(0.0, 0.0, 0.0)):
    x, y, z, w = quat
    return SimpleNamespace(transform=SimpleNamespace(
        rotation=SimpleNamespace(x=x, y=y, z=z, w=w),
        translation=SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2]),
    ))


class _FakeBuffer:
    """Minimal tf2 Buffer: hands back one transform, or always raises."""

    def __init__(self, transform=None):
        self._transform = transform
        self.calls = 0

    def lookup_transform(self, target, source, when):
        self.calls += 1
        if self._transform is None:
            raise MODULE.tf2_ros.LookupException("no such transform")
        return self._transform


class _Recorder:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


@pytest.fixture
def detector(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    rclpy.init()
    node = MODULE.ObstacleDetector()
    node._array_pub = _Recorder()
    node._flag_pub = _Recorder()
    node._markers_pub = _Recorder()
    node._stats_pub = _Recorder()
    try:
        yield node
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _header(frame_id="camera_depth_optical_frame"):
    # A real Time message, not a stand-in: rclpy.time.Time.from_msg type-checks
    # its argument, so a SimpleNamespace here would test nothing but the fake.
    return SimpleNamespace(frame_id=frame_id, stamp=TimeMsg(sec=0, nanosec=0))


def test_forward_in_optical_becomes_forward_in_odom(detector):
    """A rock 2 m straight ahead must land at odom x=+2, not y or z."""
    detector._tf_buffer = _FakeBuffer(_transform())
    cluster = np.array([[0.0, 0.0, 2.0]])          # optical: 2 m forward
    detector._publish_array([cluster], _header())

    (msg,) = detector._array_pub.published
    (obs,) = msg.obstacles
    assert obs.x == pytest.approx(2.0, abs=1e-6)
    assert obs.y == pytest.approx(0.0, abs=1e-6)


def test_right_in_optical_becomes_negative_y_in_odom(detector):
    """Optical +X is to the right, which is odom -Y. Sign errors here steer
    the rover into the obstacle it is trying to avoid."""
    detector._tf_buffer = _FakeBuffer(_transform())
    cluster = np.array([[1.0, 0.0, 2.0]])          # 2 m ahead, 1 m to the right
    detector._publish_array([cluster], _header())

    obs = detector._array_pub.published[0].obstacles[0]
    assert obs.x == pytest.approx(2.0, abs=1e-6)
    assert obs.y == pytest.approx(-1.0, abs=1e-6)


def test_translation_is_applied(detector):
    detector._tf_buffer = _FakeBuffer(_transform(xyz=(0.2756, 0.0, 0.489)))
    cluster = np.array([[0.0, 0.0, 2.0]])
    detector._publish_array([cluster], _header())

    obs = detector._array_pub.published[0].obstacles[0]
    assert obs.x == pytest.approx(2.0 + 0.2756, abs=1e-6)


def test_radius_is_horizontal_and_ignores_height(detector):
    """Radius is a footprint for planning, so a tall thin post must stay thin."""
    detector._tf_buffer = _FakeBuffer(_transform())
    # A 1 m vertical column (optical -Y is up) with a 0.2 m horizontal spread.
    cluster = np.array([
        [-0.1, 0.0, 2.0],
        [0.1, 0.0, 2.0],
        [0.0, -1.0, 2.0],
        [0.0, 0.0, 2.0],
    ])
    detector._publish_array([cluster], _header())

    obs = detector._array_pub.published[0].obstacles[0]
    assert obs.radius == pytest.approx(0.1, abs=1e-6)


def test_missing_transform_publishes_nothing_rather_than_wrong_coordinates(detector):
    detector._tf_buffer = _FakeBuffer(None)
    detector._publish_array([np.array([[0.0, 0.0, 2.0]])], _header())
    assert detector._array_pub.published == []


def test_missing_transform_does_not_suppress_the_stop_flag(detector):
    """The stop-gate rides on the Bool. A TF outage must not disarm the brake."""
    detector._tf_buffer = _FakeBuffer(None)
    detector._publish([np.array([[0.0, 0.0, 2.0]])], _header())

    assert detector._array_pub.published == []
    assert detector._flag_pub.published[0].data is True


def test_no_clusters_publishes_an_empty_array_without_needing_tf(detector):
    """An empty array is a positive statement of 'nothing there', and must not
    depend on a transform that may be missing."""
    buf = _FakeBuffer(None)
    detector._tf_buffer = buf
    detector._publish_array([], _header())

    (msg,) = detector._array_pub.published
    assert list(msg.obstacles) == []
    assert buf.calls == 0


def test_empty_output_frame_disables_the_array(detector):
    detector.set_parameters([rclpy.parameter.Parameter(
        "output_frame", rclpy.Parameter.Type.STRING, "")])
    detector._tf_buffer = _FakeBuffer(_transform())
    detector._publish_array([np.array([[0.0, 0.0, 2.0]])], _header())
    assert detector._array_pub.published == []
