#!/usr/bin/env python3
import math
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class VirtualDifferential(Node):
    """
    Software mechanical differential: always drives (L + R) / 2 → 0
    while preserving the terrain-induced differential (L - R) / 2.
      L_cmd = +diff_meas  →  /aries/L_Rocker_Joint/cmd_pos
      R_cmd = -diff_meas  →  /aries/R_Rocker_Joint/cmd_pos
    Uses the existing Gazebo JointPositionController + ROS-GZ bridge.
    """

    def __init__(self):
        super().__init__('virtual_differential')

        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('left_joint_name',    'L_Rocker_Joint')
        self.declare_parameter('right_joint_name',   'R_Rocker_Joint')
        self.declare_parameter('left_cmd_topic',     '/aries/L_Rocker_Joint/cmd_pos')
        self.declare_parameter('right_cmd_topic',    '/aries/R_Rocker_Joint/cmd_pos')
        self.declare_parameter('cmd_rate_hz',        50.0)
        self.declare_parameter('cmd_limit_deg',      18.5)
        self.declare_parameter('rate_limit_deg_s',   60.0)

        js_topic        = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        self.left_name  = self.get_parameter('left_joint_name').get_parameter_value().string_value
        self.right_name = self.get_parameter('right_joint_name').get_parameter_value().string_value
        left_cmd_topic  = self.get_parameter('left_cmd_topic').get_parameter_value().string_value
        right_cmd_topic = self.get_parameter('right_cmd_topic').get_parameter_value().string_value

        self.rate_hz    = self.get_parameter('cmd_rate_hz').get_parameter_value().double_value
        self.cmd_limit  = math.radians(self.get_parameter('cmd_limit_deg').get_parameter_value().double_value)
        self.rate_limit = math.radians(self.get_parameter('rate_limit_deg_s').get_parameter_value().double_value)

        self._pos: Dict[str, float] = {}
        self._last_l: Optional[float] = None
        self._last_r: Optional[float] = None

        self.sub_js   = self.create_subscription(JointState, js_topic, self.on_js, 50)
        self.pub_l    = self.create_publisher(Float64, left_cmd_topic,  10)
        self.pub_r    = self.create_publisher(Float64, right_cmd_topic, 10)
        self.timer    = self.create_timer(1.0 / self.rate_hz, self.update)

        self.get_logger().info(
            f'VirtualDifferential started — mimic via {left_cmd_topic} / {right_cmd_topic}')

    def on_js(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in (self.left_name, self.right_name):
                self._pos[name] = float(pos)

    def _clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def _rate_limit(self, target, last):
        if last is None:
            return target
        step = self.rate_limit / self.rate_hz
        return self._clamp(target, last - step, last + step)

    def update(self):
        if self.left_name not in self._pos or self.right_name not in self._pos:
            return

        diff_meas = 0.5 * (self._pos[self.left_name] - self._pos[self.right_name])

        l_cmd = self._rate_limit(+diff_meas, self._last_l)
        r_cmd = self._rate_limit(-diff_meas, self._last_r)

        l_cmd = self._clamp(l_cmd, -self.cmd_limit, self.cmd_limit)
        r_cmd = self._clamp(r_cmd, -self.cmd_limit, self.cmd_limit)

        self._last_l = l_cmd
        self._last_r = r_cmd

        self.pub_l.publish(Float64(data=l_cmd))
        self.pub_r.publish(Float64(data=r_cmd))


def main():
    rclpy.init()
    node = VirtualDifferential()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
