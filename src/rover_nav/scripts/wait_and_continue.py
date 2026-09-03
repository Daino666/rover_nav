#!/usr/bin/env python3
"""Press Enter in THIS terminal each time the rover stops at a waypoint, to
send it on to the next one. Pairs with cmd_vel_arbiter.py's
stop_at_waypoints:=true (see full_hardware.launch.py's argument of the same
name) -- run this alongside it, in its own terminal.

  ros2 run rover_nav wait_and_continue.py

Ctrl+C to stop watching (does not stop the rover -- use
`ros2 service call /planner/stop std_srvs/srv/Trigger` for that).
"""

import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main(args=None):
    rclpy.init(args=args)
    node = Node("wait_and_continue")
    client = node.create_client(Trigger, "/planner/continue")

    print("Press Enter each time you want the rover to continue to the next "
          "waypoint (Ctrl+C to stop watching).")
    try:
        while rclpy.ok():
            line = sys.stdin.readline()
            if line == "":   # stdin closed (EOF), not just an empty line
                break
            if not client.wait_for_service(timeout_sec=2.0):
                print("  /planner/continue not available -- is cmd_vel_arbiter "
                      "running with stop_at_waypoints:=true?")
                continue
            fut = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
            result = fut.result()
            print(f"  {result.message}" if result is not None else "  no response")
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
