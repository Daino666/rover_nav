#!/usr/bin/env python3
"""
Robust Lifecycle Node Activator for Nav2 Planning Stack

Ensures map_server, planner_server, and smoother_server are reliably transitioned
to the ACTIVE state using background worker threads without interfering with the ROS executor.
"""

import threading
import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State


class LifecycleAutostart(Node):
    def __init__(self):
        super().__init__('lifecycle_autostart')
        self.declare_parameter('node_names', ['map_server', 'planner_server', 'smoother_server'])
        self.node_names = self.get_parameter('node_names').value
        self.get_logger().info(f"🚀 Monitoring and activating Nav2 lifecycle nodes: {self.node_names}")
        
        self.worker_thread = threading.Thread(target=self._run_activator, daemon=True)
        self.worker_thread.start()

    def _run_activator(self):
        time.sleep(1.0)
        all_active = False

        while rclpy.ok() and not all_active:
            all_active = True
            for name in self.node_names:
                state = self._get_node_state(name)
                if state is None:
                    all_active = False
                    continue

                if state == State.PRIMARY_STATE_UNCONFIGURED:
                    all_active = False
                    self.get_logger().info(f"[{name}] Configuring...")
                    self._transition_node(name, Transition.TRANSITION_CONFIGURE)
                elif state == State.PRIMARY_STATE_INACTIVE:
                    all_active = False
                    self.get_logger().info(f"[{name}] Activating...")
                    self._transition_node(name, Transition.TRANSITION_ACTIVATE)
                elif state == State.PRIMARY_STATE_ACTIVE:
                    pass
                else:
                    all_active = False

            if all_active:
                self.get_logger().info("✅ All Nav2 Planning lifecycle nodes are ACTIVE and healthy!")
                break
            time.sleep(1.0)

    def _get_node_state(self, node_name):
        cli = self.create_client(GetState, f'/{node_name}/get_state')
        if not cli.wait_for_service(timeout_sec=1.0):
            return None
        req = GetState.Request()
        future = cli.call_async(req)
        start = time.time()
        while not future.done() and time.time() - start < 2.0:
            time.sleep(0.05)
        return future.result().current_state.id if future.done() and future.result() else None

    def _transition_node(self, node_name, transition_id):
        cli = self.create_client(ChangeState, f'/{node_name}/change_state')
        if not cli.wait_for_service(timeout_sec=1.0):
            return False
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = cli.call_async(req)
        start = time.time()
        while not future.done() and time.time() - start < 3.0:
            time.sleep(0.05)
        return future.result().success if future.done() and future.result() else False


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleAutostart()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
