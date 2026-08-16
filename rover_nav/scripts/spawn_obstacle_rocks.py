#!/usr/bin/env python3
"""
spawn_obstacle_rocks.py
───────────────────────
Spawns realistic boulder obstacles into the running Gazebo simulation via
the `ros_gz_sim` SpawnEntity service, placing them ON the rover's tour path
at strategic locations to test obstacle avoidance.

Usage:
  # With default rock set (5 rocks on path):
  ros2 run rover_nav spawn_obstacle_rocks.py

  # Clear all spawned rocks:
  ros2 run rover_nav spawn_obstacle_rocks.py --ros-args -p clear_rocks:=true

Design:
  Each "rock" is an SDF model composed of 1–3 overlapping ellipsoid/box
  primitives offset from each other to look like a natural boulder cluster.
  Rocks are placed directly ON the waypoint path so the rover's depth camera
  detects them and triggers the avoidance detour in Pure_pursuit_Gazebo.py.
"""

import math
import sys
import time
import textwrap

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from ros_gz_interfaces.srv import SpawnEntity
from geometry_msgs.msg import Pose


# ─── Rock definitions: (name, x, y, z_offset, description) ───────────────────
# Positions are on the marsyard2026_tour_waypoints.csv path where ≥1.4m clearance exists.
# Rock 2 and 4 were removed because they were placed in narrow corridors adjacent to steep hills.
ROCKS = [
    {
        "name": "rock_1_s1_exit",
        "x": -1.709, "y": 6.813, "z": 0.0,
        "yaw": 0.4,
        "scale": 0.85,
        "note": "~12%: First stretch leaving S1 — clear space on both sides.",
    },
    {
        "name": "rock_3_north_midfield",
        "x": -4.509, "y": 18.718, "z": 0.0,
        "yaw": 0.7,
        "scale": 0.95,
        "note": "~50%: North midfield stretch — right side open for detour.",
    },
    {
        "name": "rock_5_south_finish",
        "x": 5.500, "y": 8.313, "z": 0.0,
        "yaw": 0.3,
        "scale": 0.75,
        "note": "~82%: South finish stretch — clear right-side corridor.",
    },
]


def make_rock_sdf(name: str, x: float, y: float, z: float,
                  yaw: float, scale: float) -> str:
    """
    Generate an SDF snippet for a natural-looking boulder cluster.

    The rock is made from 3 overlapping box/sphere primitives with slight
    positional offsets and different sizes to fake an organic boulder shape.
    All geometry has collision so the rover's depth camera sees and avoids it.
    """
    s = scale

    # Boulder colours: warm sandy-brown / rust / dark stone palette
    r, g, b = 0.50, 0.38, 0.25   # warm sandstone

    sdf = textwrap.dedent(f"""\
    <?xml version="1.0"?>
    <sdf version="1.10">
      <model name="{name}">
        <static>true</static>
        <pose>{x} {y} {z + 0.18 * s} 0 0 {yaw:.4f}</pose>

        <!-- Primary boulder body -->
        <link name="body_main">
          <collision name="col_main">
            <geometry>
              <box><size>{0.65*s:.3f} {0.50*s:.3f} {0.38*s:.3f}</size></box>
            </geometry>
            <surface>
              <friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction>
            </surface>
          </collision>
          <visual name="vis_main">
            <geometry>
              <box><size>{0.65*s:.3f} {0.50*s:.3f} {0.38*s:.3f}</size></box>
            </geometry>
            <material>
              <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
              <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
              <specular>0.05 0.04 0.03 1</specular>
            </material>
          </visual>
        </link>

        <!-- Secondary lobe — offset forward/left, smaller -->
        <link name="body_lobe1">
          <pose>{0.22*s:.3f} {0.16*s:.3f} {-0.04*s:.3f} 0.1 -0.15 0</pose>
          <collision name="col_lobe1">
            <geometry>
              <sphere><radius>{0.20*s:.3f}</radius></sphere>
            </geometry>
          </collision>
          <visual name="vis_lobe1">
            <geometry>
              <sphere><radius>{0.20*s:.3f}</radius></sphere>
            </geometry>
            <material>
              <ambient>{r*0.85:.2f} {g*0.85:.2f} {b*0.85:.2f} 1</ambient>
              <diffuse>{r*0.85:.2f} {g*0.85:.2f} {b*0.85:.2f} 1</diffuse>
            </material>
          </visual>
        </link>

        <!-- Tertiary lobe — slightly behind and to the right -->
        <link name="body_lobe2">
          <pose>{-0.18*s:.3f} {-0.12*s:.3f} {-0.06*s:.3f} -0.05 0.1 0</pose>
          <collision name="col_lobe2">
            <geometry>
              <box><size>{0.28*s:.3f} {0.22*s:.3f} {0.18*s:.3f}</size></box>
            </geometry>
          </collision>
          <visual name="vis_lobe2">
            <geometry>
              <box><size>{0.28*s:.3f} {0.22*s:.3f} {0.18*s:.3f}</size></box>
            </geometry>
            <material>
              <ambient>{r*1.1:.2f} {g*1.0:.2f} {b*0.8:.2f} 1</ambient>
              <diffuse>{r*1.1:.2f} {g*1.0:.2f} {b*0.8:.2f} 1</diffuse>
            </material>
          </visual>
        </link>

      </model>
    </sdf>
    """)
    return sdf


class RockSpawner(Node):

    def __init__(self):
        super().__init__("rock_spawner")

        self.declare_parameter("clear_rocks", False)
        self.declare_parameter("world_name", "leo_marsyard")

        self._world = self.get_parameter("world_name").value
        self._clear = self.get_parameter("clear_rocks").value

        # Wait for Gazebo spawn service
        self._spawn_client = self.create_client(
            SpawnEntity, f"/world/{self._world}/create"
        )

        self._timer = self.create_timer(0.5, self._run)
        self._done = False
        self._attempt = 0

    def _run(self):
        if self._done:
            return

        self._attempt += 1
        if not self._spawn_client.wait_for_service(timeout_sec=1.0):
            if self._attempt < 10:
                self.get_logger().warn(
                    f"Waiting for Gazebo spawn service '/world/{self._world}/create' "
                    f"(attempt {self._attempt}/10) …"
                )
                return
            else:
                self.get_logger().error(
                    "Gazebo spawn service not available after 5 s. "
                    "Make sure Gazebo is running first."
                )
                self._done = True
                rclpy.shutdown()
                return

        self._done = True
        self._timer.cancel()

        if self._clear:
            self._delete_rocks()
        else:
            self._spawn_rocks()

    def _spawn_rocks(self):
        spawned = 0
        for rock in ROCKS:
            sdf = make_rock_sdf(
                name=rock["name"],
                x=rock["x"], y=rock["y"], z=rock["z"],
                yaw=rock["yaw"], scale=rock["scale"],
            )

            req = SpawnEntity.Request()
            req.entity_factory.sdf = sdf
            req.entity_factory.name = rock["name"]
            req.entity_factory.relative_to = ""

            future = self._spawn_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.result() is not None and future.result().success:
                self.get_logger().info(
                    f"✅  Spawned rock '{rock['name']}' at "
                    f"[x={rock['x']:.2f}, y={rock['y']:.2f}]  — {rock['note']}"
                )
                spawned += 1
            else:
                err = future.result().message if future.result() else "timeout"
                self.get_logger().error(
                    f"❌  Failed to spawn '{rock['name']}': {err}"
                )

            time.sleep(0.1)  # let Gazebo catch its breath between spawns

        self.get_logger().info(
            f"\n🪨  Rock spawning complete: {spawned}/{len(ROCKS)} placed.\n"
            "    The rover should now trigger avoidance detours as it approaches each rock.\n"
            "    Monitor /obstacles and /pure_pursuit/local_avoidance_path in RViz2."
        )
        rclpy.shutdown()

    def _delete_rocks(self):
        from ros_gz_interfaces.srv import DeleteEntity
        del_client = self.create_client(
            DeleteEntity, f"/world/{self._world}/remove"
        )
        if not del_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Delete service not available.")
            rclpy.shutdown()
            return

        removed = 0
        for rock in ROCKS:
            req = DeleteEntity.Request()
            req.entity.name = rock["name"]
            req.entity.type = 2  # MODEL = 2
            future = del_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is not None and future.result().success:
                self.get_logger().info(f"🗑️  Removed '{rock['name']}'")
                removed += 1
            else:
                err = future.result().message if future.result() else "timeout"
                self.get_logger().warn(f"Could not remove '{rock['name']}': {err}")

        self.get_logger().info(f"Cleared {removed}/{len(ROCKS)} rocks.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RockSpawner()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
