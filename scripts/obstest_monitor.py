#!/usr/bin/env python3
"""Live readout for the obstacle-detection chain.

pcl_obstacle_detector publishes only bbox LINE_LIST markers and a Bool, so
`ros2 topic echo` gives 24 line endpoints per obstacle and nothing you can
hold a tape measure against. This turns the same markers back into ranges
and sizes, and reports the rate at every stage so a CPU-bound detector
(flag lagging reality) is visible rather than inferred.

  obstest watch            live line, refreshed at 2 Hz
  obstest watch --stats    the above, plus an acceptance summary on Ctrl-C
"""
import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


def fmt_pts(n):
    """307200 -> '307k'; small clouds keep their exact count, since after the
    gates a cluster can be a few hundred points and '0k' hides that."""
    if n >= 10000:
        return f'{n // 1000}k'
    return str(n)


class RateCounter:
    """Message rate over a trailing window."""

    def __init__(self, window=3.0):
        self.window = window
        self.stamps = []
        self.last_size = 0

    def tick(self, size=0):
        now = time.monotonic()
        self.stamps.append(now)
        self.last_size = size
        cutoff = now - self.window
        while self.stamps and self.stamps[0] < cutoff:
            self.stamps.pop(0)

    @property
    def hz(self):
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return (len(self.stamps) - 1) / span if span > 0 else 0.0


class Monitor(Node):

    def __init__(self, cloud_topic, stats):
        super().__init__('obstest_monitor')
        self.stats = stats

        # The pcl_ros filters and the RealSense driver publish best-effort;
        # a reliable subscription silently receives nothing from them.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.r_cloud = RateCounter()
        self.r_front = RateCounter()
        self.r_denoised = RateCounter()
        self.r_markers = RateCounter()

        self.create_subscription(
            PointCloud2, cloud_topic,
            lambda m: self.r_cloud.tick(m.width * m.height), sensor_qos)
        self.create_subscription(
            PointCloud2, '/pcl/front',
            lambda m: self.r_front.tick(m.width * m.height), sensor_qos)
        self.create_subscription(
            PointCloud2, '/pcl/denoised',
            lambda m: self.r_denoised.tick(m.width * m.height), sensor_qos)
        self.create_subscription(MarkerArray, '/obstacles/markers', self._markers, 10)
        self.create_subscription(Bool, '/obstacle_detected', self._flag, 10)

        self.flag = None
        self.clusters = []          # (nearest_z, width, height, depth)
        self.t0 = time.monotonic()

        # accumulated over the run, for --stats
        self.frames = 0
        self.frames_with_obstacle = 0
        self.nearest_samples = []
        self.count_samples = []

        self.create_timer(0.5, self._print)

    def _flag(self, msg):
        self.flag = msg.data

    def _markers(self, msg):
        self.r_markers.tick()
        clusters = []
        for m in msg.markers:
            if m.action != Marker.ADD or not m.points:
                continue
            pts = np.array([[p.x, p.y, p.z] for p in m.points])
            mn, mx = pts.min(axis=0), pts.max(axis=0)
            # optical frame: +X right, +Y down, +Z forward. Nearest point is
            # what obstacle avoidance actually asks about, not the centroid.
            clusters.append((mn[2], mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]))
        clusters.sort(key=lambda c: c[0])
        self.clusters = clusters

        self.frames += 1
        self.count_samples.append(len(clusters))
        if clusters:
            self.frames_with_obstacle += 1
            self.nearest_samples.append(clusters[0][0])

    def _print(self):
        t = time.monotonic() - self.t0
        cloud = f'{self.r_cloud.hz:4.1f}Hz/{fmt_pts(self.r_cloud.last_size):>6}'
        front = f'{self.r_front.hz:4.1f}Hz/{fmt_pts(self.r_front.last_size):>6}'
        den = f'{self.r_denoised.hz:4.1f}Hz/{fmt_pts(self.r_denoised.last_size):>6}'
        det = f'{self.r_markers.hz:4.1f}Hz'

        if self.flag is None:
            flag = '\033[2m  ?  \033[0m'
        elif self.flag:
            flag = '\033[31m BLOCK\033[0m'
        else:
            flag = '\033[32m clear\033[0m'

        if self.clusters:
            shown = ', '.join(
                f'{z:.2f}m ({w:.2f}×{h:.2f})' for z, w, h, _ in self.clusters[:3])
            if len(self.clusters) > 3:
                shown += f', +{len(self.clusters) - 3}'
            body = f'{len(self.clusters)}: {shown}'
        else:
            body = '\033[2m—\033[0m'

        # A detector rate well below the cloud rate means the flag describes
        # the world as it was several frames ago.
        lag = ''
        if self.r_cloud.hz > 1 and self.r_markers.hz < 0.7 * self.r_cloud.hz:
            lag = ' \033[33m[lagging]\033[0m'

        sys.stdout.write(
            f'\r\033[K t+{t:6.1f}  cloud {cloud}  gate {front}  clean {den}  '
            f'det {det}{lag}  {flag}  {body}')
        sys.stdout.flush()

    def summary(self):
        print('\n')
        if not self.frames:
            print('  no marker frames received — was the detector running?')
            return
        pct = 100.0 * self.frames_with_obstacle / self.frames
        print(f'  frames                 {self.frames}')
        print(f'  frames with obstacle   {self.frames_with_obstacle}  ({pct:.1f}%)')
        counts = np.array(self.count_samples)
        print(f'  obstacles per frame    mean {counts.mean():.2f}  max {counts.max()}')
        if self.nearest_samples:
            n = np.array(self.nearest_samples)
            print(f'  nearest range          mean {n.mean():.3f} m  '
                  f'sd {n.std():.3f} m  min {n.min():.3f}  max {n.max():.3f}')
            print()
            # An empty_ground bag should read 0.0%; a rock bag should hold its
            # range to within a few cm across the whole pass.
            if n.std() > 0.05:
                print('  \033[33msd > 5 cm — range is not stable across frames\033[0m')
        if pct == 0.0:
            print('  \033[32mno detections for the whole run — clean on empty ground\033[0m')
        elif 0.0 < pct < 95.0:
            print(f'  \033[33mflickering: detected in only {pct:.1f}% of frames\033[0m')
        print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cloud', default='/camera/camera/depth/color/points')
    ap.add_argument('--stats', action='store_true',
                    help='print an acceptance summary on exit')
    args = ap.parse_args()

    rclpy.init()
    node = Monitor(args.cloud, args.stats)
    print('\n  \033[2mcloud → gate → clean shows Hz/points at each stage; '
          'obstacles are nearest-range (width×height)\033[0m\n')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy's own SIGINT handler tears the context down, so spin() raises
        # ExternalShutdownException rather than KeyboardInterrupt. Catching
        # only the latter loses the summary on every Ctrl-C.
        pass
    finally:
        if args.stats:
            node.summary()
        else:
            print()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
