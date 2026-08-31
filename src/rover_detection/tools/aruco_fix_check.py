#!/usr/bin/env python3
"""Measures what the ArUco localization pipeline actually reports, so its
accuracy and repeatability are numbers rather than an impression.

Samples /aruco_localization/rover_position_fix for a while and prints, per
landmark: how many fixes arrived, the mean reported rover position, the
spread (repeatability), and -- if you say where the rover really is -- the
error (accuracy).

  # repeatability only, no ground truth needed
  python3 aruco_fix_check.py --seconds 20

  # accuracy too: rover's true position, measured with a tape
  python3 aruco_fix_check.py --seconds 20 --truth 0.0,0.0

Two-marker consistency: with several landmarks in view this reports each one
separately, so you can check they AGREE. Two markers that each look stable
but disagree with each other means the error is in something common to both
-- the landmark table, marker_length_m, or the camera offset -- not in the
detection. That is a failure the single-marker view cannot see, which is why
the bench setup uses two.

Note the node itself only publishes the CLOSEST landmark per frame, so to get
both you must either place them at near-equal range, or run the detector
twice with id_whitelist set to one ID each.
"""

import argparse
import math
import statistics
import sys

import rclpy
from rclpy.node import Node
from rover_perception_msgs.msg import RoverPositionFix

SEED_ID = -1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=20.0)
    ap.add_argument('--topic', default='/aruco_localization/rover_position_fix')
    ap.add_argument('--truth', help="rover's true position 'x,y' in the map frame")
    args = ap.parse_args()

    truth = None
    if args.truth:
        truth = tuple(float(v) for v in args.truth.split(','))

    rclpy.init()
    node = Node('aruco_fix_check')
    per_id = {}

    def on_fix(msg):
        if msg.landmark_id == SEED_ID:
            return
        per_id.setdefault(msg.landmark_id, []).append(
            (msg.position.x, msg.position.y, msg.distance))

    node.create_subscription(RoverPositionFix, args.topic, on_fix, 50)
    print(f'listening on {args.topic} for {args.seconds:.0f}s '
          f'({"accuracy vs " + str(truth) if truth else "repeatability only"}) ...')

    import time
    t0 = time.time()
    while time.time() - t0 < args.seconds and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)

    if not per_id:
        print('\nNo fixes received. Check, in order:')
        print('  1. is the camera up with align_depth.enable:=true?')
        print('  2. is aruco_detect_roverpos running? (rqt_image_view on '
              '/aruco_localization/debug_image shows what it sees)')
        print('  3. is the map -> base_footprint TF available? no heading, no fix')
        print('  4. is the marker in the landmark table, and within max_reliable_range_m?')
        rclpy.shutdown()
        return 1

    print(f'\n{"landmark":>9}{"n":>6}{"range":>8}{"mean X":>10}{"mean Y":>10}'
          f'{"spread":>9}' + (f'{"error":>9}' if truth else ''))
    means = {}
    for lid in sorted(per_id):
        xs = [p[0] for p in per_id[lid]]
        ys = [p[1] for p in per_id[lid]]
        ds = [p[2] for p in per_id[lid]]
        mx, my = statistics.mean(xs), statistics.mean(ys)
        means[lid] = (mx, my)
        # spread: RMS distance of each fix from the mean fix -- one number for
        # scatter in both axes at once
        spread = math.sqrt(statistics.mean((x - mx) ** 2 + (y - my) ** 2
                                           for x, y in zip(xs, ys)))
        row = (f'{lid:>9}{len(xs):>6}{statistics.mean(ds):>7.2f}m'
               f'{mx:>10.3f}{my:>10.3f}{spread:>8.3f}m')
        if truth:
            row += f'{math.dist((mx, my), truth):>8.3f}m'
        print(row)

    if len(means) >= 2:
        ids = sorted(means)
        print('\ntwo-marker consistency (they should agree -- disagreement points at the')
        print('landmark table, marker_length_m, or the camera offset, not at detection):')
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                d = math.dist(means[ids[i]], means[ids[j]])
                verdict = 'consistent' if d < 0.10 else ('marginal' if d < 0.25 else 'DISAGREE')
                print(f'  L{ids[i]-50} vs L{ids[j]-50}: {d:.3f} m apart -- {verdict}')
    else:
        print('\nonly one landmark seen -- no cross-check possible. Place a second marker')
        print('at a similar range, or re-run with id_whitelist set to the other ID.')

    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
