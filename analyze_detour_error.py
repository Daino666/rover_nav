#!/usr/bin/env python3
"""End-to-end position error calculator for an arbitrary (turning) route --
a follow-up check after calibrate_wheel_circumference.py, since a route with
turns mixes wheel-circumference error (now corrected), WHEELBASE/turning-
scale error, and residual EMI-driven yaw error into one endpoint comparison.
This can't separate those three, but it IS a valid "how good is the whole
corrected system now" number, and comparable against earlier session
data points (e.g. 0.99m x / 0.27m y after the circumference fix, vs 2.2m x
/ 0.67m y before it).

Inputs: the REAL final position (measured however you tracked ground truth
-- tape, known waypoint coordinates, etc.), relative to the rover's actual
start point, and the EKF's reported final position (/odometry/filtered
x, y) for that same run.

  python3 analyze_detour_error.py --real-x 12.0 --real-y 8.0 --ekf-x 12.8 --ekf-y 7.6

Optionally pass the real total distance driven (odometer/measuring-wheel
reading, or a known route length) for an error-as-%-of-distance figure,
consistent with how error has been tracked all session:

  python3 analyze_detour_error.py --real-x 12.0 --real-y 8.0 --ekf-x 12.8 --ekf-y 7.6 \\
    --real-distance 45.0
"""
import argparse
import math


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--real-x", type=float, required=True, help="Real final x (m), relative to start")
    parser.add_argument("--real-y", type=float, required=True, help="Real final y (m), relative to start")
    parser.add_argument("--ekf-x", type=float, required=True, help="EKF-reported final x (m)")
    parser.add_argument("--ekf-y", type=float, required=True, help="EKF-reported final y (m)")
    parser.add_argument(
        "--real-distance", type=float, default=None,
        help="Real total distance driven (m), if known -- for an error-as-%% figure",
    )
    args = parser.parse_args()

    err_x = args.ekf_x - args.real_x
    err_y = args.ekf_y - args.real_y
    err_mag = math.hypot(err_x, err_y)
    real_displacement = math.hypot(args.real_x, args.real_y)

    print(f"real position:  ({args.real_x:.4f}, {args.real_y:.4f})  "
          f"displacement from start: {real_displacement:.4f} m")
    print(f"EKF position:   ({args.ekf_x:.4f}, {args.ekf_y:.4f})")
    print(f"error:          x={err_x:+.4f} m  y={err_y:+.4f} m  "
          f"magnitude={err_mag:.4f} m")

    if args.real_distance is not None and args.real_distance > 0:
        pct = err_mag / args.real_distance * 100.0
        print(f"error as % of real distance driven ({args.real_distance:.2f} m): {pct:.2f}%")
    elif real_displacement > 0:
        pct = err_mag / real_displacement * 100.0
        print(f"error as % of start->end displacement ({real_displacement:.2f} m): {pct:.2f}%")
        print("(pass --real-distance for the actual odometer/route length instead "
              "of straight-line displacement -- more meaningful if the route turned a lot)")

    print(
        "\nNote: this route had turns, so this number mixes wheel-circumference "
        "error (should now be small, already calibrated), WHEELBASE/turning-scale "
        "error, and residual EMI-driven yaw error. It's a valid whole-system check, "
        "not a way to isolate which of those three is responsible."
    )


if __name__ == "__main__":
    main()
