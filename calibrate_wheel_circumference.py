#!/usr/bin/env python3
"""Computes the corrected WHEEL_CIRCUMFERENCE for src/rover_nav/scripts/Odom.py
from one straight-line calibration test's measured numbers.

Rationale: distance-proportional error (roughly constant % of path length,
seen both in the historical 22m hall-loop test and the 35m x 15m course this
session) is the signature of a wheel-odometry SCALE error, not sensor noise
or magnetometer EMI -- if WHEEL_CIRCUMFERENCE is off from the rover's actual
wheel circumference, every encoder tick converts to a proportionally wrong
distance, independent of yaw/IMU entirely.

imu_deep_log.py's ekf_path_length_m is a valid proxy for this even though it
goes through the EKF: rotating a vector doesn't change its length, so as
long as the calibration drive is reasonably straight, heading error barely
affects the *magnitude* of accumulated distance, only its direction.

Usage, after running a straight-line test with imu_deep_log.py's
real_distance_m set (see its final report for "EKF path length" and "real
distance"):

  python3 calibrate_wheel_circumference.py --real-distance 18.0 --ekf-distance 18.72

  # to also patch Odom.py's WHEEL_CIRCUMFERENCE directly instead of just
  # printing the corrected value:
  python3 calibrate_wheel_circumference.py --real-distance 18.0 --ekf-distance 18.72 --apply
"""
import argparse
import re
import sys
from pathlib import Path

ODOM_PY = Path(__file__).parent / "src" / "rover_nav" / "scripts" / "Odom.py"
CONST_PATTERN = re.compile(r"^WHEEL_CIRCUMFERENCE\s*=\s*([0-9.]+)(\s*#.*)?$", re.MULTILINE)


def read_current_circumference() -> float:
    text = ODOM_PY.read_text()
    m = CONST_PATTERN.search(text)
    if not m:
        raise SystemExit(f"Could not find WHEEL_CIRCUMFERENCE in {ODOM_PY}")
    return float(m.group(1))


def apply_circumference(new_value: float) -> None:
    text = ODOM_PY.read_text()
    new_text, n = CONST_PATTERN.subn(
        f"WHEEL_CIRCUMFERENCE = {new_value:.6f}  # meters (calibrated)", text, count=1
    )
    if n != 1:
        raise SystemExit(f"Could not patch WHEEL_CIRCUMFERENCE in {ODOM_PY}")
    ODOM_PY.write_text(new_text)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--real-distance", type=float, required=True,
        help="Taped/measured real distance driven (m)",
    )
    parser.add_argument(
        "--ekf-distance", type=float, required=True,
        help="imu_deep_log.py's reported 'EKF path length' for that same run (m)",
    )
    parser.add_argument(
        "--current-circumference", type=float, default=None,
        help="Override instead of reading Odom.py's current WHEEL_CIRCUMFERENCE",
    )
    parser.add_argument(
        "--apply", action="store_true",
        help="Patch the corrected value directly into Odom.py (default: print only)",
    )
    args = parser.parse_args()

    if args.real_distance <= 0 or args.ekf_distance <= 0:
        raise SystemExit("Both distances must be positive.")

    current = (
        args.current_circumference
        if args.current_circumference is not None
        else read_current_circumference()
    )

    scale = args.real_distance / args.ekf_distance
    corrected = current * scale
    pct_error = (args.ekf_distance - args.real_distance) / args.real_distance * 100.0

    print(f"real distance:        {args.real_distance:.4f} m")
    print(f"EKF path length:      {args.ekf_distance:.4f} m")
    print(f"EKF vs real error:    {pct_error:+.2f}%")
    print(f"current WHEEL_CIRCUMFERENCE: {current:.6f} m")
    print(f"corrected WHEEL_CIRCUMFERENCE: {corrected:.6f} m  (scale x{scale:.4f})")

    if abs(pct_error) < 0.5:
        print("\nError is under 0.5% -- within normal measurement noise, "
              "probably not worth changing the constant over this alone.")
        return

    if args.apply:
        apply_circumference(corrected)
        print(f"\nPatched {ODOM_PY} -- WHEEL_CIRCUMFERENCE is now {corrected:.6f} m.")
        print("Rebuild before this takes effect: colcon build --packages-select rover_nav")
    else:
        print("\nRe-run with --apply to patch this into Odom.py directly.")


if __name__ == "__main__":
    main()
