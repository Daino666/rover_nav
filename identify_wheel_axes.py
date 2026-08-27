#!/usr/bin/env python3
"""Identify which ODrive CAN node id drives which physical wheel, by hand.

Why this exists: rover_nav/Odom.py and aries_drive/config/cmd_vel_odrive_bridge.yaml
each carry a right_wheels/left_wheels node-id list, and both files say the two
must agree -- but the mapping has changed twice on this rover, so stale copies
drift apart between workspaces:

    2026-08-12  chassis reassembled; node ids stopped following the sides,
                giving the interleaved  right [0,4,3]  left [5,1,2]
    2026-08-23  node ids reassigned back to contiguous blocks by side,
                verified by arming one axis at a time:
                  0 Front-Left    1 Middle-Left   2 Back-Left
                  3 Back-Right    4 Middle-Right  5 Front-Right
                Both lists are written FRONT -> REAR, so that is
                  right [5,4,3]  left [0,1,2]   <- current
                (node-id order [3,4,5] would be back -> front on the
                right side and contradicts the front->rear convention)

Run this whenever you are unsure which era a given tree is from, or after any
work that could have reassigned node ids.

Getting this wrong matters: cmd_vel_odrive_bridge negates one side
(left_rps = -physical_left_mps / circumference), so a wheel listed on the wrong
side spins BACKWARD relative to its neighbours on a plain forward command. Two
front wheels fighting four rear ones scrubs badly, makes the rear wheels slip,
and makes the encoders over-read ground distance -- which is very likely why
the rover to scrub. (Note: WHEEL_CIRCUMFERENCE = 0.667442 m sitting 3.4% below
the URDF's free circumference of 0.6912 m is NOT evidence of that -- a loaded
tire's rolling radius is legitimately smaller than its free radius.)

This tool reads the ODrive Get_Encoder_Estimates CAN frames directly. It sends
nothing and arms nothing -- the motors stay disarmed and the ROS stack does not
need to be running. You turn each wheel by hand and read off which node id
moved, and in which direction.

    python3 identify_wheel_axes.py

Rover on stands (or at least each wheel free to turn) is easiest, but anything
that lets you rotate one wheel at a time works.
"""
import argparse
import os
import socket
import struct
import sys
import time

# Linux SocketCAN frame: id(4B) dlc(1B) pad(3B) data(8B)
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF

# ODrive CAN: can_id = (node_id << 5) | cmd_id
ODRIVE_CMD_MASK = 0x1F
ODRIVE_NODE_SHIFT = 5
CMD_GET_ENCODER_ESTIMATES = 0x09

# Movement thresholds, in motor revolutions.
MOVED_REV = 0.25      # a wheel counts as "turned" past this much
SETTLE_REV = 0.02     # below this, treat as stationary noise


def open_can(interface):
    try:
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    except AttributeError:
        raise SystemExit("SocketCAN is not available on this platform.")
    try:
        sock.bind((interface,))
    except OSError as exc:
        raise SystemExit(
            f"Could not bind to {interface!r}: {exc}\n"
            f"Is the interface up?  ip link show {interface}"
        )
    return sock


def read_positions(sock, duration_s, positions):
    """Drain encoder frames for duration_s, updating {node_id: pos_rev}."""
    deadline = time.monotonic() + duration_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return positions
        sock.settimeout(remaining)
        try:
            frame = sock.recv(CAN_FRAME_SIZE)
        except socket.timeout:
            return positions
        if len(frame) < CAN_FRAME_SIZE:
            continue
        raw_id, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
        if raw_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG):
            continue
        can_id = raw_id & CAN_SFF_MASK
        if (can_id & ODRIVE_CMD_MASK) != CMD_GET_ENCODER_ESTIMATES:
            continue
        if dlc < 8:
            continue
        node_id = can_id >> ODRIVE_NODE_SHIFT
        pos_rev, _vel_rps = struct.unpack("<ff", data[:8])
        positions[node_id] = pos_rev
    return positions


def survey(sock, settle_s, sample_s):
    """Return (baseline, final, deltas) around one hand-turn."""
    baseline = read_positions(sock, settle_s, {})
    input("    turn the wheel now, then press Enter... ")
    final = read_positions(sock, sample_s, dict(baseline))
    deltas = {
        node: final[node] - baseline[node]
        for node in sorted(set(baseline) & set(final))
    }
    return baseline, final, deltas


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--interface", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument("--settle", type=float, default=1.0,
                        help="Seconds of baseline sampling before each turn (default: 1.0)")
    parser.add_argument("--sample", type=float, default=1.0,
                        help="Seconds of sampling after each turn (default: 1.0)")
    args = parser.parse_args()

    sock = open_can(args.interface)

    print(__doc__.split("\n\n")[0])
    print(f"\nListening on {args.interface}. Nothing is transmitted; motors stay disarmed.\n")

    seen = read_positions(sock, 2.0, {})
    if not seen:
        raise SystemExit(
            f"No ODrive encoder frames seen on {args.interface}.\n"
            "The ODrives publish these cyclically, or the odrive_can_poller must be\n"
            "running to request them. Check:  candump " + args.interface
        )
    print(f"Axes responding: {', '.join(str(n) for n in sorted(seen))}\n")

    wheels = [
        "Right-Front", "Right-Mid", "Right-Rear",
        "Left-Front", "Left-Mid", "Left-Rear",
    ]
    mapping = {}

    for wheel in wheels:
        print(f"[{wheel}]  rotate this wheel FORWARD (as if driving ahead), about one turn.")
        _baseline, _final, deltas = survey(sock, args.settle, args.sample)

        moved = {n: d for n, d in deltas.items() if abs(d) >= MOVED_REV}
        if not moved:
            print("    nothing moved past the threshold -- skipped. Turn it further next time.\n")
            continue
        node = max(moved, key=lambda n: abs(moved[n]))
        others = [n for n in moved if n != node]
        delta = moved[node]
        sign = "+" if delta > 0 else "-"
        print(f"    -> node {node}   ({delta:+.3f} rev, sign {sign})")
        if others:
            detail = ", ".join(f"{n}:{moved[n]:+.3f}" for n in others)
            print(f"       NOTE: other axes also moved ({detail}) -- belt/gear coupling, or you"
                  f"\n       turned more than one wheel. Re-run this wheel if that was unintended.")
        mapping[wheel] = (node, delta)
        print()

    if not mapping:
        raise SystemExit("No wheels identified.")

    print("\n" + "=" * 64)
    print("RESULT")
    print("=" * 64)
    for wheel in wheels:
        if wheel in mapping:
            node, delta = mapping[wheel]
            print(f"  {wheel:<12} node {node}   forward rotation = {delta:+.3f} rev")
        else:
            print(f"  {wheel:<12} (not identified)")

    right = [mapping[w][0] for w in ("Right-Front", "Right-Mid", "Right-Rear") if w in mapping]
    left = [mapping[w][0] for w in ("Left-Front", "Left-Mid", "Left-Rear") if w in mapping]

    print("\nParameter values these imply (front -> rear):")
    print(f"  right_wheels: {right}")
    print(f"  left_wheels:  {left}")

    if len(right) == 3 and len(left) == 3:
        print("\nPut these in BOTH of:")
        print("  src/aries_drive/config/cmd_vel_odrive_bridge.yaml")
        print("  src/rover_nav/scripts/Odom.py   (the declare_parameter defaults)")
        print("\nExpected since the 2026-08-23 reassignment:")
        print("  right [5, 4, 3]  left [0, 1, 2]   (front -> rear)")
        print("\nThen rebuild before anything takes effect:")
        print("  cd ~/test && colcon build --packages-select aries_drive rover_nav \\")
        print("    && source install/setup.bash")
        print("\nOnly AFTER the mapping is correct and the rover drives straight without")
        print("scrub is a WHEEL_CIRCUMFERENCE calibration run meaningful.")
    else:
        print("\nIncomplete -- identify all six wheels before changing any config.")

    print("\nAlso note the sign column: cmd_vel_odrive_bridge negates the left side")
    print("(left_rps = -physical_left_mps / circumference), so for a forward command the")
    print("two sides are expected to report OPPOSITE encoder signs. If your six signs do")
    print("not split cleanly 3-and-3 by side, the wiring/mounting polarity disagrees with")
    print("that assumption and needs sorting out before calibration too.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        sys.exit(130)
