#!/usr/bin/env python3
"""ODrive fault-capture logger: subscribes to every axis's controller_status
and odrive_status topics, writes a full wide-format snapshot row to CSV on
every incoming message (nothing is buffered in memory only -- each row is
flushed to disk immediately), and decodes active_errors/disarm_reason
bitmasks into human-readable names both in the live console output and in
the CSV, so a disarm on concrete can be read back without cross-referencing
the ODrive error enum by hand afterward.

Run before a drive test, leave it running for the whole test, Ctrl-C when done:

  ros2 run rover_nav odrive_fault_log.py --ros-args -p num_axes:=6

Bitmask values mirror ODriveError / ODriveAxisState in
src/vendor/ros_odrive/odrive_base/include/odrive_enums.h -- update here if
that header changes.
"""
import csv
import os
import time

import rclpy
from rclpy.node import Node
from odrive_can.msg import ControllerStatus, ODriveStatus

ODRIVE_ERROR = {
    0x00000001: "INITIALIZING",
    0x00000002: "SYSTEM_LEVEL",
    0x00000004: "TIMING_ERROR",
    0x00000008: "MISSING_ESTIMATE",
    0x00000010: "BAD_CONFIG",
    0x00000020: "DRV_FAULT",
    0x00000040: "MISSING_INPUT",
    0x00000100: "DC_BUS_OVER_VOLTAGE",
    0x00000200: "DC_BUS_UNDER_VOLTAGE",
    0x00000400: "DC_BUS_OVER_CURRENT",
    0x00000800: "DC_BUS_OVER_REGEN_CURRENT",
    0x00001000: "CURRENT_LIMIT_VIOLATION",
    0x00002000: "MOTOR_OVER_TEMP",
    0x00004000: "INVERTER_OVER_TEMP",
    0x00008000: "VELOCITY_LIMIT_VIOLATION",
    0x00010000: "POSITION_LIMIT_VIOLATION",
    0x01000000: "WATCHDOG_TIMER_EXPIRED",
    0x02000000: "ESTOP_REQUESTED",
    0x04000000: "SPINOUT_DETECTED",
    0x08000000: "BRAKE_RESISTOR_DISARMED",
    0x10000000: "THERMISTOR_DISCONNECTED",
    0x40000000: "CALIBRATION_ERROR",
}

AXIS_STATE_NAME = {
    0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQUENCE",
    3: "FULL_CALIBRATION_SEQUENCE", 4: "MOTOR_CALIBRATION",
    6: "ENCODER_INDEX_SEARCH", 7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL", 9: "LOCKIN_SPIN", 10: "ENCODER_DIR_FIND",
    11: "HOMING", 12: "ENCODER_HALL_POLARITY_CALIBRATION",
    13: "ENCODER_HALL_PHASE_CALIBRATION", 14: "ANTICOGGING_CALIBRATION",
}

CTRL_FIELDS = [
    "axis_state", "procedure_result", "pos_estimate", "vel_estimate",
    "iq_setpoint", "iq_measured", "torque_target", "torque_estimate",
    "trajectory_done_flag", "ctrl_active_errors",
]
STATUS_FIELDS = [
    "bus_voltage", "bus_current", "fet_temperature", "motor_temperature",
    "status_active_errors", "disarm_reason",
]


def decode_flags(value: int) -> str:
    if value == 0:
        return ""
    names = [name for bit, name in ODRIVE_ERROR.items() if value & bit]
    return "|".join(names) if names else f"UNKNOWN(0x{value:08X})"


class OdriveFaultLog(Node):
    def __init__(self):
        super().__init__("odrive_fault_log")
        self.declare_parameter("num_axes", 6)
        self.declare_parameter(
            "csv_path",
            os.path.expanduser(
                "~/jazzy_ws/src/rover_nav/data/odrive_diagnostics/"
                f"odrive_fault_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            ),
        )
        self.num_axes = int(self.get_parameter("num_axes").value)
        self.csv_path = str(self.get_parameter("csv_path").value)
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)

        self.start_time = self.get_clock().now()
        self.row_count = 0
        self.fault_events = []  # (t_s, axis, field, decoded)

        # latest known value per axis for every field, so every row is a full
        # snapshot even though only one axis/topic actually just updated
        self.latest = {
            i: {f: None for f in CTRL_FIELDS + STATUS_FIELDS}
            for i in range(self.num_axes)
        }
        self._prev_errors = {
            i: {"ctrl_active_errors": 0, "status_active_errors": 0, "disarm_reason": 0}
            for i in range(self.num_axes)
        }
        self._prev_axis_state = {i: None for i in range(self.num_axes)}

        self._csv_file = open(self.csv_path, "w", newline="")
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(self._header())
        self._csv_file.flush()

        for i in range(self.num_axes):
            self.create_subscription(
                ControllerStatus, f"/odrive_axis{i}/controller_status",
                self._make_ctrl_cb(i), 10,
            )
            self.create_subscription(
                ODriveStatus, f"/odrive_axis{i}/odrive_status",
                self._make_status_cb(i), 10,
            )

        self.create_timer(10.0, self._heartbeat)
        self.get_logger().info(
            f"ODrive fault logger running for {self.num_axes} axes -> {self.csv_path}"
        )

    def _header(self):
        cols = ["t_s", "trigger"]
        for i in range(self.num_axes):
            for f in CTRL_FIELDS:
                cols.append(f"axis{i}_{f}")
            cols.append(f"axis{i}_ctrl_errors_decoded")
            for f in STATUS_FIELDS:
                cols.append(f"axis{i}_{f}")
            cols.append(f"axis{i}_status_errors_decoded")
            cols.append(f"axis{i}_disarm_decoded")
        return cols

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def _make_ctrl_cb(self, axis: int):
        def cb(msg: ControllerStatus):
            st = self.latest[axis]
            st["axis_state"] = msg.axis_state
            st["procedure_result"] = msg.procedure_result
            st["pos_estimate"] = msg.pos_estimate
            st["vel_estimate"] = msg.vel_estimate
            st["iq_setpoint"] = msg.iq_setpoint
            st["iq_measured"] = msg.iq_measured
            st["torque_target"] = msg.torque_target
            st["torque_estimate"] = msg.torque_estimate
            st["trajectory_done_flag"] = msg.trajectory_done_flag
            st["ctrl_active_errors"] = msg.active_errors
            self._check_faults(axis, "ctrl_active_errors", msg.active_errors)
            self._check_axis_state(axis, msg.axis_state)
            self._write_row(f"axis{axis}/controller_status")
        return cb

    def _make_status_cb(self, axis: int):
        def cb(msg: ODriveStatus):
            st = self.latest[axis]
            st["bus_voltage"] = msg.bus_voltage
            st["bus_current"] = msg.bus_current
            st["fet_temperature"] = msg.fet_temperature
            st["motor_temperature"] = msg.motor_temperature
            st["status_active_errors"] = msg.active_errors
            st["disarm_reason"] = msg.disarm_reason
            self._check_faults(axis, "status_active_errors", msg.active_errors)
            self._check_faults(axis, "disarm_reason", msg.disarm_reason)
            self._write_row(f"axis{axis}/odrive_status")
        return cb

    def _check_faults(self, axis: int, field: str, value: int):
        prev = self._prev_errors[axis][field]
        newly_set = value & ~prev
        if newly_set:
            decoded = decode_flags(newly_set)
            t = self._elapsed()
            self.fault_events.append((t, axis, field, decoded))
            self.get_logger().error(
                f"[{t:7.2f}s] axis{axis} {field} -> {decoded} "
                f"(axis_state={AXIS_STATE_NAME.get(self.latest[axis]['axis_state'], '?')})"
            )
        self._prev_errors[axis][field] = value

    def _check_axis_state(self, axis: int, state: int):
        prev = self._prev_axis_state[axis]
        if prev == 8 and state != 8:  # was CLOSED_LOOP_CONTROL, dropped out
            t = self._elapsed()
            self.get_logger().error(
                f"[{t:7.2f}s] axis{axis} left CLOSED_LOOP_CONTROL -> "
                f"{AXIS_STATE_NAME.get(state, state)}"
            )
        self._prev_axis_state[axis] = state

    def _write_row(self, trigger: str):
        row = [f"{self._elapsed():.4f}", trigger]
        for i in range(self.num_axes):
            st = self.latest[i]
            for f in CTRL_FIELDS:
                row.append("" if st[f] is None else st[f])
            row.append(decode_flags(st["ctrl_active_errors"] or 0))
            for f in STATUS_FIELDS:
                row.append("" if st[f] is None else st[f])
            row.append(decode_flags(st["status_active_errors"] or 0))
            row.append(decode_flags(st["disarm_reason"] or 0))
        self._writer.writerow(row)
        self._csv_file.flush()
        self.row_count += 1

    def _heartbeat(self):
        self.get_logger().info(
            f"...logging, {self.row_count} rows so far, "
            f"{len(self.fault_events)} fault events -> {self.csv_path}",
            throttle_duration_sec=10.0,
        )

    def shutdown(self):
        self._csv_file.close()
        n = len(self.fault_events)
        if n == 0:
            self.get_logger().info(
                f"{self.row_count} rows saved to {self.csv_path} -- no faults observed."
            )
        else:
            self.get_logger().warn(
                f"{self.row_count} rows saved to {self.csv_path} -- {n} fault event(s):\n"
                + "\n".join(
                    f"  [{t:7.2f}s] axis{axis} {field} -> {decoded}"
                    for t, axis, field, decoded in self.fault_events
                )
            )


def main(args=None):
    rclpy.init(args=args)
    node = OdriveFaultLog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
