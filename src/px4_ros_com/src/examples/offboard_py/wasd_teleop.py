#!/usr/bin/env python3

import math
import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class WasdTeleop(Node):
    """Simple keyboard teleop for PX4 offboard velocity control."""

    def __init__(self) -> None:
        super().__init__("px4_wasd_teleop")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile
        )

        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.vehicle_local_position_callback,
            qos_profile,
        )
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile,
        )

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0

        self.takeoff_height = -2.0
        self.xy_speed = 1.5
        self.z_speed = 0.8
        self.yaw_rate = 0.8

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yawspeed = 0.0
        self.request_land = False

        self._input_stream = sys.stdin
        self._input_fd = None
        self._terminal_settings = None

        # ros2 launch can start this node without a usable sys.stdin TTY.
        # Fall back to the controlling terminal so keyboard control still works.
        try:
            if not sys.stdin.isatty():
                self._input_stream = open("/dev/tty", "r", encoding="utf-8", buffering=1)

            self._input_fd = self._input_stream.fileno()
            self._terminal_settings = termios.tcgetattr(self._input_fd)
            tty.setcbreak(self._input_fd)
        except OSError as exc:
            raise RuntimeError(
                "WASD teleop requires an interactive TTY. Run from a real terminal."
            ) from exc

        self.get_logger().info(
            "WASD teleop ready: w/s forward-back, a/d left-right, r/f up-down, q/e yaw, x stop, l land, Ctrl+C exit"
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

    def destroy_node(self):
        if self._input_fd is not None and self._terminal_settings is not None:
            termios.tcsetattr(self._input_fd, termios.TCSADRAIN, self._terminal_settings)
        if self._input_stream is not sys.stdin:
            self._input_stream.close()
        return super().destroy_node()

    def vehicle_local_position_callback(self, vehicle_local_position: VehicleLocalPosition) -> None:
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status: VehicleStatus) -> None:
        self.vehicle_status = vehicle_status

    def publish_vehicle_command(self, command: int, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def publish_offboard_control_heartbeat_signal(self) -> None:
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float, yawspeed: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [vx, vy, vz]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = math.nan
        msg.yawspeed = yawspeed
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def arm(self) -> None:
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self) -> None:
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self) -> None:
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def read_key(self) -> None:
        if self._input_stream is None:
            return

        if not select.select([self._input_stream], [], [], 0.0)[0]:
            return

        key = self._input_stream.read(1).lower()
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yawspeed = 0.0

        if key == "w":
            self.vx = self.xy_speed
        elif key == "s":
            self.vx = -self.xy_speed
        elif key == "a":
            self.vy = self.xy_speed
        elif key == "d":
            self.vy = -self.xy_speed
        elif key == "r":
            self.vz = -self.z_speed
        elif key == "f":
            self.vz = self.z_speed
        elif key == "q":
            self.yawspeed = self.yaw_rate
        elif key == "e":
            self.yawspeed = -self.yaw_rate
        elif key == "x":
            pass
        elif key == "l":
            self.request_land = True
        else:
            return

        if key != "x":
            self.get_logger().info(
                f"Command: key={key}, vx={self.vx:.1f}, vy={self.vy:.1f}, vz={self.vz:.1f}, yaw_rate={self.yawspeed:.1f}"
            )

    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        if self.request_land:
            self.land()
            return

        in_offboard = self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        below_takeoff_alt = self.vehicle_local_position.z > self.takeoff_height

        if in_offboard and below_takeoff_alt:
            self.publish_velocity_setpoint(0.0, 0.0, -self.z_speed, 0.0)
            return

        self.read_key()
        self.publish_velocity_setpoint(self.vx, self.vy, self.vz, self.yawspeed)


def main(args=None) -> None:
    print("Starting PX4 WASD teleop node...")
    rclpy.init(args=args)
    teleop = WasdTeleop()
    rclpy.spin(teleop)
    teleop.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
