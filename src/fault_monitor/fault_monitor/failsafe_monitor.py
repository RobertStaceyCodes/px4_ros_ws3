#!/usr/bin/env python3
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

from px4_msgs.msg import VehicleCommand, VehicleOdometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


@dataclass
class Watchdog:
    name: str
    last_time: float = 0.0
    timeout_s: float = 0.5
    critical: bool = False


class FailsafeMonitor(Node):
    """
    Failsafe rules:
      - If /d500/scan times out -> FAILSAFE -> LAND
      - If /fmu/out/vehicle_odometry times out -> FAILSAFE -> LAND
      - If tf_luna up/down times out -> WARN only (no landing)
    """

    def __init__(self):
        super().__init__("failsafe_monitor")

        # ---- Parameters you can tweak at runtime ----
        self.timeout_odom = float(self.declare_parameter("timeout_odom_s", 0.30).value)
        self.timeout_d500 = float(self.declare_parameter("timeout_d500_s", 0.50).value)
        self.timeout_tfluna = float(self.declare_parameter("timeout_tfluna_s", 0.80).value)

        self.check_period = float(self.declare_parameter("check_period_s", 0.10).value)

        # Optional: brief hold before landing (0 = land immediately)
        self.hold_before_land_s = float(self.declare_parameter("hold_before_land_s", 0.0).value)

        # ---- Topic names (match your system) ----
        self.topic_odom = self.declare_parameter("topic_odom", "/fmu/out/vehicle_odometry").value
        self.topic_d500 = self.declare_parameter("topic_d500", "/d500/scan").value
        self.topic_tfluna_down = self.declare_parameter("topic_tfluna_down", "/tf_luna/down/scan").value
        self.topic_tfluna_up = self.declare_parameter("topic_tfluna_up", "/tf_luna/up/scan").value

        # ---- Watchdogs ----
        now = time.time()
        self.wd_odom = Watchdog("vehicle_odometry", now, self.timeout_odom, critical=True)
        self.wd_d500 = Watchdog("d500_scan", now, self.timeout_d500, critical=True)
        self.wd_down = Watchdog("tf_luna_down", now, self.timeout_tfluna, critical=False)
        self.wd_up = Watchdog("tf_luna_up", now, self.timeout_tfluna, critical=False)

        self.failsafe_triggered = False

        # ---- PX4 command publisher + failsafe status publisher ----
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.failsafe_pub = self.create_publisher(Bool, "/failsafe_triggered", 10)
        
        # ---- QoS for PX4 topics (BEST_EFFORT) ----
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ---- Subscriptions ----
        self.create_subscription(VehicleOdometry, self.topic_odom, self._on_odom, px4_qos)
        self.create_subscription(LaserScan, self.topic_d500, self._on_d500, 10)
        self.create_subscription(LaserScan, self.topic_tfluna_down, self._on_down, 10)
        self.create_subscription(LaserScan, self.topic_tfluna_up, self._on_up, 10)

        # Periodic health check
        self.create_timer(self.check_period, self._health_check)

        self.get_logger().info(
            "FailsafeMonitor started.\n"
            f"  Critical: {self.topic_odom} (>{self.timeout_odom}s), {self.topic_d500} (>{self.timeout_d500}s)\n"
            f"  Warn-only: {self.topic_tfluna_down}, {self.topic_tfluna_up} (>{self.timeout_tfluna}s)\n"
            f"  hold_before_land_s={self.hold_before_land_s}"
        )

    # ---- Callbacks update watchdog timestamps ----
    def _on_odom(self, _msg: VehicleOdometry):
        self.wd_odom.last_time = time.time()

    def _on_d500(self, _msg: LaserScan):
        self.wd_d500.last_time = time.time()

    def _on_down(self, _msg: LaserScan):
        self.wd_down.last_time = time.time()

    def _on_up(self, _msg: LaserScan):
        self.wd_up.last_time = time.time()

    # ---- Main logic ----
    def _health_check(self):
        now = time.time()

        # Check each watchdog
        self._check_one(self.wd_odom, now)
        self._check_one(self.wd_d500, now)
        self._check_one(self.wd_down, now)
        self._check_one(self.wd_up, now)

    def _check_one(self, wd: Watchdog, now: float):
        dt = now - wd.last_time
        if dt <= wd.timeout_s:
            return

        if wd.critical:
            if not self.failsafe_triggered:
                self.get_logger().error(
                    f"CRITICAL FAULT: {wd.name} timeout ({dt:.2f}s > {wd.timeout_s}s). Triggering LAND."
                )
                self._trigger_failsafe()
        else:
            # warn-only (rate limited by only warning once per ~timeout window)
            # simple rate limit: reset last_time so it won't spam constantly
            self.get_logger().warn(f"Non-critical sensor warning: {wd.name} timeout ({dt:.2f}s > {wd.timeout_s}s).")
            wd.last_time = now

    def _trigger_failsafe(self):
        self.failsafe_triggered = True

        # Tell other nodes (like maze_navigator) to stop setpoints
        self.failsafe_pub.publish(Bool(data=True))

        if self.hold_before_land_s > 0.0:
            self.get_logger().warn(f"Holding {self.hold_before_land_s:.1f}s before LAND...")
            time.sleep(self.hold_before_land_s)

        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().warn("LAND command sent. (failsafe_triggered=True)")

    def _send_vehicle_command(self, command: int):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 0.0
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = FailsafeMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
