import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleCommand

class SimpleAvoidance(Node):
    def __init__(self):
        super().__init__('simple_avoidance')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers & Subscriptions
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.obstacle_detected = False
        self.min_safe_distance = 8.0  # Increased slightly for higher speed safety
        self.offboard_setpoint_counter = 0

    def lidar_callback(self, msg):
        # 1. Focus on the front 60-degree cone
        # With a 270-deg scan, the front is the middle of the array
        total_points = len(msg.ranges)
        center_index = total_points // 2
        window_size = total_points // 6  # Looks at 1/6th of total FOV in the center
        
        front_window = msg.ranges[center_index - window_size : center_index + window_size]
        
        # 2. Filter out .inf and 0.0
        valid_distances = [d for d in front_window if 0.15 < d < 30.0]
        
        if valid_distances:
            current_min = min(valid_distances)
            # Log every half second to keep terminal clean
            self.get_logger().info(f"Clear path ahead. Nearest object: {current_min:.2f}m", throttle_duration_sec=0.5)

            if current_min < self.min_safe_distance:
                self.get_logger().warn(f"!!! BRAKING !!! Wall detected at {current_min:.2f}m")
                self.obstacle_detected = True
            else:
                self.obstacle_detected = False
        else:
            self.obstacle_detected = False

    def timer_callback(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Heartbeat
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = timestamp
        offboard_msg.position, offboard_msg.velocity = True, False
        offboard_msg.acceleration, offboard_msg.attitude, offboard_msg.body_rate = False, False, False
        self.offboard_mode_pub.publish(offboard_msg)

        # Auto-switch to Offboard
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
        if self.offboard_setpoint_counter == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.get_logger().info("Switched to OFFBOARD mode")

        # Command Logic
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = timestamp
        
        if self.obstacle_detected:
            # STOP immediately
            trajectory_msg.position = [float('nan'), float('nan'), float('nan')]
            trajectory_msg.velocity = [0.0, 0.0, 0.0] 
        else:
            # TARGET: 30m forward, 5m high
            trajectory_msg.position = [30.0, 0.0, -5.0] 
        
        self.trajectory_pub.publish(trajectory_msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command, msg.param1, msg.param2 = command, param1, param2
        msg.target_system, msg.target_component = 1, 1
        msg.source_system, msg.source_component = 1, 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
