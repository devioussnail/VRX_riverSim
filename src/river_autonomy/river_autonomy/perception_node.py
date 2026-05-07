import math
from collections import deque
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32MultiArray


@dataclass
class RobotState:
    """Aggregated state from all sensors."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    timestamp: float = 0.0


class SensorBuffer:
    """Circular buffer for filtering sensor data."""
    def __init__(self, max_size: int = 5):
        self.buffer = deque(maxlen=max_size)
    
    def add(self, value: float):
        self.buffer.append(value)
    
    def mean(self) -> float:
        if not self.buffer:
            return 0.0
        return sum(self.buffer) / len(self.buffer)
    
    def latest(self) -> float:
        if not self.buffer:
            return 0.0
        return self.buffer[-1]


def yaw_from_quaternion(quaternion):
    """Extract yaw angle from quaternion."""
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PerceptionNode(Node):
    """
    Perception layer for the WAM-V autonomous boat.
    
    Subscribes to:
    - Odometry (ground truth position/velocity)
    - IMU (acceleration, angular velocity)
    - Lidar (2D range scan for obstacle detection)
    
    Publishes:
    - Robot state (aggregated pose + velocity)
    - Obstacle distances (front sector analysis)
    - Raw sensor data (for debugging/logging)
    """
    
    def __init__(self):
        super().__init__('perception_node')
        
        # Declare parameters
        self.declare_parameter('odom_topic', '/wamv/sensors/position/ground_truth_odometry')
        self.declare_parameter('imu_topic', '/wamv/sensors/imu/imu/data')
        self.declare_parameter('scan_topic', '/wamv/sensors/lidars/lidar_wamv_sensor/scan')
        self.declare_parameter('state_pub_topic', '/perception/robot_state')
        self.declare_parameter('obstacles_pub_topic', '/perception/obstacles')
        self.declare_parameter('lidar_filter_range', 50.0)  # meters, max range to process
        self.declare_parameter('lidar_sector_angle', 60.0)  # degrees, beam width for front sector
        self.declare_parameter('filter_buffer_size', 5)  # samples for moving average
        self.declare_parameter('publish_rate_hz', 10.0)
        
        # Get parameters
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.state_pub_topic = str(self.get_parameter('state_pub_topic').value)
        self.obstacles_pub_topic = str(self.get_parameter('obstacles_pub_topic').value)
        self.lidar_filter_range = float(self.get_parameter('lidar_filter_range').value)
        self.lidar_sector_angle = float(self.get_parameter('lidar_sector_angle').value)
        filter_buffer_size = int(self.get_parameter('filter_buffer_size').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        
        # Create subscriptions
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        
        # Create publishers
        self.state_pub = self.create_publisher(Float32MultiArray, self.state_pub_topic, 10)
        self.obstacles_pub = self.create_publisher(Float32MultiArray, self.obstacles_pub_topic, 10)
        
        # Current state
        self.robot_state = RobotState()
        self.latest_scan = None
        
        # Filtering buffers
        self.accel_filter = SensorBuffer(filter_buffer_size)
        self.yaw_rate_filter = SensorBuffer(filter_buffer_size)
        
        # Timer for periodic publishing
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_state)
        
        self.get_logger().info(
            f'Perception node started.\n'
            f'  Subscriptions: odom={self.odom_topic}, imu={self.imu_topic}, scan={self.scan_topic}\n'
            f'  Publications: state={self.state_pub_topic}, obstacles={self.obstacles_pub_topic}'
        )
    
    def odom_callback(self, msg: Odometry):
        """
        Update robot pose and velocity from odometry.
        
        Args:
            msg: Odometry message with pose and twist
        """
        # Position
        self.robot_state.x = msg.pose.pose.position.x
        self.robot_state.y = msg.pose.pose.position.y
        
        # Orientation (yaw only, for 2D ASV)
        self.robot_state.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        
        # Velocity (linear in base_link frame)
        self.robot_state.vx = msg.twist.twist.linear.x
        self.robot_state.vy = msg.twist.twist.linear.y
        
        # Angular velocity
        self.robot_state.vyaw = msg.twist.twist.angular.z
        
        self.robot_state.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    
    def imu_callback(self, msg: Imu):
        """
        Update acceleration and angular rates from IMU.
        
        Args:
            msg: Imu message with linear/angular accelerations
        """
        # Linear acceleration (filter for noise reduction)
        accel_magnitude = math.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2
        )
        self.accel_filter.add(accel_magnitude)
        self.robot_state.ax = msg.linear_acceleration.x
        self.robot_state.ay = msg.linear_acceleration.y
        self.robot_state.az = msg.linear_acceleration.z
        
        # Angular velocity (filter for noise reduction)
        self.yaw_rate_filter.add(abs(msg.angular_velocity.z))
    
    def scan_callback(self, msg: LaserScan):
        """
        Store latest lidar scan for processing.
        
        Args:
            msg: LaserScan message (16-beam lidar)
        """
        self.latest_scan = msg
    
    def process_lidar(self) -> dict:
        """
        Process lidar scan to extract obstacle information.
        
        Returns:
            Dictionary with obstacle distances in different sectors:
            {
                'front_left': distance to nearest obstacle in left sector,
                'front_center': distance to nearest obstacle in center sector,
                'front_right': distance to nearest obstacle in right sector,
                'min_distance': minimum distance to any obstacle,
                'nearest_bearing': bearing of nearest obstacle in front cone (rad),
            }
        """
        if self.latest_scan is None:
            return {
                'front_left': float('inf'),
                'front_center': float('inf'),
                'front_right': float('inf'),
                'min_distance': float('inf'),
                'nearest_bearing': float('nan'),
            }
        
        scan = self.latest_scan
        ranges = scan.ranges
        num_beams = len(ranges)
        
        # Filter out invalid ranges
        filtered_ranges = [
            r for r in ranges
            if scan.range_min <= r <= min(scan.range_max, self.lidar_filter_range)
        ]
        
        if not filtered_ranges:
            return {
                'front_left': float('inf'),
                'front_center': float('inf'),
                'front_right': float('inf'),
                'min_distance': float('inf'),
                'nearest_bearing': float('nan'),
            }
        
        # Split a front cone centered at 0 rad into left/center/right bins.
        # Using angles avoids assumptions about scan index ordering.
        half_front_angle = math.radians(self.lidar_sector_angle) * 0.5
        center_half_angle = half_front_angle / 3.0
        max_valid_range = min(scan.range_max, self.lidar_filter_range)

        front_left = float('inf')
        front_center = float('inf')
        front_right = float('inf')
        nearest_range = float('inf')
        nearest_bearing = float('nan')

        for i, r in enumerate(ranges):
            if not (scan.range_min <= r <= max_valid_range):
                continue

            beam_angle = normalize_angle(scan.angle_min + i * scan.angle_increment)

            # Keep only beams in the front cone.
            if abs(beam_angle) > half_front_angle:
                continue

            if beam_angle > center_half_angle:
                front_left = min(front_left, r)
            elif beam_angle < -center_half_angle:
                front_right = min(front_right, r)
            else:
                front_center = min(front_center, r)

            if r < nearest_range:
                nearest_range = r
                nearest_bearing = beam_angle
        
        min_distance = min(front_left, front_center, front_right)
        
        return {
            'front_left': front_left,
            'front_center': front_center,
            'front_right': front_right,
            'min_distance': min_distance,
            'nearest_bearing': nearest_bearing,
        }
    
    def publish_state(self):
        """Publish aggregated robot state to planning layer."""
        # Create state message: [x, y, yaw, vx, vy, vyaw, ax, ay, az, timestamp]
        state_msg = Float32MultiArray()
        state_msg.data = [
            float(self.robot_state.x),
            float(self.robot_state.y),
            float(self.robot_state.yaw),
            float(self.robot_state.vx),
            float(self.robot_state.vy),
            float(self.robot_state.vyaw),
            float(self.robot_state.ax),
            float(self.robot_state.ay),
            float(self.robot_state.az),
            float(self.robot_state.timestamp),
        ]
        self.state_pub.publish(state_msg)
        
        # Process and publish obstacle distances
        obstacles = self.process_lidar()
        obstacles_msg = Float32MultiArray()
        obstacles_msg.data = [
            float(obstacles['front_left']),
            float(obstacles['front_center']),
            float(obstacles['front_right']),
            float(obstacles['min_distance']),
            float(obstacles['nearest_bearing']),
        ]
        self.obstacles_pub.publish(obstacles_msg)
        
        # Optional: logging for debugging
        self.get_logger().debug(
            f'State: pos=({self.robot_state.x:.1f}, {self.robot_state.y:.1f}), '
            f'yaw={math.degrees(self.robot_state.yaw):.1f}°, '
            f'vel=({self.robot_state.vx:.2f}, {self.robot_state.vy:.2f}), '
            f'obstacles={obstacles["front_center"]:.1f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
