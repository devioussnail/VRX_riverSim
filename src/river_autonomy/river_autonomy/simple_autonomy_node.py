import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class SimpleAutonomyNode(Node):
    def __init__(self):
        super().__init__('simple_autonomy_node')

        self.declare_parameter('odom_topic', '/wamv/sensors/position/ground_truth_odometry')
        self.declare_parameter('imu_topic', '/wamv/sensors/imu/imu/data')
        self.declare_parameter('scan_topic', '/wamv/sensors/lidars/lidar_wamv_sensor/scan')
        self.declare_parameter('left_thrust_topic', '/wamv/thrusters/left/thrust')
        self.declare_parameter('right_thrust_topic', '/wamv/thrusters/right/thrust')
        self.declare_parameter('left_pos_topic', '/wamv/thrusters/left/pos')
        self.declare_parameter('right_pos_topic', '/wamv/thrusters/right/pos')
        self.declare_parameter('goal_x', 120.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 5.0)
        self.declare_parameter('base_thrust', 450.0)
        self.declare_parameter('max_thrust', 900.0)
        self.declare_parameter('heading_kp', 250.0)
        self.declare_parameter('obstacle_distance', 12.0)
        self.declare_parameter('danger_distance', 6.0)
        self.declare_parameter('avoidance_turn', 0.8)
        self.declare_parameter('upstream_heading', 0.0)
        self.declare_parameter('control_rate_hz', 10.0)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.base_thrust = float(self.get_parameter('base_thrust').value)
        self.max_thrust = float(self.get_parameter('max_thrust').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.obstacle_distance = float(self.get_parameter('obstacle_distance').value)
        self.danger_distance = float(self.get_parameter('danger_distance').value)
        self.avoidance_turn = float(self.get_parameter('avoidance_turn').value)
        self.upstream_heading = float(self.get_parameter('upstream_heading').value)
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        odom_topic = str(self.get_parameter('odom_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)

        self.left_thrust_pub = self.create_publisher(
            Float64, str(self.get_parameter('left_thrust_topic').value), 10)
        self.right_thrust_pub = self.create_publisher(
            Float64, str(self.get_parameter('right_thrust_topic').value), 10)
        self.left_pos_pub = self.create_publisher(
            Float64, str(self.get_parameter('left_pos_topic').value), 10)
        self.right_pos_pub = self.create_publisher(
            Float64, str(self.get_parameter('right_pos_topic').value), 10)

        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.latest_scan = None
        self.goal_reached = False
        self.last_mode = None
        self.reported_odom_fallback = False

        self.timer = self.create_timer(1.0 / control_rate_hz, self.control_loop)

        self.get_logger().info(
            f'Simple autonomy started. Goal=({self.goal_x:.1f}, {self.goal_y:.1f}), '
            f'odom={odom_topic}, imu={imu_topic}, scan={scan_topic}')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def imu_callback(self, msg):
        if self.current_yaw is None:
            self.current_yaw = yaw_from_quaternion(msg.orientation)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def publish_thrusters(self, left_thrust, right_thrust, left_pos=0.0, right_pos=0.0):
        self.left_pos_pub.publish(Float64(data=left_pos))
        self.right_pos_pub.publish(Float64(data=right_pos))
        self.left_thrust_pub.publish(Float64(data=left_thrust))
        self.right_thrust_pub.publish(Float64(data=right_thrust))

    def stop_boat(self):
        self.publish_thrusters(0.0, 0.0)

    def front_sector_distances(self):
        if self.latest_scan is None:
            return None

        left_min = float('inf')
        center_min = float('inf')
        right_min = float('inf')

        angle = self.latest_scan.angle_min
        for range_value in self.latest_scan.ranges:
            if math.isfinite(range_value):
                if 0.15 < angle <= 0.9:
                    left_min = min(left_min, range_value)
                elif -0.15 <= angle <= 0.15:
                    center_min = min(center_min, range_value)
                elif -0.9 <= angle < -0.15:
                    right_min = min(right_min, range_value)
            angle += self.latest_scan.angle_increment

        return {
            'left': left_min,
            'center': center_min,
            'right': right_min,
        }

    def control_loop(self):
        if self.current_yaw is None or self.latest_scan is None:
            if self.last_mode != 'waiting':
                self.get_logger().info('Waiting for heading and lidar data before moving.')
                self.last_mode = 'waiting'
            self.stop_boat()
            return

        distance_to_goal = None
        target_heading = self.upstream_heading
        mode = 'track_heading'

        if self.current_x is not None and self.current_y is not None:
            dx = self.goal_x - self.current_x
            dy = self.goal_y - self.current_y
            distance_to_goal = math.hypot(dx, dy)
            target_heading = math.atan2(dy, dx)
            mode = 'track_goal'

            if distance_to_goal <= self.goal_tolerance:
                if not self.goal_reached:
                    self.goal_reached = True
                    self.get_logger().info('Reached upstream goal, stopping boat.')
                self.stop_boat()
                return
        elif not self.reported_odom_fallback:
            self.get_logger().info(
                'Odometry topic is not available, falling back to IMU heading-hold mode. '
                'The boat will keep driving upstream but will not stop at a goal point.')
            self.reported_odom_fallback = True

        sectors = self.front_sector_distances()

        if sectors is not None and sectors['center'] < self.obstacle_distance:
            mode = 'avoid_obstacle'
            if sectors['left'] > sectors['right']:
                target_heading += self.avoidance_turn
            else:
                target_heading -= self.avoidance_turn

        heading_error = normalize_angle(target_heading - self.current_yaw)
        turn_command = clamp(self.heading_kp * heading_error, -self.max_thrust * 0.5, self.max_thrust * 0.5)

        base_thrust = self.base_thrust
        if sectors is not None and sectors['center'] < self.danger_distance:
            base_thrust *= 0.4
        elif sectors is not None and sectors['center'] < self.obstacle_distance:
            base_thrust *= 0.7

        left_thrust = clamp(base_thrust - turn_command, -self.max_thrust, self.max_thrust)
        right_thrust = clamp(base_thrust + turn_command, -self.max_thrust, self.max_thrust)

        self.publish_thrusters(left_thrust, right_thrust)

        if mode != self.last_mode:
            status = f'Mode={mode}'
            if self.current_x is not None and self.current_y is not None and distance_to_goal is not None:
                status += f' x={self.current_x:.1f} y={self.current_y:.1f} goal_error={distance_to_goal:.1f}m'
            else:
                status += f' heading={self.current_yaw:.2f}rad'
            self.get_logger().info(status)
            self.last_mode = mode


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAutonomyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_boat()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
