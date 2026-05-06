import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PlanningNode(Node):
    """Planning layer for Perceive-Plan-Control architecture.

    Subscribes to processed perception outputs and publishes motion goals
    for a future control node.
    """

    def __init__(self):
        super().__init__('planning_node')

        self.declare_parameter('robot_state_topic', '/perception/robot_state')
        self.declare_parameter('obstacles_topic', '/perception/obstacles')
        self.declare_parameter('target_heading_topic', '/planning/target_heading')
        self.declare_parameter('target_speed_topic', '/planning/target_speed')
        self.declare_parameter('target_yaw_rate_topic', '/planning/target_yaw_rate')

        self.declare_parameter('goal_x', 140.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 4.0)

        self.declare_parameter('cruise_speed', 2.0)
        self.declare_parameter('slow_speed', 0.8)
        self.declare_parameter('stop_distance', 4.0)
        self.declare_parameter('caution_distance', 10.0)
        self.declare_parameter('avoidance_yaw_rate', 0.5)

        self.declare_parameter('plan_rate_hz', 10.0)

        robot_state_topic = str(self.get_parameter('robot_state_topic').value)
        obstacles_topic = str(self.get_parameter('obstacles_topic').value)

        self.target_heading_pub = self.create_publisher(
            Float64, str(self.get_parameter('target_heading_topic').value), 10
        )
        self.target_speed_pub = self.create_publisher(
            Float64, str(self.get_parameter('target_speed_topic').value), 10
        )
        self.target_yaw_rate_pub = self.create_publisher(
            Float64, str(self.get_parameter('target_yaw_rate_topic').value), 10
        )

        self.create_subscription(
            Float32MultiArray, robot_state_topic, self.robot_state_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, obstacles_topic, self.obstacles_callback, 10
        )

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.slow_speed = float(self.get_parameter('slow_speed').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.caution_distance = float(self.get_parameter('caution_distance').value)
        self.avoidance_yaw_rate = float(self.get_parameter('avoidance_yaw_rate').value)

        plan_rate_hz = float(self.get_parameter('plan_rate_hz').value)

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.front_left = float('inf')
        self.front_center = float('inf')
        self.front_right = float('inf')
        self.min_distance = float('inf')

        self.goal_reached = False
        self.last_mode = None

        self.timer = self.create_timer(1.0 / plan_rate_hz, self.plan_loop)

        self.get_logger().info(
            f'Planning node started. Goal=({self.goal_x:.1f}, {self.goal_y:.1f}), '
            f'state={robot_state_topic}, obstacles={obstacles_topic}'
        )

    def robot_state_callback(self, msg):
        if len(msg.data) < 3:
            return
        self.current_x = float(msg.data[0])
        self.current_y = float(msg.data[1])
        self.current_yaw = float(msg.data[2])

    def obstacles_callback(self, msg):
        if len(msg.data) < 4:
            return
        self.front_left = float(msg.data[0])
        self.front_center = float(msg.data[1])
        self.front_right = float(msg.data[2])
        self.min_distance = float(msg.data[3])

    def set_mode(self, mode):
        if mode != self.last_mode:
            self.last_mode = mode
            self.get_logger().info(f'Planning mode: {mode}')

    def publish_targets(self, heading, speed, yaw_rate):
        heading_msg = Float64()
        heading_msg.data = float(heading)
        self.target_heading_pub.publish(heading_msg)

        speed_msg = Float64()
        speed_msg.data = float(speed)
        self.target_speed_pub.publish(speed_msg)

        yaw_rate_msg = Float64()
        yaw_rate_msg.data = float(yaw_rate)
        self.target_yaw_rate_pub.publish(yaw_rate_msg)

    def plan_loop(self):
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            self.set_mode('waiting_for_perception')
            self.publish_targets(0.0, 0.0, 0.0)
            return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance_to_goal = math.hypot(dx, dy)

        if distance_to_goal <= self.goal_tolerance:
            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info('Goal reached. Holding position command.')
            self.set_mode('goal_reached')
            self.publish_targets(self.current_yaw, 0.0, 0.0)
            return

        desired_heading = math.atan2(dy, dx)

        if self.min_distance < self.stop_distance:
            self.set_mode('emergency_avoid')
            turn_direction = 1.0 if self.front_left > self.front_right else -1.0
            yaw_rate = turn_direction * self.avoidance_yaw_rate
            self.publish_targets(self.current_yaw, 0.0, yaw_rate)
            return

        if self.min_distance < self.caution_distance:
            self.set_mode('avoid_with_forward_motion')
            turn_direction = 1.0 if self.front_left > self.front_right else -1.0
            yaw_rate = turn_direction * self.avoidance_yaw_rate * 0.7
            self.publish_targets(desired_heading, self.slow_speed, yaw_rate)
            return

        self.set_mode('track_goal')
        heading_error = normalize_angle(desired_heading - self.current_yaw)
        yaw_rate = clamp(heading_error, -0.8, 0.8)
        self.publish_targets(desired_heading, self.cruise_speed, yaw_rate)


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
