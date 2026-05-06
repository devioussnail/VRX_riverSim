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


class ControlNode(Node):
    """Control layer for Perceive-Plan-Control architecture.

    Subscribes to planning outputs and current robot state, then computes
    left/right thrust and pod steering angles for WAM-V thrusters.
    """

    def __init__(self):
        super().__init__('control_node')

        self.declare_parameter('robot_state_topic', '/perception/robot_state')
        self.declare_parameter('target_heading_topic', '/planning/target_heading')
        self.declare_parameter('target_speed_topic', '/planning/target_speed')
        self.declare_parameter('target_yaw_rate_topic', '/planning/target_yaw_rate')

        self.declare_parameter('left_thrust_topic', '/wamv/thrusters/left/thrust')
        self.declare_parameter('right_thrust_topic', '/wamv/thrusters/right/thrust')
        self.declare_parameter('left_pos_topic', '/wamv/thrusters/left/pos')
        self.declare_parameter('right_pos_topic', '/wamv/thrusters/right/pos')

        self.declare_parameter('base_thrust_gain', 420.0)
        self.declare_parameter('max_thrust', 900.0)
        self.declare_parameter('heading_kp', 1.5)
        self.declare_parameter('yaw_rate_kp', 250.0)
        self.declare_parameter('max_pod_angle', 0.9)
        self.declare_parameter('control_rate_hz', 20.0)

        robot_state_topic = str(self.get_parameter('robot_state_topic').value)
        target_heading_topic = str(self.get_parameter('target_heading_topic').value)
        target_speed_topic = str(self.get_parameter('target_speed_topic').value)
        target_yaw_rate_topic = str(self.get_parameter('target_yaw_rate_topic').value)

        self.left_thrust_pub = self.create_publisher(
            Float64, str(self.get_parameter('left_thrust_topic').value), 10
        )
        self.right_thrust_pub = self.create_publisher(
            Float64, str(self.get_parameter('right_thrust_topic').value), 10
        )
        self.left_pos_pub = self.create_publisher(
            Float64, str(self.get_parameter('left_pos_topic').value), 10
        )
        self.right_pos_pub = self.create_publisher(
            Float64, str(self.get_parameter('right_pos_topic').value), 10
        )

        self.create_subscription(
            Float32MultiArray, robot_state_topic, self.robot_state_callback, 10
        )
        self.create_subscription(
            Float64, target_heading_topic, self.target_heading_callback, 10
        )
        self.create_subscription(
            Float64, target_speed_topic, self.target_speed_callback, 10
        )
        self.create_subscription(
            Float64, target_yaw_rate_topic, self.target_yaw_rate_callback, 10
        )

        self.base_thrust_gain = float(self.get_parameter('base_thrust_gain').value)
        self.max_thrust = float(self.get_parameter('max_thrust').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.yaw_rate_kp = float(self.get_parameter('yaw_rate_kp').value)
        self.max_pod_angle = float(self.get_parameter('max_pod_angle').value)
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        self.current_yaw = None
        self.current_yaw_rate = 0.0

        self.target_heading = None
        self.target_speed = 0.0
        self.target_yaw_rate = 0.0

        self.last_mode = None

        self.timer = self.create_timer(1.0 / control_rate_hz, self.control_loop)

        self.get_logger().info(
            'Control node started. Subscriptions: '
            f'state={robot_state_topic}, heading={target_heading_topic}, '
            f'speed={target_speed_topic}, yaw_rate={target_yaw_rate_topic}'
        )

    def robot_state_callback(self, msg):
        if len(msg.data) < 6:
            return
        self.current_yaw = float(msg.data[2])
        self.current_yaw_rate = float(msg.data[5])

    def target_heading_callback(self, msg):
        self.target_heading = float(msg.data)

    def target_speed_callback(self, msg):
        self.target_speed = float(msg.data)

    def target_yaw_rate_callback(self, msg):
        self.target_yaw_rate = float(msg.data)

    def set_mode(self, mode):
        if mode != self.last_mode:
            self.last_mode = mode
            self.get_logger().info(f'Control mode: {mode}')

    def publish_thrusters(self, left_thrust, right_thrust, left_pos, right_pos):
        left_thrust_msg = Float64()
        left_thrust_msg.data = float(left_thrust)
        self.left_thrust_pub.publish(left_thrust_msg)

        right_thrust_msg = Float64()
        right_thrust_msg.data = float(right_thrust)
        self.right_thrust_pub.publish(right_thrust_msg)

        left_pos_msg = Float64()
        left_pos_msg.data = float(left_pos)
        self.left_pos_pub.publish(left_pos_msg)

        right_pos_msg = Float64()
        right_pos_msg.data = float(right_pos)
        self.right_pos_pub.publish(right_pos_msg)

    def control_loop(self):
        if self.current_yaw is None or self.target_heading is None:
            self.set_mode('waiting_for_planning_or_state')
            self.publish_thrusters(0.0, 0.0, 0.0, 0.0)
            return

        heading_error = normalize_angle(self.target_heading - self.current_yaw)
        yaw_rate_error = self.target_yaw_rate - self.current_yaw_rate

        desired_yaw_effort = self.heading_kp * heading_error + yaw_rate_error
        steer_cmd = clamp(-desired_yaw_effort, -self.max_pod_angle, self.max_pod_angle)

        forward_thrust = clamp(
            self.target_speed * self.base_thrust_gain,
            0.0,
            self.max_thrust,
        )

        turn_delta = clamp(self.yaw_rate_kp * desired_yaw_effort, -0.4 * self.max_thrust, 0.4 * self.max_thrust)

        left_thrust = clamp(forward_thrust - turn_delta, -self.max_thrust, self.max_thrust)
        right_thrust = clamp(forward_thrust + turn_delta, -self.max_thrust, self.max_thrust)

        self.set_mode('tracking_targets')
        self.publish_thrusters(left_thrust, right_thrust, steer_cmd, steer_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
