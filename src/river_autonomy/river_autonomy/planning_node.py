import math
import time

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
    """Stage-based planner for smooth obstacle pass maneuvers."""

    WAITING = 'waiting_for_perception'
    TRACK_GOAL = 'track_goal'
    APPROACH_SLOW = 'approach_slow'
    COMMIT_TURN = 'commit_turn'
    PASS_OBSTACLE = 'pass_obstacle'
    RECOVER_TO_GOAL = 'recover_to_goal'
    GOAL_REACHED = 'goal_reached'

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

        self.declare_parameter('cruise_speed', 1.4)
        self.declare_parameter('slow_speed', 0.5)
        self.declare_parameter('recover_speed', 0.9)
        self.declare_parameter('pass_speed', 0.65)

        self.declare_parameter('stop_distance', 6.0)
        self.declare_parameter('critical_stop_distance', 4.0)
        self.declare_parameter('caution_distance', 14.0)
        self.declare_parameter('stop_exit_distance', 8.0)
        self.declare_parameter('caution_exit_distance', 16.0)

        self.declare_parameter('avoidance_yaw_rate', 0.45)
        self.declare_parameter('turn_heading_offset_deg', 34.0)
        self.declare_parameter('pass_heading_offset_deg', 18.0)

        self.declare_parameter('commit_min_hold_sec', 1.8)
        self.declare_parameter('pass_min_hold_sec', 2.2)
        self.declare_parameter('recover_heading_tolerance_deg', 10.0)

        self.declare_parameter('obstacle_filter_alpha', 0.2)
        self.declare_parameter('turn_hold_time_sec', 1.5)
        self.declare_parameter('turn_switch_margin', 1.2)
        self.declare_parameter('speed_slew_rate', 0.4)
        self.declare_parameter('yaw_rate_slew_rate', 0.55)

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
        self.recover_speed = float(self.get_parameter('recover_speed').value)
        self.pass_speed = float(self.get_parameter('pass_speed').value)

        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.critical_stop_distance = float(self.get_parameter('critical_stop_distance').value)
        self.caution_distance = float(self.get_parameter('caution_distance').value)
        self.stop_exit_distance = float(self.get_parameter('stop_exit_distance').value)
        self.caution_exit_distance = float(self.get_parameter('caution_exit_distance').value)

        self.avoidance_yaw_rate = float(self.get_parameter('avoidance_yaw_rate').value)
        self.turn_heading_offset = math.radians(float(self.get_parameter('turn_heading_offset_deg').value))
        self.pass_heading_offset = math.radians(float(self.get_parameter('pass_heading_offset_deg').value))

        self.commit_min_hold_sec = float(self.get_parameter('commit_min_hold_sec').value)
        self.pass_min_hold_sec = float(self.get_parameter('pass_min_hold_sec').value)
        self.recover_heading_tolerance = math.radians(float(self.get_parameter('recover_heading_tolerance_deg').value))

        self.obstacle_filter_alpha = float(self.get_parameter('obstacle_filter_alpha').value)
        self.turn_hold_time_sec = float(self.get_parameter('turn_hold_time_sec').value)
        self.turn_switch_margin = float(self.get_parameter('turn_switch_margin').value)
        self.speed_slew_rate = float(self.get_parameter('speed_slew_rate').value)
        self.yaw_rate_slew_rate = float(self.get_parameter('yaw_rate_slew_rate').value)

        plan_rate_hz = float(self.get_parameter('plan_rate_hz').value)
        self.plan_dt = 1.0 / max(plan_rate_hz, 1e-6)

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.front_left_filt = float('inf')
        self.front_center_filt = float('inf')
        self.front_right_filt = float('inf')
        self.min_distance_filt = float('inf')
        self.nearest_bearing_filt = float('nan')

        self.last_cmd_speed = 0.0
        self.last_cmd_yaw_rate = 0.0

        self.state = self.WAITING
        self.state_enter_time = time.monotonic()
        self.goal_reached = False

        self.turn_direction = 1.0
        self.last_turn_switch_time = 0.0

        self.timer = self.create_timer(self.plan_dt, self.plan_loop)

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

        front_left = float(msg.data[0])
        front_center = float(msg.data[1])
        front_right = float(msg.data[2])
        min_distance = float(msg.data[3])
        nearest_bearing = float(msg.data[4]) if len(msg.data) >= 5 else float('nan')

        self.front_left_filt = self.low_pass(self.front_left_filt, front_left)
        self.front_center_filt = self.low_pass(self.front_center_filt, front_center)
        self.front_right_filt = self.low_pass(self.front_right_filt, front_right)
        self.min_distance_filt = self.low_pass(self.min_distance_filt, min_distance)
        self.nearest_bearing_filt = self.low_pass_angle(self.nearest_bearing_filt, nearest_bearing)

    def set_state(self, state):
        if state != self.state:
            self.state = state
            self.state_enter_time = time.monotonic()
            self.get_logger().info(f'Planning mode: {state}')

    def state_elapsed(self):
        return time.monotonic() - self.state_enter_time

    def low_pass(self, current, measurement):
        if not math.isfinite(measurement):
            return measurement
        if not math.isfinite(current):
            return measurement
        alpha = clamp(self.obstacle_filter_alpha, 0.0, 1.0)
        return (1.0 - alpha) * current + alpha * measurement

    def low_pass_angle(self, current, measurement):
        if not math.isfinite(measurement):
            return measurement
        if not math.isfinite(current):
            return measurement
        alpha = clamp(self.obstacle_filter_alpha, 0.0, 1.0)
        s = (1.0 - alpha) * math.sin(current) + alpha * math.sin(measurement)
        c = (1.0 - alpha) * math.cos(current) + alpha * math.cos(measurement)
        return math.atan2(s, c)

    def slew(self, current, target, rate_limit):
        max_delta = max(rate_limit, 1e-6) * self.plan_dt
        delta = clamp(target - current, -max_delta, max_delta)
        return current + delta

    def publish_targets(self, heading, speed, yaw_rate):
        speed = self.slew(self.last_cmd_speed, speed, self.speed_slew_rate)
        yaw_rate = self.slew(self.last_cmd_yaw_rate, yaw_rate, self.yaw_rate_slew_rate)

        heading_msg = Float64()
        heading_msg.data = float(heading)
        self.target_heading_pub.publish(heading_msg)

        speed_msg = Float64()
        speed_msg.data = float(speed)
        self.target_speed_pub.publish(speed_msg)

        yaw_rate_msg = Float64()
        yaw_rate_msg.data = float(yaw_rate)
        self.target_yaw_rate_pub.publish(yaw_rate_msg)

        self.last_cmd_speed = speed
        self.last_cmd_yaw_rate = yaw_rate

    def choose_turn_direction(self):
        now = time.monotonic()
        prefer = self.turn_direction

        # If nearest bearing is known, steer away from it.
        if math.isfinite(self.nearest_bearing_filt):
            prefer = -1.0 if self.nearest_bearing_filt > 0.0 else 1.0

        # When left/right clearances are significantly different, use that signal.
        if math.isfinite(self.front_left_filt) and math.isfinite(self.front_right_filt):
            gap = abs(self.front_left_filt - self.front_right_filt)
            if gap >= self.turn_switch_margin:
                prefer = 1.0 if self.front_left_filt > self.front_right_filt else -1.0

        if prefer != self.turn_direction and (now - self.last_turn_switch_time) >= self.turn_hold_time_sec:
            self.turn_direction = prefer
            self.last_turn_switch_time = now

        return self.turn_direction

    def plan_loop(self):
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            self.set_state(self.WAITING)
            self.publish_targets(0.0, 0.0, 0.0)
            return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance_to_goal = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx)

        if distance_to_goal <= self.goal_tolerance:
            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info('Goal reached. Holding position command.')
            self.set_state(self.GOAL_REACHED)
            self.publish_targets(self.current_yaw, 0.0, 0.0)
            return

        heading_error = normalize_angle(desired_heading - self.current_yaw)

        # Enter obstacle sequence from nominal navigation states.
        if self.state in (self.TRACK_GOAL, self.RECOVER_TO_GOAL) and self.min_distance_filt < self.caution_distance:
            self.choose_turn_direction()
            self.set_state(self.APPROACH_SLOW)

        if self.state == self.WAITING:
            self.set_state(self.TRACK_GOAL)

        if self.state == self.APPROACH_SLOW:
            turn_direction = self.choose_turn_direction()
            yaw_rate = turn_direction * self.avoidance_yaw_rate * 0.45
            self.publish_targets(desired_heading, self.slow_speed, yaw_rate)

            if self.min_distance_filt < self.stop_distance:
                self.set_state(self.COMMIT_TURN)
                return

            if self.min_distance_filt > self.caution_exit_distance and self.state_elapsed() > 0.8:
                self.set_state(self.RECOVER_TO_GOAL)
                return

        elif self.state == self.COMMIT_TURN:
            turn_direction = self.choose_turn_direction()
            turn_heading = normalize_angle(self.current_yaw + turn_direction * self.turn_heading_offset)
            turn_speed = 0.0 if self.min_distance_filt < self.critical_stop_distance else self.slow_speed
            yaw_rate = turn_direction * self.avoidance_yaw_rate
            self.publish_targets(turn_heading, turn_speed, yaw_rate)

            front_is_clear = (
                self.front_center_filt > self.stop_exit_distance
                or self.min_distance_filt > self.stop_exit_distance
            )
            if self.state_elapsed() > self.commit_min_hold_sec and front_is_clear:
                self.set_state(self.PASS_OBSTACLE)
                return

        elif self.state == self.PASS_OBSTACLE:
            turn_direction = self.turn_direction
            pass_heading = normalize_angle(desired_heading + turn_direction * self.pass_heading_offset)
            yaw_rate = turn_direction * self.avoidance_yaw_rate * 0.6
            self.publish_targets(pass_heading, self.pass_speed, yaw_rate)

            if self.min_distance_filt < self.critical_stop_distance:
                self.set_state(self.COMMIT_TURN)
                return

            if self.state_elapsed() > self.pass_min_hold_sec and self.min_distance_filt > self.caution_exit_distance:
                self.set_state(self.RECOVER_TO_GOAL)
                return

        elif self.state == self.RECOVER_TO_GOAL:
            yaw_rate = clamp(heading_error, -0.55, 0.55)
            self.publish_targets(desired_heading, self.recover_speed, yaw_rate)

            if self.min_distance_filt < self.caution_distance:
                self.set_state(self.APPROACH_SLOW)
                return

            if abs(heading_error) < self.recover_heading_tolerance and self.min_distance_filt > self.caution_exit_distance:
                self.set_state(self.TRACK_GOAL)
                return

        elif self.state == self.GOAL_REACHED:
            self.publish_targets(self.current_yaw, 0.0, 0.0)

        else:
            self.set_state(self.TRACK_GOAL)
            yaw_rate = clamp(heading_error, -0.6, 0.6)
            self.publish_targets(desired_heading, self.cruise_speed, yaw_rate)

        if self.state == self.TRACK_GOAL:
            yaw_rate = clamp(heading_error, -0.6, 0.6)
            self.publish_targets(desired_heading, self.cruise_speed, yaw_rate)


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
