from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    planning_node = Node(
        package='river_autonomy',
        executable='planning_node',
        name='planning_node',
        output='screen',
        parameters=[
            {'robot_state_topic': '/perception/robot_state'},
            {'obstacles_topic': '/perception/obstacles'},
            {'target_heading_topic': '/planning/target_heading'},
            {'target_speed_topic': '/planning/target_speed'},
            {'target_yaw_rate_topic': '/planning/target_yaw_rate'},
            {'goal_x': 140.0},
            {'goal_y': 0.0},
            {'goal_tolerance': 4.0},
            {'cruise_speed': 1.4},
            {'slow_speed': 0.5},
            {'recover_speed': 0.9},
            {'pass_speed': 0.65},
            {'stop_distance': 6.0},
            {'critical_stop_distance': 4.0},
            {'caution_distance': 14.0},
            {'stop_exit_distance': 8.0},
            {'caution_exit_distance': 16.0},
            {'avoidance_yaw_rate': 0.45},
            {'turn_heading_offset_deg': 34.0},
            {'pass_heading_offset_deg': 18.0},
            {'commit_min_hold_sec': 1.8},
            {'pass_min_hold_sec': 2.2},
            {'recover_heading_tolerance_deg': 10.0},
            {'obstacle_filter_alpha': 0.2},
            {'turn_hold_time_sec': 1.5},
            {'turn_switch_margin': 1.2},
            {'speed_slew_rate': 0.4},
            {'yaw_rate_slew_rate': 0.55},
            {'plan_rate_hz': 10.0},
        ],
    )

    return LaunchDescription([planning_node])
