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
            {'cruise_speed': 2.0},
            {'slow_speed': 0.8},
            {'stop_distance': 4.0},
            {'caution_distance': 10.0},
            {'avoidance_yaw_rate': 0.5},
            {'plan_rate_hz': 10.0},
        ],
    )

    return LaunchDescription([planning_node])
