from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    control_node = Node(
        package='river_autonomy',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[
            {'robot_state_topic': '/perception/robot_state'},
            {'target_heading_topic': '/planning/target_heading'},
            {'target_speed_topic': '/planning/target_speed'},
            {'target_yaw_rate_topic': '/planning/target_yaw_rate'},
            {'left_thrust_topic': '/wamv/thrusters/left/thrust'},
            {'right_thrust_topic': '/wamv/thrusters/right/thrust'},
            {'left_pos_topic': '/wamv/thrusters/left/pos'},
            {'right_pos_topic': '/wamv/thrusters/right/pos'},
            {'base_thrust_gain': 420.0},
            {'max_thrust': 900.0},
            {'heading_kp': 1.5},
            {'yaw_rate_kp': 250.0},
            {'max_pod_angle': 0.9},
            {'control_rate_hz': 20.0},
        ],
    )

    return LaunchDescription([control_node])
