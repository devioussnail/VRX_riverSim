from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    perception_node = Node(
        package='river_autonomy',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[
            {'odom_topic': '/wamv/sensors/position/ground_truth_odometry'},
            {'imu_topic': '/wamv/sensors/imu/imu/data'},
            {'scan_topic': '/wamv/sensors/lidars/lidar_wamv_sensor/scan'},
            {'state_pub_topic': '/perception/robot_state'},
            {'obstacles_pub_topic': '/perception/obstacles'},
            {'lidar_filter_range': 50.0},
            {'lidar_sector_angle': 60.0},
            {'filter_buffer_size': 5},
            {'publish_rate_hz': 10.0},
        ],
    )

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

    return LaunchDescription([
        perception_node,
        planning_node,
        control_node,
    ])
