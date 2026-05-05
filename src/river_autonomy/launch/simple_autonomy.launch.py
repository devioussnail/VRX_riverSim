from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value='120.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('imu_topic', default_value='/wamv/sensors/imu/imu/data'),
        DeclareLaunchArgument('scan_topic', default_value='/wamv/sensors/lidars/lidar_wamv_sensor/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/wamv/sensors/position/ground_truth_odometry'),
        Node(
            package='river_autonomy',
            executable='simple_autonomy_node',
            name='simple_autonomy_node',
            output='screen',
            parameters=[{
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
            }],
        ),
    ])
