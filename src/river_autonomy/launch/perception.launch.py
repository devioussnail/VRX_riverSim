from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the perception node.
    
    The perception node subscribes to sensor data (odometry, IMU, lidar)
    and publishes processed state information for the planning layer.
    """
    
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
    
    return LaunchDescription([perception_node])
