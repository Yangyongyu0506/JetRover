from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

my_pkg_share_dir = get_package_share_directory('ugv_slam')

def generate_launch_description():
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=['-configuration_directory', os.path.join(my_pkg_share_dir, 'config'),
                   '-configuration_basename', 'cartographer.lua'],
        remappings=[('imu', 'imu/data'), ('odom', 'odometry/filtered')],
    )
    occupancy_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )
    return LaunchDescription([
        cartographer_node,
        occupancy_node,
    ])
