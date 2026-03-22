from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('ugv_localization_py')

def generate_launch_description():
    robot_description_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(os.path.join(get_package_share_directory('ugv_description'), 'urdf', 'ugv_rover.urdf'), 'r').read()}]
    )
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        output='screen',
        parameters=[{
            'publish_tf': False,
            'use_mag': False,
            'gain': 0.5,
        }]
    )
    robot_localization_efk_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[    
            os.path.join(pkg_share, 'config/ekf.yaml'),
        ]
    )
    encoder_node = Node(
        package='ugv_localization_py',
        executable='encoder_node',
        name='encoder_node',
        output='screen',
    )

    return LaunchDescription([
        robot_description_node,
        encoder_node,
        imu_filter_node,
        robot_localization_efk_node,
    ])