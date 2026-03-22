from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    serial_node = Node(
        package='ugv_bringup_py',
        executable='serial_node',
        name='serial_node',
        output='screen',
        parameters=[{
            'do_servo_calib': False,
            'sample_period_ms': 10,
        }]
    )
    joyparser_node = Node(
        package='ugv_bringup_py',
        executable='joyparser_node',
        name='joyparser_node',
        output='screen',
    )
    camera_node = Node(
        package='ugv_bringup_py',
        executable='camera_node',
        name='camera_node',
        output='screen',
    )
    lidar_node = Node(
      package='ldlidar_stl_ros2',
      executable='ldlidar_stl_ros2_node',
      name='LD06',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD06'},
        {'topic_name': 'scan'},
        {'frame_id': 'base_lidar_link'},
        {'port_name': '/dev/ttyACM0'},
        {'port_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},
        {'angle_crop_max': 225.0}
      ]
    )
    return LaunchDescription([
        serial_node,
        joyparser_node,
        lidar_node,
        camera_node,
    ])