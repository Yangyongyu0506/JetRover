from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    serial_node = Node(
        package='ugv_bringup_py',
        executable='serial_node',
        name='serial_node',
        output='screen',
        parameters=[{'do_servo_calib': False}]
    )
    joyparser_node = Node(
        package='ugv_bringup_py',
        executable='joyparser_node',
        name='joyparser_node',
        output='screen',
    )
    return LaunchDescription([
        serial_node,
        joyparser_node,
    ])