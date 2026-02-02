from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    serial_node = Node(
        package='ugv_bringup_py',
        executable='serial_node',
        name='serial_node',
        output='screen',
    )
    joyparser_node = Node(
        package='ugv_bringup_py',
        executable='joyparser_node',
        name='joyparser_node',
        output='screen',
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )
    return LaunchDescription([
        serial_node,
        joyparser_node,
        joy_node,
    ])