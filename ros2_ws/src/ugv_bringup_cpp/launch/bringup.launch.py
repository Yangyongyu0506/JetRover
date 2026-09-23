from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PKG_DIR = get_package_share_directory('ugv_bringup_cpp')
with open(PKG_DIR + '/urdf/ugv_rover.urdf', 'r') as infp:
    robot_description = infp.read()

def generate_launch_description():
    serial_node = Node(
        package="ugv_bringup_cpp",
        executable="serial_node_cpp",
        name="serial_node",
        output="screen",
        parameters=[
            {
                "baudrate": 230400,
                "do_servo_calib": False,
                "sample_period_ms": 20,
            }
        ],
    )
    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera_node",
        output="screen",
        parameters=[{
            "frame_id": "pt_camera_link",
        }]
    )
    lidar_node = Node(
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        name="LD06",
        output="screen",
        parameters=[
            {"product_name": "LDLiDAR_LD06"},
            {"topic_name": "scan"},
            {"frame_id": "base_lidar_link"},
            {"port_name": "/dev/ttyACM0"},
            {"port_baudrate": 230400},
            {"laser_scan_dir": True},
            {"enable_angle_crop_func": False},
            {"angle_crop_min": 135.0},
            {"angle_crop_max": 225.0},
        ],
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            'robot_description': robot_description,
        }]
    )
    return LaunchDescription(
        [
            serial_node,
            lidar_node,
            robot_state_publisher_node,
            camera_node,
        ]
    )
