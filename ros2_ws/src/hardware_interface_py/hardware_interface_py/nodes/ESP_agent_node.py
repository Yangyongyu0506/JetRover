# Non-Lifecycle version of ESP_agent.py
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from hardware_interface_py.drivers.base import BaseController
from ament_index_python.packages import get_package_share_directory
import os
import math
import threading
from sensor_msgs.msg import Imu, MagneticField, JointState, BatteryState
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import Empty

class ESPAgentNode(Node):
    def __init__(self):
        super().__init__('esp_agent_node')
        # Node initialization code here
        # params
        self.declare_parameter('uart_interface', '/dev/ttyTHS1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('config_path', 'base_config.yaml')
        self.declare_parameter('base_pub_freq', 10) # in Hz
        # Base controller
        curpath = get_package_share_directory('hardware_interface_py')
        config_path = os.path.join(
            curpath, 
            'config', 
            self.get_parameter('config_path').get_parameter_value().string_value
        )
        self.base = BaseController(
            self.get_parameter('uart_interface').get_parameter_value().string_value,
            self.get_parameter('baud_rate').get_parameter_value().integer_value,
            config_path
        )
        self.base.activate_ser()
        # Create callback groups
        self.mtxcbgroup = MutuallyExclusiveCallbackGroup()
        self.retcbgroup = ReentrantCallbackGroup()
        # Initialize the publishers
        self.pub_imu = self.create_publisher(
            Imu, 
            'imu/data_raw', 
            10
        )
        self.pub_mag = self.create_publisher(
            MagneticField, 
            'imu/mag', 
            10
        )
        self.pub_bat = self.create_publisher(
            BatteryState, 
            'battery_state', 
            10
        )
        # Initialize the timer
        self.timer = self.create_timer(
            1 / self.get_parameter('base_pub_freq').get_parameter_value().integer_value,
            self.timer_callback,
            callback_group=self.retcbgroup
        )
        # Initialize the subscribers
        self.sub_vel = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10,
            callback_group=self.mtxcbgroup
        )
        self.sub_joints = self.create_subscription(
            JointState,
            'pan_tilt',
            self.joint_callback,
            10,
            callback_group=self.mtxcbgroup
        )
        self.sub_led = self.create_subscription(
            UInt8MultiArray,
            'led_ctrl',
            self.led_callback,
            10,
            callback_group=self.mtxcbgroup
        )
        # Initialize the services
        self.srv_ledoff = self.create_service(
            Empty,
            'led_off',
            self.ledoff_callback,
            callback_group=self.mtxcbgroup
        )
        self.srv_ptmiddle = self.create_service(
            Empty,
            'pt_set_mid',
            self.ptsetmid_callback,
            callback_group=self.mtxcbgroup
        )
        self.get_logger().info("ESP Agent Node has been initialized.")

    def destroy_node(self):
        # Cleanup timer
        self.destroy_timer(self.timer)
        # Cleanup publishers
        for pub in [self.pub_imu, self.pub_mag, self.pub_bat]:
            self.destroy_publisher(pub)
        # Cleanup subscribers
        for sub in [self.sub_vel, self.sub_joints, self.sub_led]:
            self.destroy_subscription(sub)
        # Cleanup services
        for srv in [self.srv_ledoff, self.srv_ptmiddle]:
            self.destroy_service(srv)
        # Cleanup base controller
        self.base.deactivate_ser()
        self.base.cleanup()
        self.base.command_queue.queue.clear()
        return super().destroy_node()

    def publish_imu_data_raw(self, timestamp):
        msg = Imu()
        msg.header.stamp = timestamp
        msg.header.frame_id = "imu_link"
        # acceleration in m/s^2, gyro in rad/s
        msg.linear_acceleration.x = self.base.base_data['ax'] * 9.8 / 8192
        msg.linear_acceleration.y = self.base.base_data['ay'] * 9.8 / 8192
        msg.linear_acceleration.z = self.base.base_data['az'] * 9.8 / 8192
        msg.angular_velocity.x = math.radians(self.base.base_data['gx'] / 16.4)
        msg.angular_velocity.y = math.radians(self.base.base_data['gy'] / 16.4)
        msg.angular_velocity.z = math.radians(self.base.base_data['gz'] / 16.4)
        # covariances can be set here if available
        self.pub_imu.publish(msg)

    def publish_imu_mag(self, timestamp):
        msg = MagneticField()
        msg.header.stamp = timestamp
        msg.header.frame_id = "imu_link"
        # magnetic field in Tesla
        msg.magnetic_field.x = self.base.base_data['mx'] * 0.15
        msg.magnetic_field.y = self.base.base_data['my'] * 0.15
        msg.magnetic_field.z = self.base.base_data['mz'] * 0.15
        # covariances can be set here if available
        self.pub_mag.publish(msg)

    def publish_batstate(self, timestamp):
        msg = BatteryState()
        msg.header.stamp = timestamp
        msg.header.frame_id = "base_link"
        # voltage in Volts
        msg.voltage = self.base.base_data['v'] / 100.0
        # current, charge, capacity, design_capacity, percentage can be set if available
        self.pub_bat.publish(msg)

    def timer_callback(self):
        with self.base.lock:
            if self.base.base_data is None:
                return
            timestamp = self.get_clock().now().to_msg()
            self.publish_imu_data_raw(timestamp)
            self.publish_imu_mag(timestamp)
            self.publish_batstate(timestamp)

    def cmd_vel_callback(self, msg: Twist):
        with self.base.lock:
            v = msg.linear.x
            omega = msg.angular.z
            # Apply minimum threshold to angular velocity if linear velocity is zero
            if v == 0:
                if 0 < omega < 0.2:
                    omega = 0.2
                elif -0.2 < omega < 0:
                    omega = -0.2
            self.base.send_command({
                'T': 13, 
                'X': v, 
                'Z': omega
            })

    def joint_callback(self, msg: JointState):
        with self.base.lock:
            name = msg.name
            position = msg.position
            try:
                x_rad = position[name.index('pt_yaw')]
                y_rad = position[name.index('pt_pitch')]
            except ValueError as e:
                self.get_logger().error(f"Joint names not found in the message: {e}")
                return
            x_degree = math.degrees(x_rad)
            y_degree = math.degrees(y_rad)
            self.base.send_command({
                'T': 134,
                'X': x_degree,
                'Y': y_degree,
                "SX": 600,
                "SY": 600,
            })

    def led_callback(self, msg: UInt8MultiArray):
        with self.base.lock:
            self.base.send_command({
                'T': 132,
                'IO4': msg.data[0],
                'IO5': msg.data[1],
            })

    def ledoff_callback(self, request, response):
        with self.base.lock:
            self.base.send_command({
                'T': 132,
                'IO4': 0,
                'IO5': 0,
            })
            return response

    def ptsetmid_callback(self, request, response):
        with self.base.lock:
            self.base.send_command({
                'T': 141,
                'X': 2,
                'Y': 2,
                'SPD': 0
            })
            return response

def main(args=None):
    rclpy.init(args=args)
    esp_agent_node = ESPAgentNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(esp_agent_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        executor.remove_node(esp_agent_node)
        esp_agent_node.destroy_node()
        rclpy.shutdown()