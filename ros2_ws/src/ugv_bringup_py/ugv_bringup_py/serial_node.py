# This node interacts with the base via UART.
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from .utils import BaseController
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Imu, MagneticField, JointState
from geometry_msgs.msg import Twist
from numpy import deg2rad, rad2deg

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        # declare parameters
        self.baudrate = self.declare_parameter('baudrate', 115200).value
        self.port = self.declare_parameter('port', '/dev/ttyTHS1').value

        self.pub_name_imu = self.declare_parameter('pub_name_imu', 'imu/data_raw').value
        self.pub_name_mag = self.declare_parameter('pub_name_mag', 'imu/mag').value
        self.pub_name_odomraw = self.declare_parameter('pub_name_odomraw', 'odom/odom_raw').value
        self.pub_name_vol = self.declare_parameter('pub_name_vol', 'voltage').value
        self.pub_name_joints = self.declare_parameter('pub_name_joints', 'joint_states').value

        self.sub_name_vel = self.declare_parameter('sub_name_vel', 'cmd_vel').value
        self.sub_name_lights = self.declare_parameter('sub_name_lights', 'ugv/led_strl').value
        self.sub_name_servos = self.declare_parameter('sub_name_servos', 'ugv/servos').value
        # serial setup
        self.base = BaseController(self.port, self.baudrate, self.get_logger())
        self.base.send_command({'T': 131, 'cmd': 1})  # request base info
        # ROS2 components
        # pubs
        self.pub_imu = self.create_publisher(Imu, self.pub_name_imu, 10)
        self.pub_mag = self.create_publisher(MagneticField, self.pub_name_mag, 10)
        self.pub_odomraw = self.create_publisher(Float32MultiArray, self.pub_name_odomraw, 10)
        self.pub_vol = self.create_publisher(Float32, self.pub_name_vol, 10)
        self.pub_joints = self.create_publisher(JointState, self.pub_name_joints, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        # subs
        self.sub_vel = self.create_subscription(
            Twist,
            self.sub_name_vel,
            self.vel_callback,
            1) # TO ensure real time control, set QOS profile to 1
        self.sub_lights = self.create_subscription(
            Float32MultiArray,
            self.sub_name_lights,
            self.lights_callback,
            1)
        self.sub_servos = self.create_subscription(
            Float32MultiArray,
            self.sub_name_servos,
            self.servos_callback,
            1)
        # end of initialization
        self.get_logger().info("Serial Node has been started.")

    def timer_callback(self):
        self.base.feedback_data()
        if self.base.base_data["T"] == 1001:
            time_now = self.get_clock().now().to_msg()
            self.publish_imu(time_now, self.base.base_data)
            self.publish_mag(time_now, self.base.base_data)
            self.publish_odomraw(time_now, self.base.base_data)
            self.publish_voltage(time_now, self.base.base_data)
            self.publish_joints(time_now, self.base.base_data)

    def publish_imu(self, time_now, base_data):
        imu_msg = Imu()
        imu_msg.header.stamp = time_now
        imu_msg.header.frame_id = 'base_imu_link'

        imu_msg.linear_acceleration.x = 9.8 * float(base_data["ax"]) / 8192
        imu_msg.linear_acceleration.y = 9.8 * float(base_data["ay"]) / 8192
        imu_msg.linear_acceleration.z = 9.8 * float(base_data["az"]) / 8192
        imu_msg.angular_velocity.x = deg2rad(float(base_data["gx"]) / 16.4)
        imu_msg.angular_velocity.y = deg2rad(float(base_data["gy"]) / 16.4)
        imu_msg.angular_velocity.z = deg2rad(float(base_data["gz"]) / 16.4)

        # We need covariances here

        self.pub_imu.publish(imu_msg)

    def publish_mag(self, time_now, base_data):
        mag_msg = MagneticField()
        mag_msg.header.stamp = time_now
        mag_msg.header.frame_id = 'base_imu_link'

        mag_msg.magnetic_field.x = float(base_data["mx"]) * 0.15
        mag_msg.magnetic_field.y = float(base_data["my"]) * 0.15
        mag_msg.magnetic_field.z = float(base_data["mz"]) * 0.15

        # We need covariances here

        self.pub_mag.publish(mag_msg)

    def publish_odomraw(self, time_now, base_data):
        odomraw_msg = Float32MultiArray()
        odomraw_msg.data = [float(base_data["odl"]) / 100, float(base_data["odr"]) / 100]
        self.pub_odomraw.publish(odomraw_msg)

    def publish_voltage(self, time_now, base_data):
        vol_msg = Float32()
        vol_msg.data = float(base_data["v"]) / 100
        self.pub_vol.publish(vol_msg)

    def publish_joints(self, time_now, base_data):
        joints_msg = JointState()
        joints_msg.header.stamp = time_now
        joints_msg.name = ['pt_base_link_to_pt_link1', 'pt_link1_to_pt_link2']
        joints_msg.position = [deg2rad(base_data['pan']), deg2rad(base_data['tilt'])]
        self.pub_joints.publish(joints_msg)

    def vel_callback(self, msg):
        vx = msg.linear.x  # m/s
        wz = msg.angular.z  # rad/s
        if -0.2 < wz < 0.2:
            wz = 0.0
        vel_data = {'T': 13, 'X': vx, 'Z': wz}
        self.base.send_command(vel_data)

    def lights_callback(self, msg):
        assert len(msg.data) == 2, "LED control message must contain exactly two elements."
        IO4 = msg.data[0]
        IO5 = msg.data[1]
        led_ctrl_data = {
            'T': 132, 
            "IO4": IO4,
            "IO5": IO5,
        }
        self.base.send_command(led_ctrl_data)

    def servos_callback(self, msg):
        assert len(msg.data) == 2, "Servos control message must contain exactly two elements."
        yaw_deg = msg.data[0]
        pitch_deg = msg.data[1]
        servo_ctrl_data = {
            'T': 134,
            'X': yaw_deg,
            'Y': pitch_deg,
            'SX': 600,
            'SY': 600,
        }
        self.base.send_command(servo_ctrl_data)

    def destroy_node(self):
        self.vel_callback(Twist())  # send zero velocity command
        self.lights_callback(Float32MultiArray(data=[0., 0.]))  # turn off lights
        self.servos_callback(Float32MultiArray(data=[0., 0.]))  # reset servos
        self.base.destroy_base()
        self.get_logger().info("Serial Node has been shut down.")
        super().destroy_node()

def main():
    rclpy.init()
    serial_node = SerialNode()
    try:
        rclpy.spin(serial_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        serial_node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()