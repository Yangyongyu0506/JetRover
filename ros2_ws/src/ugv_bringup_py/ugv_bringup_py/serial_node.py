# This node interacts with the base via UART.
import json

from numpy import deg2rad
import serial

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import ok
from sensor_msgs.msg import Imu, JointState, MagneticField
from std_msgs.msg import Float32, Float32MultiArray

from .utils import BaseController


class SerialNode(Node):
    def __init__(self):
        super().__init__("serial_node")
        # declare parameters
        self.baudrate = int(self.declare_parameter("baudrate", 115200).value or 115200)
        if not self.baudrate:
            self.baudrate = 115200
        self.port = str(self.declare_parameter("port", "/dev/ttyTHS1").value)

        self.pub_name_imu = str(
            self.declare_parameter("pub_name_imu", "imu/data_raw").value
        )
        self.pub_name_mag = str(self.declare_parameter("pub_name_mag", "imu/mag").value)
        self.pub_name_odomraw = str(
            self.declare_parameter("pub_name_odomraw", "encoder").value
        )
        self.pub_name_vol = str(self.declare_parameter("pub_name_vol", "voltage").value)
        self.pub_name_joints = str(
            self.declare_parameter("pub_name_joints", "joint_states").value
        )

        self.sub_name_vel = str(self.declare_parameter("sub_name_vel", "cmd_vel").value)
        self.sub_name_lights = str(
            self.declare_parameter("sub_name_lights", "ugv/led_strl").value
        )
        self.sub_name_servos = str(
            self.declare_parameter("sub_name_servos", "ugv/servos").value
        )

        self.do_servo_calib = bool(self.declare_parameter("do_servo_calib", True).value)
        self.calib_timeout_sec = float(
            self.declare_parameter("calib_timeout_sec", 10.0).value or 10.0
        )
        if self.calib_timeout_sec <= 0:
            self.calib_timeout_sec = 10.0
        sample_period_ms = float(
            self.declare_parameter("sample_period_ms", 50).value or 50.0
        )
        if sample_period_ms <= 0:
            sample_period_ms = 50.0
        self.base = None
        self.calib_pending = False
        # serial setup
        try:
            self.base = BaseController(self.port, self.baudrate, self.get_logger())
            self.base.send_command({"T": 131, "cmd": 1})  # request base info
            # set ESP32 sample rate
            self.base.send_command({"T": 142, "cmd": sample_period_ms})
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
        # sensor data
        self.imu_acce_cov = [0.0] * 9
        self.imu_gyro_cov = [0.0] * 9
        self.mag_cov = [0.0] * 9
        try:
            cov_path = (
                get_package_share_directory("ugv_bringup_py")
                + "/config/covariances.json"
            )
            with open(cov_path, "r", encoding="utf-8") as cov_file:
                covs = json.load(cov_file)
            self.imu_acce_cov = covs.get("imu_acce_covariance", self.imu_acce_cov)
            self.imu_gyro_cov = covs.get("imu_gyro_covariance", self.imu_gyro_cov)
            self.mag_cov = covs.get("magnetic_field_covariance", self.mag_cov)
        except (OSError, json.JSONDecodeError) as e:
            self.get_logger().error(f"Failed to load covariances: {e}")
        # perform servo calibration if required
        if self.do_servo_calib:
            self.start_servo_calib()
        # ROS2 components
        # pubs
        self.pub_imu = self.create_publisher(Imu, self.pub_name_imu, 10)
        self.pub_mag = self.create_publisher(MagneticField, self.pub_name_mag, 10)
        self.pub_odomraw = self.create_publisher(
            Float32MultiArray, self.pub_name_odomraw, 10
        )
        self.pub_vol = self.create_publisher(Float32, self.pub_name_vol, 10)
        self.pub_joints = self.create_publisher(JointState, self.pub_name_joints, 10)
        self.timer = self.create_timer(
            sample_period_ms / 2000.0, self.timer_callback
        )  # set timer period to half of sample period to ensure no data loss
        # subs
        self.sub_vel = self.create_subscription(
            Twist, self.sub_name_vel, self.vel_callback, 1
        )  # TO ensure real time control, set QOS profile to 1
        self.sub_lights = self.create_subscription(
            Float32MultiArray, self.sub_name_lights, self.lights_callback, 1
        )
        self.sub_servos = self.create_subscription(
            Float32MultiArray, self.sub_name_servos, self.servos_callback, 1
        )
        # end of initialization
        self.get_logger().info("Serial Node has been started.")

    def start_servo_calib(self):
        if not self.base:
            self.get_logger().warn("Skipping servo calibration: serial not available.")
            return
        if self.calib_pending:
            return
        self.get_logger().info(
            "Starting servo calibration. Center servos now; will finalize after timeout."
        )
        self.base.send_command({"T": 210, "cmd": 0})  # shutdown servos' torque lock
        self.calib_pending = True
        self.calib_timer = self.create_timer(
            self.calib_timeout_sec, self.finish_servo_calib
        )

    def finish_servo_calib(self):
        if not self.base:
            self.get_logger().warn(
                "Skipping servo calibration finalize: serial not available."
            )
            return
        if not self.calib_pending:
            return
        self.base.send_command({"T": 502, "id": 1})
        self.base.send_command(
            {"T": 502, "id": 2}
        )  # set current position as zero position
        self.calib_pending = False
        self.calib_timer.cancel()
        self.get_logger().info("Servo calibration completed.")

    def timer_callback(self):
        if not self.base:
            return
        base_data = self.base.feedback_data()
        if not base_data:
            return
        if base_data.get("T") == 1001:
            time_now = self.get_clock().now().to_msg()
            self.publish_imu(time_now, base_data)
            self.publish_mag(time_now, base_data)
            self.publish_odomraw(time_now, base_data)
            self.publish_voltage(time_now, base_data)
            self.publish_joints(time_now, base_data)

    def publish_imu(self, time_now, base_data):
        imu_msg = Imu()
        imu_msg.header.stamp = time_now
        imu_msg.header.frame_id = "base_imu_link"

        imu_msg.linear_acceleration.x = 9.8 * float(base_data.get("ax", 0.0)) / 8192
        imu_msg.linear_acceleration.y = 9.8 * float(base_data.get("ay", 0.0)) / 8192
        imu_msg.linear_acceleration.z = 9.8 * float(base_data.get("az", 0.0)) / 8192
        imu_msg.linear_acceleration_covariance = self.imu_acce_cov
        imu_msg.angular_velocity.x = deg2rad(float(base_data.get("gx", 0.0)) / 16.4)
        imu_msg.angular_velocity.y = deg2rad(float(base_data.get("gy", 0.0)) / 16.4)
        imu_msg.angular_velocity.z = deg2rad(float(base_data.get("gz", 0.0)) / 16.4)
        imu_msg.angular_velocity_covariance = self.imu_gyro_cov
        imu_msg.orientation_covariance[0] = -1  # orientation and its covariance not provided

        self.pub_imu.publish(imu_msg)

    def publish_mag(self, time_now, base_data):
        mag_msg = MagneticField()
        mag_msg.header.stamp = time_now
        mag_msg.header.frame_id = "base_imu_link"

        mag_msg.magnetic_field.x = float(base_data.get("mx", 0.0)) * 0.15 * 1e-7
        mag_msg.magnetic_field.y = float(base_data.get("my", 0.0)) * 0.15 * 1e-7
        mag_msg.magnetic_field.z = float(base_data.get("mz", 0.0)) * 0.15 * 1e-7
        mag_msg.magnetic_field_covariance = self.mag_cov

        self.pub_mag.publish(mag_msg)

    def publish_odomraw(self, _, base_data):
        odomraw_msg = Float32MultiArray()
        odomraw_msg.data = [
            float(base_data.get("odl", 0.0)) / 100,
            float(base_data.get("odr", 0.0)) / 100,
            float(base_data.get("L", 0.0)),
            float(base_data.get("R", 0.0)),
        ]
        self.pub_odomraw.publish(odomraw_msg)

    def publish_voltage(self, _, base_data):
        vol_msg = Float32()
        vol_msg.data = float(base_data.get("v", 0.0)) / 100
        self.pub_vol.publish(vol_msg)

    def publish_joints(self, time_now, base_data):
        joints_msg = JointState()
        joints_msg.header.stamp = time_now
        joints_msg.name = ["pt_base_link_to_pt_link1", "pt_link1_to_pt_link2"]
        joints_msg.position = [
            deg2rad(base_data.get("pan", 0.0)),
            deg2rad(base_data.get("tilt", 0.0)),
        ]
        self.pub_joints.publish(joints_msg)

    def vel_callback(self, msg):
        if not self.base:
            return
        vx = msg.linear.x  # m/s
        wz = msg.angular.z  # rad/s
        if -0.2 < wz < 0.2:
            wz = 0.0
        vel_data = {"T": 13, "X": vx, "Z": wz}
        self.base.send_command(vel_data)

    def lights_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().error(
                "LED control message must contain exactly two elements."
            )
            return
        IO4 = msg.data[0]
        IO5 = msg.data[1]
        led_ctrl_data = {
            "T": 132,
            "IO4": IO4,
            "IO5": IO5,
        }
        if self.base:
            self.base.send_command(led_ctrl_data)

    def servos_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().error(
                "Servos control message must contain exactly two elements."
            )
            return
        yaw_deg = msg.data[0]
        pitch_deg = msg.data[1]
        servo_ctrl_data = {
            "T": 134,
            "X": yaw_deg,
            "Y": pitch_deg,
            "SX": 600,
            "SY": 600,
        }
        if self.base:
            self.base.send_command(servo_ctrl_data)

    def destroy_node(self):
        if self.base:
            self.vel_callback(Twist())  # send zero velocity command
            self.lights_callback(Float32MultiArray(data=[0.0, 0.0]))  # turn off lights
            self.servos_callback(Float32MultiArray(data=[0.0, 0.0]))  # reset servos
            self.base.destroy_base()
        self.get_logger().info("Serial Node has been shut down.")
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException) as e:
        node.get_logger().warn(f"Serial node interrupted: {e}")
    finally:
        node.destroy_node()
        if ok():
            rclpy.shutdown()
