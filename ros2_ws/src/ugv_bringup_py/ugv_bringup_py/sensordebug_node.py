# This Node calculates covariances for Imu and MagneticField messages.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from ament_index_python.packages import get_package_share_directory
import numpy as np
import json

class SensordebugNode(Node):
    def __init__(self):
        super().__init__('sensordebug_node')
        # declare parameters
        self.sub_name_imu = self.declare_parameter('sub_name_imu', 'imu/data_raw').value
        self.sub_name_mag = self.declare_parameter('sub_name_mag', 'imu/mag').value
        self.data_store_name = self.declare_parameter('data_store_name', 'covariances').value

        self.imu_acce_data_list = []
        self.imu_gyro_data_list = []
        self.mag_data_list = []
        self.count_imu = 0
        self.count_mag = 0
        self.sample_num = 1000
        self.data_save_path = "/home/nvidia/UGV_Rover/ros2_ws/src/ugv_bringup_py/config/"
        # ROS2 components
        # subs
        self.sub_imu = self.create_subscription(
            Imu,
            self.sub_name_imu,
            self.imu_callback,
            10)
        self.sub_mag = self.create_subscription(
            MagneticField,
            self.sub_name_mag,
            self.mag_callback,
            10)
        # timer
        self.timer = self.create_timer(1.0, self.timer_callback)
        # end of initialization
        self.get_logger().info("Sensordebug Node has been started.")

    def imu_callback(self, msg: Imu):
        if self.count_imu < self.sample_num:
            imu_data = [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ]
            self.imu_acce_data_list.append(imu_data[:3])
            self.imu_gyro_data_list.append(imu_data[3:])
            self.count_imu += 1

    def mag_callback(self, msg: MagneticField):
        if self.count_mag < self.sample_num:
            mag_data = [
                msg.magnetic_field.x,
                msg.magnetic_field.y,
                msg.magnetic_field.z
            ]
            self.mag_data_list.append(mag_data)
            self.count_mag += 1

    def calc_result(self):
        imu_acce_array = np.array(self.imu_acce_data_list)
        imu_gyro_array = np.array(self.imu_gyro_data_list)
        mag_array = np.array(self.mag_data_list)

        imu_acce_cov = np.cov(imu_acce_array, rowvar=False)
        imu_gyro_cov = np.cov(imu_gyro_array, rowvar=False)
        mag_cov = np.cov(mag_array, rowvar=False)

        result = {
            'imu_acce_covariance': imu_acce_cov.flatten().tolist(),
            'imu_gyro_covariance': imu_gyro_cov.flatten().tolist(),
            'magnetic_field_covariance': mag_cov.flatten().tolist()
        }

        save_path = self.data_save_path + self.data_store_name + '.json'
        with open(save_path, 'w') as f:
            json.dump(result, f)

        self.get_logger().info(f"Covariance data saved to {save_path}")

    def timer_callback(self):
        if self.count_imu >= self.sample_num and self.count_mag >= self.sample_num:
            self.calc_result()
            self.timer.cancel()  # stop the timer after calculation
            self.destroy_node()
            rclpy.shutdown()
    
def main(args=None):
    rclpy.init(args=args)
    sensordebug_node = SensordebugNode()
    rclpy.spin(sensordebug_node)