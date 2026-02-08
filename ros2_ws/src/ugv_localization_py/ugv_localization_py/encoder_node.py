# This Node converts encoder data to raw odometry.
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        # declare parameters
        self.sub_name_encoder = self.declare_parameter('sub_name_encoder', 'encoder').value
        self.pub_name_odomraw = self.declare_parameter('pub_name_odomraw', 'odom/odom_raw').value
        self.pub_name_setpose = self.declare_parameter('pub_name_setpose', 'set_pose').value
        self.odom_frame_id = self.declare_parameter('odom_frame_id', 'odom').value
        self.base_frame_id = self.declare_parameter('base_frame_id', 'base_footprint').value
        self.track_width = self.declare_parameter('track_width', 0.175 * 2.256).value

        self.last_encoder_val = None
        self.pos = np.array([0., 0., 0.])  # x, y, theta
        self.pose_cov_static = [1e-9, 0., 0., 0., 0., 0.,
                                0., 1e-3, 1e-9, 0., 0., 0.,
                                0., 0., 1e6, 0., 0., 0.,
                                0., 0., 0., 1e6, 0., 0.,
                                0., 0., 0., 0., 1e6, 0.,
                                0., 0., 0., 0., 0., 1e-9]
        self.pose_cov_dynamic = [1e-3, 0., 0., 0., 0., 0.,
                                0., 1e-3, 0., 0., 0., 0.,
                                0., 0., 1e6, 0., 0., 0.,
                                0., 0., 0., 1e6, 0., 0.,
                                0., 0., 0., 0., 1e6, 0.,
                                0., 0., 0., 0., 0., 1e3]
        self.twist_cov_static = [1e-9, 0., 0., 0., 0., 0.,
                                0., 1e-3, 1e-9, 0., 0., 0.,
                                0., 0., 1e6, 0., 0., 0.,
                                0., 0., 0., 1e6, 0., 0.,
                                0., 0., 0., 0., 1e6, 0.,
                                0., 0., 0., 0., 0., 1e-9]
        self.twist_cov_dynamic = [1e-3, 0., 0., 0., 0., 0.,
                                0., 1e-3, 0., 0., 0., 0.,
                                0., 0., 1e6, 0., 0., 0.,
                                0., 0., 0., 1e6, 0., 0.,
                                0., 0., 0., 0., 1e6, 0.,
                                0., 0., 0., 0., 0., 1e3]
        # ROS2 components
        # pubs
        self.pub_odomraw = self.create_publisher(Odometry, self.pub_name_odomraw, 10)
        self.pub_setpose = self.create_publisher(PoseWithCovarianceStamped, self.pub_name_setpose, 10)
        # subs
        self.sub_encoder = self.create_subscription(
            Float32MultiArray,
            self.sub_name_encoder,
            self.encoder_callback,
            10)
        # end of initialization
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.stamp = self.get_clock().now().to_msg()
        init_pose.header.frame_id = self.odom_frame_id
        self.pub_setpose.publish(init_pose)
        self.get_logger().info("Encoder Node has been started.")

    def encoder_callback(self, msg: Float32MultiArray):
        if self.last_encoder_val is None:
            self.last_encoder_val = [msg.data[0], msg.data[1]]
            return
        odl, odr = msg.data[:2]
        vl, vr = msg.data[2:]
        dleft = odl - self.last_encoder_val[0]
        dright = odr - self.last_encoder_val[1]
        self.last_encoder_val = [odl, odr]
        drange = (dleft + dright) / 2.0
        dtheta = (dright - dleft) / self.track_width
        self.pos = self.pos + np.array([
            drange * np.cos(self.pos[2] + dtheta / 2.0),
            drange * np.sin(self.pos[2] + dtheta / 2.0),
            dtheta
        ])
        vx = (vl + vr) / 2.0
        wz = (vr - vl) / self.track_width
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        odom_msg.pose.pose.position.x = self.pos[0]
        odom_msg.pose.pose.position.y = self.pos[1]
        odom_msg.pose.pose.position.z = 0.0
        qz = np.sin(self.pos[2] / 2.0)
        qw = np.cos(self.pos[2] / 2.0)
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = wz
        if abs(vx) < 1e-3 and abs(wz) < 1e-3:
            odom_msg.pose.covariance = self.pose_cov_static
            odom_msg.twist.covariance = self.twist_cov_static
        else:
            odom_msg.pose.covariance = self.pose_cov_dynamic
            odom_msg.twist.covariance = self.twist_cov_dynamic
        self.pub_odomraw.publish(odom_msg)

def main():
    rclpy.init()
    node = EncoderNode()
    rclpy.spin(node)
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()