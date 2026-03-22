import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        # declare parameters
        self.pub_name_img = str(
            self.declare_parameter("pub_name_img", "camera/image_raw").value
        )
        self.timer_period = float(
            self.declare_parameter("timer_period", 0.1).value or 0.1
        )
        self.frame_id = str(self.declare_parameter("frame_id", "pt_camera_link").value)
        self.camera_id = self.declare_parameter("camera_id", 0).value
        self.img_resolution = self.declare_parameter("img_resolution", [640, 480]).value
        self.gst_pipeline = str(self.declare_parameter("gst_pipeline", "").value)

        self.cap = None
        self.last_open_error = None
        self.open_camera()
        # ROS2 components
        self.bridge = CvBridge()
        self.pub_img = self.create_publisher(Image, self.pub_name_img, 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def open_camera(self):
        if self.cap is not None:
            return
        cap = None
        if self.gst_pipeline:
            cap = cv2.VideoCapture(self.gst_pipeline, cv2.CAP_GSTREAMER)
        else:
            cap_source = self.camera_id
            if isinstance(cap_source, str) and cap_source.startswith("/dev/video"):
                cap = cv2.VideoCapture(cap_source, cv2.CAP_V4L2)
            else:
                try:
                    cap = cv2.VideoCapture(int(cap_source or 0), cv2.CAP_V4L2)
                except (TypeError, ValueError):
                    cap = cv2.VideoCapture(cap_source)

        if not cap or not cap.isOpened():
            error_msg = f"Failed to open camera source: {self.camera_id}"
            if error_msg != self.last_open_error:
                self.get_logger().error(error_msg)
                self.last_open_error = error_msg
            return

        if (
            isinstance(self.img_resolution, (list, tuple))
            and len(self.img_resolution) == 2
        ):
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.img_resolution[0]))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.img_resolution[1]))

        self.cap = cap
        self.last_open_error = None
        self.get_logger().info(f"Camera opened: {self.camera_id}")

    def timer_callback(self):
        if self.cap is None:
            self.open_camera()
            return
        ret, frame = self.cap.read()
        if not ret:
            return
        frame = np.ascontiguousarray(frame)
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = self.frame_id
        self.pub_img.publish(img_msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()
