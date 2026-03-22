# This node parses joystick inputs and transfers them into ugv commands.
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Joy, JoyFeedback
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
import numpy as np

class JoyparserNode(Node):
    def __init__(self):
        super().__init__('joyparser_node')
        # declare parameters
        self.sub_name_joy = self.declare_parameter('sub_name_joy', 'joy').value
        self.sub_name_vol = self.declare_parameter('sub_name_vol', 'voltage').value
        self.pub_name_vel = self.declare_parameter('pub_name_vel', 'cmd_vel').value
        self.pub_name_lights = self.declare_parameter('pub_name_lights', 'ugv/led_strl').value
        self.pub_name_servos = self.declare_parameter('pub_name_servos', 'ugv/servos').value
        self.pub_name_joyfeedback = self.declare_parameter('pub_name_joyfeedback', 'joy/set_feedback').value

        self.use_timeout = self.declare_parameter('use_timeout', True).value
        self.vol_threshold = self.declare_parameter('vol_threshold', 10.5).value
        self.max_v = self.declare_parameter('max_linear_vel', 1.0).value
        self.max_w = self.declare_parameter('max_angular_vel', 2.0).value

        self.servo_bias = np.array([0., 0.])
        self.last_joy_time = self.get_clock().now()
        # ROS2 components
        # pubs
        self.pub_vel = self.create_publisher(Twist, self.pub_name_vel, 1)
        self.pub_lights = self.create_publisher(Float32MultiArray, self.pub_name_lights, 1)
        self.pub_servos = self.create_publisher(Float32MultiArray, self.pub_name_servos, 1)
        self.pub_joyfeedback = self.create_publisher(JoyFeedback, self.pub_name_joyfeedback, 1)
        # subs
        self.sub_joy = self.create_subscription(
            Joy,
            self.sub_name_joy,
            self.joy_callback,
            10)
        self.sub_vol = self.create_subscription(
            Float32,
            self.sub_name_vol,
            self.vol_callback,
            10)
        # timers
        if self.use_timeout:
            self.timer_joy = self.create_timer(5.0, self.timer_joy_callback)
        # end of initialization
        self.get_logger().info("Joyparser Node has been started.")

    def timer_joy_callback(self):
        time_diff = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if time_diff > 5.0:
            self.pub_vel.publish(Twist())  # stop the robot
            self.pub_lights.publish(Float32MultiArray(data=[0., 0.]))  # turn off lights
            self.pub_servos.publish(Float32MultiArray(data=[0., 0.]))  # center servos
            self.get_logger().warn(f"No joystick input received for {time_diff} seconds. Stopping the robot.")

    def vol_callback(self, msg: Float32):
        voltage = msg.data
        if voltage < self.vol_threshold:
            # feedback_msg = JoyFeedback()
            # feedback_msg.type = JoyFeedback.TYPE_RUMBLE
            # feedback_msg.id = 1
            # feedback_msg.intensity = 1.0
            # self.pub_joyfeedback.publish(feedback_msg)
            self.get_logger().fatal(f"Low voltage detected: {voltage}V")

    def joy_callback(self, msg: Joy):
        self.last_joy_time = self.get_clock().now()
        axes = msg.axes
        buttons = msg.buttons
        if buttons[8]:
            self.servo_bias = np.array([0., 0.])  # reset servo bias
        self.publish_vel(axes[3:5])
        self.publish_lights(axes[2:6:3])  # only use axes[2] and axes[5] for lights control
        self.publish_servos(axes[0:2] + axes[6:8])

    def publish_vel(self, axes_slice):
        vel_msg = Twist()
        vel_msg.linear.x = axes_slice[1] * self.max_v  # map from -1~1 to -max_v~max_v m/s
        vel_msg.angular.z = axes_slice[0] * self.max_w  # map from -1~1 to -max_w~max_w rad/s
        self.pub_vel.publish(vel_msg)

    def publish_lights(self, axes_slice):
        IO5 = (1 - axes_slice[0]) * 255
        IO4 = (1 - axes_slice[1]) * 255
        lights_msg = Float32MultiArray()
        lights_msg.data = [IO4, IO5]
        self.pub_lights.publish(lights_msg)

    def publish_servos(self, axes_slice):
        self.servo_bias += np.array([-axes_slice[2], axes_slice[3]])  # adjust bias with buttons
        self.servo_bias = np.clip(self.servo_bias, [-180., -45.], [180., 90.])  # limit bias
        servo1 = (axes_slice[0]) * -180  # map from -1~1 to -180~180
        servo2 = (axes_slice[1]) * 90 if axes_slice[1] > 0 else (axes_slice[1]) * 45   # map from -1~1 to -45~90
        servos_msg = Float32MultiArray()
        servos_msg.data = np.clip(np.array([servo1 + self.servo_bias[0], servo2 + self.servo_bias[1]]), [-180., -45.], [180., 90.]).tolist()
        self.pub_servos.publish(servos_msg)

    def on_shutdown(self):
        self.pub_vel.publish(Twist())  # stop the robot on shutdown
        self.pub_lights.publish(Float32MultiArray(data=[0., 0.]))  # turn off lights on shutdown
        self.pub_servos.publish(Float32MultiArray(data=[0., 0.]))  # center servos on shutdown
        self.get_logger().warn("Joyparser Node has been shut down.")

def main():
    rclpy.init()
    node = JoyparserNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        node.on_shutdown()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()