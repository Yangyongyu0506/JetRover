# This node parses joystick inputs and transfers them into ugv commands.
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class JoyparserNode(Node):
    def __init__(self):
        super().__init__('joyparser_node')
        # declare parameters
        self.sub_name_joy = self.declare_parameter('sub_name_joy', 'joy').value
        self.pub_name_vel = self.declare_parameter('pub_name_vel', 'cmd_vel').value
        self.pub_name_lights = self.declare_parameter('pub_name_lights', 'ugv/led_strl').value
        self.pub_name_servos = self.declare_parameter('pub_name_servos', 'ugv/servos').value
        # ROS2 components
        # pubs
        self.pub_vel = self.create_publisher(Twist, self.pub_name_vel, 1)
        self.pub_lights = self.create_publisher(Float32MultiArray, self.pub_name_lights, 1)
        self.pub_servos = self.create_publisher(Float32MultiArray, self.pub_name_servos, 1)
        # subs
        self.sub_joy = self.create_subscription(
            Joy,
            self.sub_name_joy,
            self.joy_callback,
            10)
        # end of initialization
        self.get_logger().info("Joyparser Node has been started.")

    def joy_callback(self, msg: Joy):
        axes = msg.axes
        self.publish_vel(axes)
        self.publish_lights(axes)
        self.publish_servos(axes)

    def publish_vel(self, axes):
        vel_msg = Twist()
        vel_msg.linear.x = axes[3] * 1.0  # map from -1~1 to -1~1 m/s
        vel_msg.angular.z = axes[2] * 2.0  # map from -1~1 to 1~-1 rad/s
        self.pub_vel.publish(vel_msg)

    def publish_lights(self, axes):
        IO4 = (1 - axes[4]) * 255
        IO5 = (1 - axes[5]) * 255
        lights_msg = Float32MultiArray()
        lights_msg.data = [IO4, IO5]
        self.pub_lights.publish(lights_msg)

    def publish_servos(self, axes):
        servo1 = (axes[0]) * -180  # map from -1~1 to -180~180
        servo2 = (axes[1]) * 90 if axes[1] > 0 else (axes[1]) * 45   # map from -1~1 to -45~90
        servos_msg = Float32MultiArray()
        servos_msg.data = [servo1, servo2]
        self.pub_servos.publish(servos_msg)

    def destroy_node(self):
        self.pub_vel.publish(Twist())  # stop the robot on shutdown
        self.pub_lights.publish(Float32MultiArray([0., 0.]))  # turn off lights on shutdown
        self.pub_servos.publish(Float32MultiArray([0., 0.]))  # center servos on shutdown
        self.get_logger().warn("Joyparser Node has been shut down.")
        super().destroy_node()

def main():
    rclpy.init()
    joyparser_node = JoyparserNode()
    try:
        rclpy.spin(joyparser_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        joyparser_node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()