#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace ugv_bringup_cpp {

class JoyparserNode : public rclcpp::Node {
public:
  JoyparserNode() : Node("joyparser_node") {
    sub_name_joy_ = this->declare_parameter<std::string>("sub_name_joy", "joy");
    sub_name_vol_ = this->declare_parameter<std::string>("sub_name_vol", "voltage");
    pub_name_vel_ = this->declare_parameter<std::string>("pub_name_vel", "cmd_vel");
    pub_name_lights_ = this->declare_parameter<std::string>("pub_name_lights", "ugv/led_strl");
    pub_name_servos_ = this->declare_parameter<std::string>("pub_name_servos", "ugv/servos");
    pub_name_joyfeedback_ = this->declare_parameter<std::string>("pub_name_joyfeedback", "joy/set_feedback");
    use_timeout_ = this->declare_parameter<bool>("use_timeout", true);
    vol_threshold_ = this->declare_parameter<double>("vol_threshold", 10.5);
    max_v_ = this->declare_parameter<double>("max_linear_vel", 1.0);
    max_w_ = this->declare_parameter<double>("max_angular_vel", 2.0);

    pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(pub_name_vel_, 1);
    pub_lights_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(pub_name_lights_, 1);
    pub_servos_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(pub_name_servos_, 1);
    pub_joyfeedback_ = this->create_publisher<sensor_msgs::msg::JoyFeedback>(pub_name_joyfeedback_, 1);

    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
        sub_name_joy_, 10, std::bind(&JoyparserNode::JoyCallback, this, std::placeholders::_1));
    sub_vol_ = this->create_subscription<std_msgs::msg::Float32>(
        sub_name_vol_, 10, std::bind(&JoyparserNode::VolCallback, this, std::placeholders::_1));

    last_joy_time_ = this->now();
    if (use_timeout_) {
      timer_ = this->create_wall_timer(std::chrono::seconds(5),
                                       std::bind(&JoyparserNode::TimerJoyCallback, this));
    }
    RCLCPP_INFO(this->get_logger(), "Joyparser Node has been started.");
  }

private:
  void TimerJoyCallback() {
    double time_diff = (this->now() - last_joy_time_).seconds();
    if (time_diff > 5.0) {
      pub_vel_->publish(geometry_msgs::msg::Twist());
      std_msgs::msg::Float32MultiArray lights_msg;
      lights_msg.data = {0.0f, 0.0f};
      pub_lights_->publish(lights_msg);
      std_msgs::msg::Float32MultiArray servos_msg;
      servos_msg.data = {0.0f, 0.0f};
      pub_servos_->publish(servos_msg);
      RCLCPP_WARN(this->get_logger(),
                  "No joystick input received for %.2f seconds. Stopping the robot.",
                  time_diff);
    }
  }

  void VolCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    double voltage = msg->data;
    if (voltage < vol_threshold_) {
      RCLCPP_FATAL(this->get_logger(), "Low voltage detected: %.2fV", voltage);
    }
  }

  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    last_joy_time_ = this->now();
    const auto &axes = msg->axes;
    const auto &buttons = msg->buttons;
    if (buttons.size() > 8 && buttons[8]) {
      servo_bias_ = {0.0, 0.0};
    }
    if (axes.size() >= 5) {
      PublishVel({axes[3], axes[4]});
    }
    if (axes.size() >= 6) {
      PublishLights({axes[2], axes[5]});
    }
    if (axes.size() >= 8) {
      PublishServos({axes[0], axes[1], axes[6], axes[7]});
    }
  }

  void PublishVel(const std::array<float, 2> &axes_slice) {
    geometry_msgs::msg::Twist vel_msg;
    vel_msg.linear.x = axes_slice[1] * max_v_;
    vel_msg.angular.z = axes_slice[0] * max_w_;
    pub_vel_->publish(vel_msg);
  }

  void PublishLights(const std::array<float, 2> &axes_slice) {
    float io5 = (1.0f - axes_slice[0]) * 255.0f;
    float io4 = (1.0f - axes_slice[1]) * 255.0f;
    std_msgs::msg::Float32MultiArray lights_msg;
    lights_msg.data = {io4, io5};
    pub_lights_->publish(lights_msg);
  }

  void PublishServos(const std::array<float, 4> &axes_slice) {
    servo_bias_[0] += -axes_slice[2];
    servo_bias_[1] += axes_slice[3];
    servo_bias_[0] = std::clamp(servo_bias_[0], -180.0, 180.0);
    servo_bias_[1] = std::clamp(servo_bias_[1], -45.0, 90.0);
    double servo1 = axes_slice[0] * -180.0;
    double servo2 = axes_slice[1] > 0 ? axes_slice[1] * 90.0 : axes_slice[1] * 45.0;
    std_msgs::msg::Float32MultiArray servos_msg;
    servos_msg.data = {
        static_cast<float>(std::clamp(servo1 + servo_bias_[0], -180.0, 180.0)),
        static_cast<float>(std::clamp(servo2 + servo_bias_[1], -45.0, 90.0)),
    };
    pub_servos_->publish(servos_msg);
  }

  std::string sub_name_joy_;
  std::string sub_name_vol_;
  std::string pub_name_vel_;
  std::string pub_name_lights_;
  std::string pub_name_servos_;
  std::string pub_name_joyfeedback_;
  bool use_timeout_{true};
  double vol_threshold_{10.5};
  double max_v_{1.0};
  double max_w_{2.0};
  rclcpp::Time last_joy_time_;
  std::array<double, 2> servo_bias_{0.0, 0.0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_lights_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_servos_;
  rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr pub_joyfeedback_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_vol_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ugv_bringup_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ugv_bringup_cpp::JoyparserNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
