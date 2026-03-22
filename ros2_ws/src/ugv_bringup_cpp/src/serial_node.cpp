#include <chrono>
#include <cmath>
#include <functional>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "ugv_bringup_cpp/base_controller.hpp"

namespace ugv_bringup_cpp {

class SerialNode : public rclcpp::Node {
public:
  SerialNode() : Node("serial_node") {
    baudrate_ = this->declare_parameter<int>("baudrate", 115200);
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyTHS1");
    pub_name_imu_ = this->declare_parameter<std::string>("pub_name_imu", "imu/data_raw");
    pub_name_mag_ = this->declare_parameter<std::string>("pub_name_mag", "imu/mag");
    pub_name_odomraw_ = this->declare_parameter<std::string>("pub_name_odomraw", "encoder");
    pub_name_vol_ = this->declare_parameter<std::string>("pub_name_vol", "voltage");
    pub_name_joints_ = this->declare_parameter<std::string>("pub_name_joints", "joint_states");
    sub_name_vel_ = this->declare_parameter<std::string>("sub_name_vel", "cmd_vel");
    sub_name_lights_ = this->declare_parameter<std::string>("sub_name_lights", "ugv/led_strl");
    sub_name_servos_ = this->declare_parameter<std::string>("sub_name_servos", "ugv/servos");
    do_servo_calib_ = this->declare_parameter<bool>("do_servo_calib", true);
    calib_timeout_sec_ = this->declare_parameter<double>("calib_timeout_sec", 10.0);
    sample_period_ms_ = static_cast<double>(
        this->declare_parameter<int>("sample_period_ms", 50));

    if (baudrate_ <= 0) {
      baudrate_ = 115200;
    }
    if (calib_timeout_sec_ <= 0.0) {
      calib_timeout_sec_ = 10.0;
    }
    if (sample_period_ms_ <= 0.0) {
      sample_period_ms_ = 50.0;
    }

    base_ = std::make_unique<BaseController>(port_, baudrate_, this->get_logger());
    if (base_->IsOpen()) {
      base_->SendCommand(nlohmann::json{{"T", 131}, {"cmd", 1}});
      base_->SendCommand(nlohmann::json{{"T", 142}, {"cmd", sample_period_ms_}});
    }

    imu_acce_cov_.fill(0.0);
    imu_gyro_cov_.fill(0.0);
    mag_cov_.fill(0.0);
    LoadCovariances();

    if (do_servo_calib_) {
      StartServoCalib();
    }

    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(pub_name_imu_, 10);
    pub_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>(pub_name_mag_, 10);
    pub_odomraw_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(pub_name_odomraw_, 10);
    pub_vol_ = this->create_publisher<std_msgs::msg::Float32>(pub_name_vol_, 10);
    pub_joints_ = this->create_publisher<sensor_msgs::msg::JointState>(pub_name_joints_, 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(sample_period_ms_ / 2.0)),
        std::bind(&SerialNode::TimerCallback, this));

    sub_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        sub_name_vel_, 1, std::bind(&SerialNode::VelCallback, this, std::placeholders::_1));
    sub_lights_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        sub_name_lights_, 1, std::bind(&SerialNode::LightsCallback, this, std::placeholders::_1));
    sub_servos_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        sub_name_servos_, 1, std::bind(&SerialNode::ServosCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Serial Node has been started.");
  }

  ~SerialNode() {
    if (base_) {
      base_->Stop();
    }
  }

private:
  void LoadCovariances() {
    try {
      std::string cov_path = ament_index_cpp::get_package_share_directory("ugv_bringup_cpp") +
                             std::string("/config/covariances.json");
      std::ifstream cov_file(cov_path);
      if (!cov_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open covariances file: %s", cov_path.c_str());
        return;
      }
      nlohmann::json covs;
      cov_file >> covs;
      if (covs.contains("imu_acce_covariance")) {
        FillCovariance(covs["imu_acce_covariance"], imu_acce_cov_);
      }
      if (covs.contains("imu_gyro_covariance")) {
        FillCovariance(covs["imu_gyro_covariance"], imu_gyro_cov_);
      }
      if (covs.contains("magnetic_field_covariance")) {
        FillCovariance(covs["magnetic_field_covariance"], mag_cov_);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load covariances: %s", e.what());
    }
  }

  void StartServoCalib() {
    if (!base_ || !base_->IsOpen()) {
      RCLCPP_WARN(this->get_logger(), "Skipping servo calibration: serial not available.");
      return;
    }
    if (calib_pending_) {
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Starting servo calibration. Center servos now; will finalize after timeout.");
    base_->SendCommand(nlohmann::json{{"T", 210}, {"cmd", 0}});
    calib_pending_ = true;
    calib_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(calib_timeout_sec_),
        std::bind(&SerialNode::FinishServoCalib, this));
  }

  void FinishServoCalib() {
    if (!base_ || !base_->IsOpen()) {
      RCLCPP_WARN(this->get_logger(), "Skipping servo calibration finalize: serial not available.");
      return;
    }
    if (!calib_pending_) {
      return;
    }
    base_->SendCommand(nlohmann::json{{"T", 502}, {"id", 1}});
    base_->SendCommand(nlohmann::json{{"T", 502}, {"id", 2}});
    calib_pending_ = false;
    if (calib_timer_) {
      calib_timer_->cancel();
    }
    RCLCPP_INFO(this->get_logger(), "Servo calibration completed.");
  }

  void TimerCallback() {
    if (!base_ || !base_->IsOpen()) {
      return;
    }
    auto base_data_opt = base_->FeedbackData();
    if (!base_data_opt.has_value()) {
      return;
    }
    const auto &base_data = base_data_opt.value();
    if (!base_data.contains("T") || !base_data["T"].is_number_integer() ||
        base_data["T"].get<int>() != 1001) {
      return;
    }
    auto time_now = this->now();
    PublishImu(time_now, base_data);
    PublishMag(time_now, base_data);
    PublishOdomRaw(base_data);
    PublishVoltage(base_data);
    PublishJoints(time_now, base_data);
  }

  void PublishImu(const rclcpp::Time &time_now, const nlohmann::json &base_data) {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = time_now;
    imu_msg.header.frame_id = "base_imu_link";
    imu_msg.linear_acceleration.x = 9.8 * base_data.value("ax", 0.0) / 8192.0;
    imu_msg.linear_acceleration.y = 9.8 * base_data.value("ay", 0.0) / 8192.0;
    imu_msg.linear_acceleration.z = 9.8 * base_data.value("az", 0.0) / 8192.0;
    imu_msg.linear_acceleration_covariance = imu_acce_cov_;
    imu_msg.angular_velocity.x = ToRad(base_data.value("gx", 0.0) / 16.4);
    imu_msg.angular_velocity.y = ToRad(base_data.value("gy", 0.0) / 16.4);
    imu_msg.angular_velocity.z = ToRad(base_data.value("gz", 0.0) / 16.4);
    imu_msg.angular_velocity_covariance = imu_gyro_cov_;
    imu_msg.orientation_covariance[0] = -1;
    pub_imu_->publish(imu_msg);
  }

  void PublishMag(const rclcpp::Time &time_now, const nlohmann::json &base_data) {
    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header.stamp = time_now;
    mag_msg.header.frame_id = "base_imu_link";
    mag_msg.magnetic_field.x = base_data.value("mx", 0.0) * 0.15 * 1e-7;
    mag_msg.magnetic_field.y = base_data.value("my", 0.0) * 0.15 * 1e-7;
    mag_msg.magnetic_field.z = base_data.value("mz", 0.0) * 0.15 * 1e-7;
    mag_msg.magnetic_field_covariance = mag_cov_;
    pub_mag_->publish(mag_msg);
  }

  void PublishOdomRaw(const nlohmann::json &base_data) {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {
        static_cast<float>(base_data.value("odl", 0.0) / 100.0),
        static_cast<float>(base_data.value("odr", 0.0) / 100.0),
        static_cast<float>(base_data.value("L", 0.0)),
        static_cast<float>(base_data.value("R", 0.0)),
    };
    pub_odomraw_->publish(msg);
  }

  void PublishVoltage(const nlohmann::json &base_data) {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(base_data.value("v", 0.0) / 100.0);
    pub_vol_->publish(msg);
  }

  void PublishJoints(const rclcpp::Time &time_now, const nlohmann::json &base_data) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = time_now;
    msg.name = {"pt_base_link_to_pt_link1", "pt_link1_to_pt_link2"};
    msg.position = {ToRad(base_data.value("pan", 0.0)), ToRad(base_data.value("tilt", 0.0))};
    pub_joints_->publish(msg);
  }

  void VelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!base_ || !base_->IsOpen()) {
      return;
    }
    double vx = msg->linear.x;
    double wz = msg->angular.z;
    if (wz > -0.2 && wz < 0.2) {
      wz = 0.0;
    }
    base_->SendCommand(nlohmann::json{{"T", 13}, {"X", vx}, {"Z", wz}});
  }

  void LightsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() != 2) {
      RCLCPP_ERROR(this->get_logger(), "LED control message must contain exactly two elements.");
      return;
    }
    if (!base_ || !base_->IsOpen()) {
      return;
    }
    base_->SendCommand(nlohmann::json{{"T", 132}, {"IO4", msg->data[0]}, {"IO5", msg->data[1]}});
  }

  void ServosCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() != 2) {
      RCLCPP_ERROR(this->get_logger(), "Servos control message must contain exactly two elements.");
      return;
    }
    if (!base_ || !base_->IsOpen()) {
      return;
    }
    base_->SendCommand(nlohmann::json{{"T", 134}, {"X", msg->data[0]}, {"Y", msg->data[1]},
                                      {"SX", 600}, {"SY", 600}});
  }

  static constexpr double kPi = 3.14159265358979323846;
  static double ToRad(double degrees) { return degrees * kPi / 180.0; }

  static void FillCovariance(const nlohmann::json &json_array,
                             std::array<double, 9> &target) {
    if (!json_array.is_array() || json_array.size() != target.size()) {
      return;
    }
    for (std::size_t i = 0; i < target.size(); ++i) {
      if (json_array[i].is_number()) {
        target[i] = json_array[i].get<double>();
      }
    }
  }

  int baudrate_{115200};
  std::string port_;
  std::string pub_name_imu_;
  std::string pub_name_mag_;
  std::string pub_name_odomraw_;
  std::string pub_name_vol_;
  std::string pub_name_joints_;
  std::string sub_name_vel_;
  std::string sub_name_lights_;
  std::string sub_name_servos_;
  bool do_servo_calib_{true};
  double calib_timeout_sec_{10.0};
  double sample_period_ms_{50.0};

  std::unique_ptr<BaseController> base_;
  bool calib_pending_{false};
  rclcpp::TimerBase::SharedPtr calib_timer_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_odomraw_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_vol_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_lights_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_servos_;

  std::array<double, 9> imu_acce_cov_{};
  std::array<double, 9> imu_gyro_cov_{};
  std::array<double, 9> mag_cov_{};
};

}  // namespace ugv_bringup_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ugv_bringup_cpp::SerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
