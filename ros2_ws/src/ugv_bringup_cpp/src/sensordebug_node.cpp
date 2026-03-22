#include <array>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace ugv_bringup_cpp {

class SensordebugNode : public rclcpp::Node {
public:
  SensordebugNode() : Node("sensordebug_node") {
    sub_name_imu_ = this->declare_parameter<std::string>("sub_name_imu", "imu/data_raw");
    sub_name_mag_ = this->declare_parameter<std::string>("sub_name_mag", "imu/mag");
    data_store_name_ = this->declare_parameter<std::string>("data_store_name", "covariances");
    data_save_path_ = this->declare_parameter<std::string>(
        "data_save_path", "/home/nvidia/UGV_Rover/ros2_ws/src/ugv_bringup_py/config/");
    sample_num_ = this->declare_parameter<int>("sample_num", 1000);

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        sub_name_imu_, 10, std::bind(&SensordebugNode::ImuCallback, this, std::placeholders::_1));
    sub_mag_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
        sub_name_mag_, 10, std::bind(&SensordebugNode::MagCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     std::bind(&SensordebugNode::TimerCallback, this));
    RCLCPP_INFO(this->get_logger(), "Sensordebug Node has been started.");
  }

private:
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (count_imu_ >= sample_num_) {
      return;
    }
    imu_acce_data_.push_back({msg->linear_acceleration.x, msg->linear_acceleration.y,
                              msg->linear_acceleration.z});
    imu_gyro_data_.push_back({msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z});
    count_imu_++;
  }

  void MagCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
    if (count_mag_ >= sample_num_) {
      return;
    }
    mag_data_.push_back({msg->magnetic_field.x, msg->magnetic_field.y, msg->magnetic_field.z});
    count_mag_++;
  }

  void TimerCallback() {
    if (count_imu_ >= sample_num_ && count_mag_ >= sample_num_) {
      CalcResult();
      timer_->cancel();
      rclcpp::shutdown();
    }
  }

  static std::vector<double> Covariance(const std::vector<std::array<double, 3>> &data) {
    std::vector<double> cov(9, 0.0);
    if (data.empty()) {
      return cov;
    }
    double mean[3] = {0.0, 0.0, 0.0};
    for (const auto &row : data) {
      mean[0] += row[0];
      mean[1] += row[1];
      mean[2] += row[2];
    }
    mean[0] /= data.size();
    mean[1] /= data.size();
    mean[2] /= data.size();
    for (const auto &row : data) {
      double dx = row[0] - mean[0];
      double dy = row[1] - mean[1];
      double dz = row[2] - mean[2];
      cov[0] += dx * dx;
      cov[1] += dx * dy;
      cov[2] += dx * dz;
      cov[3] += dy * dx;
      cov[4] += dy * dy;
      cov[5] += dy * dz;
      cov[6] += dz * dx;
      cov[7] += dz * dy;
      cov[8] += dz * dz;
    }
    double denom = data.size() > 1 ? static_cast<double>(data.size() - 1) : 1.0;
    for (auto &val : cov) {
      val /= denom;
    }
    return cov;
  }

  void CalcResult() {
    auto imu_acce_cov = Covariance(imu_acce_data_);
    auto imu_gyro_cov = Covariance(imu_gyro_data_);
    auto mag_cov = Covariance(mag_data_);

    nlohmann::json result;
    result["imu_acce_covariance"] = imu_acce_cov;
    result["imu_gyro_covariance"] = imu_gyro_cov;
    result["magnetic_field_covariance"] = mag_cov;

    std::string save_path = data_save_path_ + data_store_name_ + ".json";
    std::ofstream out(save_path);
    if (!out.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", save_path.c_str());
      return;
    }
    out << result.dump();
    RCLCPP_INFO(this->get_logger(), "Covariance data saved to %s", save_path.c_str());
  }

  std::string sub_name_imu_;
  std::string sub_name_mag_;
  std::string data_store_name_;
  std::string data_save_path_;
  int sample_num_{1000};

  int count_imu_{0};
  int count_mag_{0};
  std::vector<std::array<double, 3>> imu_acce_data_;
  std::vector<std::array<double, 3>> imu_gyro_data_;
  std::vector<std::array<double, 3>> mag_data_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr sub_mag_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ugv_bringup_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ugv_bringup_cpp::SensordebugNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
