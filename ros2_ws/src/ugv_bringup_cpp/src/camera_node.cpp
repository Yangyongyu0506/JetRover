#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace ugv_bringup_cpp {

class CameraNode : public rclcpp::Node {
public:
  CameraNode() : Node("camera_node") {
    pub_name_img_ = this->declare_parameter<std::string>("pub_name_img", "camera/image_raw");
    timer_period_sec_ = this->declare_parameter<double>("timer_period", 0.1);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "pt_camera_link");
    camera_id_ = this->declare_parameter<int>("camera_id", 0);
    camera_device_ = this->declare_parameter<std::string>("camera_device", "");
    img_resolution_ = this->declare_parameter<std::vector<int64_t>>("img_resolution",
                                                                     std::vector<int64_t>{640, 480});
    gst_pipeline_ = this->declare_parameter<std::string>("gst_pipeline", "");

    if (timer_period_sec_ <= 0.0) {
      timer_period_sec_ = 0.1;
    }

    pub_img_ = this->create_publisher<sensor_msgs::msg::Image>(pub_name_img_, 10);
    OpenCamera();

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_period_sec_),
        std::bind(&CameraNode::TimerCallback, this));
  }

  ~CameraNode() {
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

private:
  void OpenCamera() {
    if (cap_.isOpened()) {
      return;
    }
    if (!gst_pipeline_.empty()) {
      cap_.open(gst_pipeline_, cv::CAP_GSTREAMER);
    } else if (!camera_device_.empty()) {
      cap_.open(camera_device_, cv::CAP_V4L2);
    } else {
      cap_.open(camera_id_, cv::CAP_V4L2);
    }

    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera source (id=%d device=%s)",
                   camera_id_, camera_device_.c_str());
      return;
    }

    if (img_resolution_.size() == 2) {
      cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(img_resolution_[0]));
      cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(img_resolution_[1]));
    }
    cap_.set(cv::CAP_PROP_CONVERT_RGB, 1);
    RCLCPP_INFO(this->get_logger(), "Camera opened.");
  }

  void TimerCallback() {
    if (!cap_.isOpened()) {
      OpenCamera();
      return;
    }
    try {
      cv::Mat frame;
      if (!cap_.read(frame)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Failed to read frame from camera.");
        return;
      }
      if (frame.empty() || frame.rows <= 0 || frame.cols <= 0 || frame.data == nullptr) {
        return;
      }
      if (frame.channels() == 1) {
        cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
      } else if (frame.channels() == 4) {
        cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
      } else if (frame.channels() != 3) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Unexpected frame channels: %d", frame.channels());
        return;
      }
      if (!frame.isContinuous()) {
        frame = frame.clone();
      }
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      msg->header.stamp = this->now();
      msg->header.frame_id = frame_id_;
      pub_img_->publish(*msg);
    } catch (const cv::Exception &e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "OpenCV error: %s", e.what());
      if (cap_.isOpened()) {
        cap_.release();
      }
    }
  }

  std::string pub_name_img_;
  double timer_period_sec_{0.1};
  std::string frame_id_;
  int camera_id_{0};
  std::string camera_device_;
  std::vector<int64_t> img_resolution_;
  std::string gst_pipeline_;

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ugv_bringup_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ugv_bringup_cpp::CameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
