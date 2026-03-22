#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>

#include <boost/asio.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ugv_bringup_cpp {

class BaseController {
public:
  BaseController(const std::string &port, int baudrate, rclcpp::Logger logger);
  ~BaseController();

  bool IsOpen() const;
  std::optional<nlohmann::json> FeedbackData();
  void SendCommand(const nlohmann::json &data);
  void Stop();

private:
  class ReadLine {
  public:
    explicit ReadLine(boost::asio::serial_port &port);
    std::optional<std::string> ReadLineOnce();
    void ClearBuffer();

  private:
    boost::asio::serial_port &port_;
    std::string buffer_;
  };

  void ProcessCommands();

  rclcpp::Logger logger_;
  boost::asio::io_context io_;
  boost::asio::serial_port serial_;
  ReadLine readline_;
  std::atomic<bool> stop_{false};
  std::thread command_thread_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::queue<std::string> command_queue_;
  bool is_open_{false};
};

}  // namespace ugv_bringup_cpp
