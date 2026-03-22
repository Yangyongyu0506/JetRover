#include "ugv_bringup_cpp/base_controller.hpp"

#include <array>
#include <chrono>
#include <string>

#include <cerrno>
#include <sys/select.h>
#include <unistd.h>

namespace ugv_bringup_cpp {

BaseController::ReadLine::ReadLine(boost::asio::serial_port &port) : port_(port) {}

std::optional<std::string> BaseController::ReadLine::ReadLineOnce() {
  int fd = port_.native_handle();
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(fd, &read_fds);
  timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 0;
  int ready = select(fd + 1, &read_fds, nullptr, nullptr, &timeout);
  if (ready <= 0 || !FD_ISSET(fd, &read_fds)) {
    return std::nullopt;
  }
  std::array<char, 512> temp{};
  ssize_t read_bytes = ::read(fd, temp.data(), temp.size());
  if (read_bytes <= 0) {
    if (read_bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return std::nullopt;
    }
    return std::nullopt;
  }
  buffer_.append(temp.data(), static_cast<std::size_t>(read_bytes));
  auto newline_pos = buffer_.find('\n');
  if (newline_pos != std::string::npos) {
    std::string line = buffer_.substr(0, newline_pos + 1);
    buffer_.erase(0, newline_pos + 1);
    return line;
  }
  return std::nullopt;
}

void BaseController::ReadLine::ClearBuffer() {
  boost::system::error_code ec;
  port_.cancel(ec);
  buffer_.clear();
}

BaseController::BaseController(const std::string &port, int baudrate, rclcpp::Logger logger)
    : logger_(logger), serial_(io_), readline_(serial_) {
  boost::system::error_code ec;
  serial_.open(port, ec);
  if (ec) {
    RCLCPP_ERROR(logger_, "Failed to open serial port %s: %s", port.c_str(), ec.message().c_str());
    return;
  }
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate), ec);
  if (ec) {
    RCLCPP_ERROR(logger_, "Failed to set baudrate %d: %s", baudrate, ec.message().c_str());
    serial_.close();
    return;
  }
  serial_.set_option(boost::asio::serial_port_base::character_size(8));
  serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  is_open_ = true;
  command_thread_ = std::thread(&BaseController::ProcessCommands, this);
}

BaseController::~BaseController() {
  Stop();
}

bool BaseController::IsOpen() const { return is_open_; }

std::optional<nlohmann::json> BaseController::FeedbackData() {
  if (!is_open_) {
    return std::nullopt;
  }
  auto line_opt = readline_.ReadLineOnce();
  if (!line_opt.has_value()) {
    return std::nullopt;
  }
  std::string line = line_opt.value();
  if (line.empty()) {
    return std::nullopt;
  }
  try {
    auto parsed = nlohmann::json::parse(line);
    if (!parsed.is_object()) {
      RCLCPP_ERROR(logger_, "Unexpected base data type: %s", parsed.type_name());
      return std::nullopt;
    }
    return parsed;
  } catch (const nlohmann::json::parse_error &e) {
    RCLCPP_ERROR(logger_, "JSON decode error: %s with line: %s", e.what(), line.c_str());
    readline_.ClearBuffer();
    return std::nullopt;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "[base_ctrl.feedback_data] unexpected error: %s", e.what());
    readline_.ClearBuffer();
    return std::nullopt;
  }
}

void BaseController::SendCommand(const nlohmann::json &data) {
  if (!is_open_ || stop_.load()) {
    return;
  }
  std::string payload = data.dump();
  payload.push_back('\n');
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    command_queue_.push(payload);
  }
  queue_cv_.notify_one();
}

void BaseController::ProcessCommands() {
  while (!stop_.load()) {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
      return stop_.load() || !command_queue_.empty();
    });
    if (stop_.load()) {
      break;
    }
    if (command_queue_.empty()) {
      continue;
    }
    std::string payload = command_queue_.front();
    command_queue_.pop();
    lock.unlock();

    boost::system::error_code ec;
    boost::asio::write(serial_, boost::asio::buffer(payload), ec);
    if (ec) {
      RCLCPP_ERROR(logger_, "Serial write error: %s", ec.message().c_str());
      stop_.store(true);
      break;
    }
  }
}

void BaseController::Stop() {
  stop_.store(true);
  queue_cv_.notify_all();
  if (command_thread_.joinable()) {
    command_thread_.join();
  }
  if (is_open_) {
    boost::system::error_code ec;
    serial_.close(ec);
    if (ec) {
      RCLCPP_ERROR(logger_, "Serial close error: %s", ec.message().c_str());
    }
    is_open_ = false;
  }
}

}  // namespace ugv_bringup_cpp
