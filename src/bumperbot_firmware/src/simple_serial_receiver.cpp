// simple_serial_receiver.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int64.hpp>
#include <libserial/SerialPort.h>

#include <chrono>
#include <string>
#include <sstream>
#include <vector>

using namespace std::chrono_literals;

static inline std::string trim(const std::string &s) {
  auto front = s.find_first_not_of(" \t\r\n");
  if (front == std::string::npos) return "";
  auto back = s.find_last_not_of(" \t\r\n");
  return s.substr(front, back - front + 1);
}

static inline void split_csv_first_two(const std::string &line, std::string &f0, std::string &f1) {
  size_t p = line.find(',');
  if (p == std::string::npos) { f0.clear(); f1.clear(); return; }
  f0 = trim(line.substr(0, p));
  size_t p2 = line.find(',', p + 1);
  if (p2 == std::string::npos) {
    f1 = trim(line.substr(p + 1));
  } else {
    f1 = trim(line.substr(p + 1, p2 - (p + 1)));
  }
}

class SimpleSerialReceiver : public rclcpp::Node {
public:
  SimpleSerialReceiver() : Node("simple_serial_receiver") {
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud", 115200);
    this->get_parameter("port", port_);
    this->get_parameter("baud", baud_);

    try {
      arduino_.Open(port_);
      // If you want to honor the "baud" param, map to LibSerial enum.
      arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
      RCLCPP_INFO(get_logger(), "Opened serial port: %s", port_.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port %s : %s", port_.c_str(), e.what());
      // Node still runs; will keep trying to read each timer tick
    }

    left_pub_  = create_publisher<std_msgs::msg::Int64>("left_encoder", 10);
    right_pub_ = create_publisher<std_msgs::msg::Int64>("right_encoder", 10);
    raw_pub_   = create_publisher<std_msgs::msg::String>("serial_receiver_raw", 5);

    timer_ = create_wall_timer(100ms, std::bind(&SimpleSerialReceiver::timerCallback, this));
    RCLCPP_INFO(get_logger(), "SimpleSerialReceiver started (publishing Int64 left/right encoders)");
  }

  ~SimpleSerialReceiver() override {
    if (arduino_.IsOpen()) {
      arduino_.Close();
    }
  }

private:
  void timerCallback() {
    if (!arduino_.IsOpen()) {
      // Try to re-open if disconnected
      try {
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000, "Reopened %s", port_.c_str());
      } catch (...) {
        return; // wait for next tick
      }
    }

    while (rclcpp::ok() && arduino_.IsDataAvailable()) {
      std::string raw;
      try {
        arduino_.ReadLine(raw);  // reads up to '\n'
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Serial read error: %s", e.what());
        break;
      }

      auto msg_raw = std_msgs::msg::String();
      msg_raw.data = raw;
      raw_pub_->publish(msg_raw);

      std::string line = trim(raw);
      if (line.empty()) continue;

      // Ignore comment/header lines if firmware prints CSV header starting with '#'
      if (!line.empty() && line[0] == '#') continue;

      // Parse first two comma-separated fields as left/right ticks
      std::string f0, f1;
      split_csv_first_two(line, f0, f1);
      if (f0.empty() || f1.empty()) {
        RCLCPP_WARN(get_logger(), "Malformed line (need at least two CSV fields): '%s'", line.c_str());
        continue;
      }

      try {
        // Use stoll to support large counts
        long long left_val  = std::stoll(f0);
        long long right_val = std::stoll(f1);

        std_msgs::msg::Int64 left_msg;
        std_msgs::msg::Int64 right_msg;
        left_msg.data  = left_val;
        right_msg.data = right_val;

        left_pub_->publish(left_msg);
        right_pub_->publish(right_msg);
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Conversion error for line: '%s' (%s)", line.c_str(), e.what());
      }
    }
  }

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr left_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr right_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  LibSerial::SerialPort arduino_;
  std::string port_;
  int baud_{115200};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSerialReceiver>());
  rclcpp::shutdown();
  return 0;
}
