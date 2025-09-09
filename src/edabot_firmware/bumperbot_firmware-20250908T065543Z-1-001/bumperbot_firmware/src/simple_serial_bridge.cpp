// simple_serial_bridge.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int64.hpp>
#include <libserial/SerialPort.h>
#include <chrono>
#include <string>
#include <mutex>

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
  auto t0 = trim(line.substr(0, p));
  size_t p2 = line.find(',', p + 1);
  auto t1 = (p2 == std::string::npos) ? trim(line.substr(p + 1))
                                      : trim(line.substr(p + 1, p2 - (p + 1)));
  f0 = std::move(t0); f1 = std::move(t1);
}

class SimpleSerialBridge : public rclcpp::Node {
public:
  SimpleSerialBridge()
  : Node("simple_serial_bridge")
  {
    // Parameters
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud", 115200);
    this->declare_parameter<int>("read_period_ms", 100);     // how often to poll for reads
    this->declare_parameter<bool>("append_newline_on_write", true); // add '\n' to writes

    this->get_parameter("port", port_);
    this->get_parameter("baud", baud_);
    int read_period_ms = 100;
    this->get_parameter("read_period_ms", read_period_ms);
    this->get_parameter("append_newline_on_write", append_newline_);

    open_port_or_warn_();

    // Publishers
    left_pub_ = create_publisher<std_msgs::msg::Int64>("left_encoder", 10);
    right_pub_ = create_publisher<std_msgs::msg::Int64>("right_encoder", 10);
    raw_pub_  = create_publisher<std_msgs::msg::String>("serial_receiver_raw", 5);

    // RX timer (polls for available data and emits messages)
    timer_ = create_wall_timer(
      std::chrono::milliseconds(read_period_ms),
      std::bind(&SimpleSerialBridge::rx_timer_cb_, this));

    // TX subscriber (writes raw strings to the serial port)
    sub_ = create_subscription<std_msgs::msg::String>(
      "serial_transmitter", 10,
      std::bind(&SimpleSerialBridge::tx_cb_, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "SimpleSerialBridge started (port=%s, baud=%d, read_period_ms=%d)",
                port_.c_str(), baud_, read_period_ms);
  }

  ~SimpleSerialBridge() override {
    if (serial_.IsOpen()) {
      serial_.Close();
    }
  }

private:
  // --- Serial open ---
  void open_port_or_warn_() {
    try {
      serial_.Open(port_);
      // Map common baud rates
      LibSerial::BaudRate br = LibSerial::BaudRate::BAUD_115200;
      switch (baud_) {
        case 9600: br  = LibSerial::BaudRate::BAUD_9600;  break;
        case 19200: br = LibSerial::BaudRate::BAUD_19200; break;
        case 38400: br = LibSerial::BaudRate::BAUD_38400; break;
        case 57600: br = LibSerial::BaudRate::BAUD_57600; break;
        case 115200: default: br = LibSerial::BaudRate::BAUD_115200; break;
      }
      serial_.SetBaudRate(br);
      // Optional: 8N1, no flow control are usually defaults
      RCLCPP_INFO(get_logger(), "Opened serial port: %s @ %d", port_.c_str(), baud_);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s: %s", port_.c_str(), e.what());
    }
  }

  // --- TX path ---
  void tx_cb_(const std_msgs::msg::String &msg) {
    if (!serial_.IsOpen()) {
      open_port_or_warn_();
      if (!serial_.IsOpen()) return;
    }
    try {
      std::lock_guard<std::mutex> lk(tx_mx_); // conservative guard in case of multithreaded exec
      if (append_newline_) {
        serial_.Write(msg.data + "\n");
      } else {
        serial_.Write(msg.data);
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Serial write error: %s", e.what());
    }
  }

  // --- RX path ---
  void rx_timer_cb_() {
    if (!serial_.IsOpen()) {
      open_port_or_warn_();
      if (!serial_.IsOpen()) return;
    }

    // Drain all available lines this tick
    while (rclcpp::ok() && serial_.IsDataAvailable()) {
      std::string raw;
      try {
        serial_.ReadLine(raw);   // blocks up to newline, but IsDataAvailable() keeps it snappy
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Serial read error: %s", e.what());
        break;
      }

      // Publish raw line
      std_msgs::msg::String raw_msg;
      raw_msg.data = raw;
      raw_pub_->publish(raw_msg);

      // Parse "<left>,<right>[,...]" CSV
      const std::string line = trim(raw);
      if (line.empty() || line[0] == '#') continue;

      std::string f0, f1;
      split_csv_first_two(line, f0, f1);
      if (f0.empty() || f1.empty()) {
        RCLCPP_WARN(get_logger(), "Malformed CSV (need at least two fields): '%s'", line.c_str());
        continue;
      }

      try {
        long long left  = std::stoll(f0);
        long long right = std::stoll(f1);
        std_msgs::msg::Int64 lmsg; lmsg.data = left;
        std_msgs::msg::Int64 rmsg; rmsg.data = right;
        left_pub_->publish(lmsg);
        right_pub_->publish(rmsg);
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Conversion error for line: '%s' (%s)", line.c_str(), e.what());
      }
    }
  }

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr left_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr right_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Serial state
  LibSerial::SerialPort serial_;
  std::string port_;
  int baud_{115200};
  bool append_newline_{true};
  std::mutex tx_mx_; // only needed if using MultiThreadedExecutor
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  // SingleThreadedExecutor avoids concurrent callbacks; safe with LibSerial
  rclcpp::spin(std::make_shared<SimpleSerialBridge>());
  rclcpp::shutdown();
  return 0;
}
