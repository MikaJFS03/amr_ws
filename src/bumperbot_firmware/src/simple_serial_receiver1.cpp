// simple_serial_receiver.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <libserial/SerialPort.h>

#include <chrono>
#include <string>
#include <algorithm>

using namespace std::chrono_literals;

static inline std::string trim(const std::string &s) {
    auto front = s.find_first_not_of(" \t\r\n");
    if (front == std::string::npos) return "";
    auto back = s.find_last_not_of(" \t\r\n");
    return s.substr(front, back - front + 1);
}

class SimpleSerialReceiver : public rclcpp::Node
{
public:
    SimpleSerialReceiver() : Node("simple_serial_receiver")
    {
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud", 115200);
        this->get_parameter("port", port_);
        this->get_parameter("baud", baud_);

        try {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            RCLCPP_INFO(get_logger(), "Opened serial port: %s", port_.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Failed to open serial port %s : %s", port_.c_str(), e.what());
        }

        pub_ = create_publisher<sensor_msgs::msg::JointState>("wheel_encoders", 10);
        raw_pub_ = create_publisher<std_msgs::msg::String>("serial_receiver_raw", 5);
        timer_ = create_wall_timer(100ms, std::bind(&SimpleSerialReceiver::timerCallback, this));
        RCLCPP_INFO(get_logger(), "SimpleSerialReceiver (JointState) started");
    }

    ~SimpleSerialReceiver()
    {
        if (arduino_.IsOpen()) arduino_.Close();
    }

private:
    void timerCallback()
    {
        while (rclcpp::ok() && arduino_.IsDataAvailable()) {
            std::string raw;
            try {
                arduino_.ReadLine(raw);
            } catch (const std::exception &e) {
                RCLCPP_WARN(get_logger(), "Serial read error: %s", e.what());
                break;
            }

            // publish raw (debug)
            auto raw_msg = std_msgs::msg::String();
            raw_msg.data = raw;
            raw_pub_->publish(raw_msg);

            std::string line = trim(raw);
            if (line.empty()) continue;

            auto comma_pos = line.find(',');
            if (comma_pos == std::string::npos) {
                RCLCPP_WARN(get_logger(), "Malformed line (no comma): '%s'", line.c_str());
                continue;
            }

            std::string left_str = trim(line.substr(0, comma_pos));
            std::string right_str = trim(line.substr(comma_pos + 1));

            try {
                int left_val = std::stoi(left_str);
                int right_val = std::stoi(right_str);

                auto msg = sensor_msgs::msg::JointState();
                msg.header.stamp = this->now();
                msg.name = {"left_wheel", "right_wheel"};
                // store values as doubles in position array (you can use velocity if more appropriate)
                msg.position = {static_cast<double>(left_val), static_cast<double>(right_val)};

                pub_->publish(msg);
                RCLCPP_DEBUG(get_logger(), "Published JointState left=%d right=%d", left_val, right_val);

            } catch (const std::invalid_argument &) {
                RCLCPP_WARN(get_logger(), "Invalid integer conversion for line: '%s'", line.c_str());
            } catch (const std::out_of_range &) {
                RCLCPP_WARN(get_logger(), "Integer out of range for line: '%s'", line.c_str());
            }
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    LibSerial::SerialPort arduino_;
    std::string port_;
    int baud_{115200};
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSerialReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
