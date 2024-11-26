#include "edabot_firmware/edabot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace edabot_firmware
{

EdabotInterface::EdabotInterface()
{
}

EdabotInterface::~EdabotInterface()
{
    if (stm_.IsOpen())
    {
        try
        {
            stm_.Close();
        }
        catch (...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("EdabotInterface"), "Something went wrong while closing connection with port " << port_);
        }
    }
}


CallbackReturn EdabotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    try
    {
        port_ = info_.hardware_parameters.at("port");
    }
    catch (const std::out_of_range &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("EdabotInterface"), "No Serial Port provided! Aborting");
        return CallbackReturn::FAILURE;
    }

    velocity_commands_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
    velocity_states_.reserve(info_.joints.size());
    last_run_ = rclcpp::Clock().now();

    return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> EdabotInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Provide only a position interface
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
    }

    return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> EdabotInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Provide only a velocity interface
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
    }

    return command_interfaces;
}


CallbackReturn EdabotInterface::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("EdabotInterface"), "Starting robot hardware ...");

    // Reset commands and states
    velocity_commands_  = {0.0, 0.0};
    position_states_    = {0.0, 0.0};
    velocity_states_    = {0.0, 0.0};

    try
    {
        stm_.Open(port_);
        stm_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch (...)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("EdabotInterface"), "Something went wrong while interacting with port " << port_);
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("EdabotInterface"),
    "Hardware started, ready to take commands");
    return CallbackReturn::SUCCESS;
}


CallbackReturn EdabotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("EdabotInterface"), "Stopping robot hardware ...");

    if (stm_.IsOpen())
    {
        try
        {
            stm_.Close();
        }
        catch (...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("EdabotInterface"), "Something went wrong while closing connection with port " << port_);
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("EdabotInterface"), "Hardware stopped");
    return CallbackReturn::SUCCESS;
}


hardware_interface::return_type EdabotInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    return hardware_interface::return_type::OK;
}


hardware_interface::return_type EdabotInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    // Implement communication protocol with the Arduino
    std::stringstream message_stream;
    double multiplier = 5.0;

    char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
    char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';
    std::string compensate_zeros_right = "";
    std::string compensate_zeros_left = "";

    if(std::abs(velocity_commands_.at(0)) < 10.0)
    {
        compensate_zeros_right = "0";
    }
    else
    {
        compensate_zeros_right = "";
    }
    if(std::abs(velocity_commands_.at(1)) < 10.0)
    {
        compensate_zeros_left = "0";
    }
    else
    {
        compensate_zeros_left = "";
    }

    // kode lama
    message_stream << int(int(velocity_commands_.at(0)) * multiplier) << "," << int(int(velocity_commands_.at(1)) * multiplier) << "\n";

    // message_stream << std::fixed << std::setprecision(2)
    //                << "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(0) * multiplier)
    //                << ",l" <<  left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands_.at(1) * multiplier) << ",\n";

    try
    {
        // // for STM32
        // stm_.Write(message_stream.str());
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("EdabotInterface"), "Try to sending the message: " << message_stream.str() << " to the port " << port_);

        // for Arduino Mega
        if(velocity_commands_.at(0) != last_vel[0]
        && velocity_commands_.at(1) != last_vel[1])
        {
            stm_.Write(message_stream.str());
            RCLCPP_INFO_STREAM(rclcpp::get_logger("EdabotInterface"), "Try to sending the message: " << message_stream.str() << " to the port " << port_);
        }
        
        last_vel[0] = velocity_commands_.at(0);
        last_vel[1] = velocity_commands_.at(1);
    }
    catch (...)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("EdabotInterface"), "Something went wrong while sending the message " << message_stream.str() << " to the port " << port_);
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

}  // namespace edabot_firmware

PLUGINLIB_EXPORT_CLASS(edabot_firmware::EdabotInterface, hardware_interface::SystemInterface)