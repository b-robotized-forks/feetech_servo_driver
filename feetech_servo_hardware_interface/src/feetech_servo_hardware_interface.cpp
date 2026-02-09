#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "feetech_servo_hardware_interface/feetech_servo_hardware_interface.hpp"

namespace feetech_servo_hardware_interface
{

hardware_interface::CallbackReturn FeetechServoHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (
        hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if( !sm_st.begin(115200, "/dev/ttyUSB0") ){
        RCLCPP_FATAL(get_logger(), "Failed to init sms/sts motor!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    int id = sm_st.Ping(1);
    RCLCPP_INFO(get_logger(), "Ping ID: %d", id);
    sm_st.end();

//   if (hw_params.find("ip_address") != hw_params.end())
//   {
//     ip_address = hw_params.at("ip_address");
//   }
//   else
//   {
//     RCLCPP_FATAL(get_logger(), "Parameter 'ip_address' is required but not provided");
//     return hardware_interface::CallbackReturn::ERROR;
//   }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FeetechServoHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FeetechServoHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FeetechServoHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FeetechServoHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FeetechServoHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FeetechServoHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    return hardware_interface::return_type::OK;
}

}  // namespace feetech_servo_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    feetech_servo_hardware_interface::FeetechServoHardwareInterface, hardware_interface::SystemInterface)