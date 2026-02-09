#ifndef FEETECH_SERVO_HARDWARE_INTERFACE_HPP_
#define FEETECH_SERVO_HARDWARE_INTERFACE_HPP_
#include <memory>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
#include "SCServo.h"

namespace feetech_servo_hardware_interface
{
class FeetechServoHardwareInterface : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params) override;
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
private:
    rclcpp::Logger getLogger() { return rclcpp::get_logger("FeetechServoHardwareInterface"); }
    SMS_STS sm_st;
};


}
#endif // feetech_servo_hardware_interface