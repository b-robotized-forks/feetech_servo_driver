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
    
    //  SMS_STS(u8 End, u8 Level);
    /* Constructor: with protocol end byte and response level
	 * End: Protocol end byte (0 or 1)
	 * Level: Response level (0=no response, 1=response enabled) 
    */
    SMS_STS servo_{0, 1};

    // Helper to map physical units to servo steps
    // TODO: rename to make more sense
    int map_to_servo(double physical_pos);
    double map_to_physical(int servo_pos);

    // Configuration Parameters
    std::string serial_port_;
    int baud_rate_;
    int servo_id_;
    // Calibration
    int servo_open_step_;
    int servo_closed_step_;
    double gripper_open_pos_;
    double gripper_closed_pos_;

    // TODO: add more parameters, like open/close speed? max/min speed?
};


}
#endif // feetech_servo_hardware_interface