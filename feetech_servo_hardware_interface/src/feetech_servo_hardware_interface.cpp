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

    // TODO: add these params to readme with examples
    try {
        serial_port_ = info_.hardware_parameters.at("serial_port");
        baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
        servo_id_ = std::stoi(info_.hardware_parameters.at("servo_id"));

        servo_open_step_ = std::stoi(info_.hardware_parameters.at("servo_open_step"));
        servo_closed_step_ = std::stoi(info_.hardware_parameters.at("servo_closed_step"));
        gripper_open_pos_ = std::stod(info_.hardware_parameters.at("gripper_open_pos"));
        gripper_closed_pos_ = std::stod(info_.hardware_parameters.at("gripper_closed_pos"));

    } catch (const std::exception & e) {
        RCLCPP_FATAL(getLogger(), "Exception parsing parameters: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.joints.size() != 1)
    {
        RCLCPP_FATAL(get_logger(), "Hardware info has %zu joints. 1 joint is expected.", info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FeetechServoHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Configuring Feetech Gripper Servo on %s at %d baud", serial_port_.c_str(), baud_rate_);

    if (!servo_.begin(baud_rate_, serial_port_.c_str()))
    {
        RCLCPP_FATAL(rclcpp::get_logger("FeetechGripperHardware"), "Failed to open serial port %s", serial_port_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }


    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FeetechServoHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    servo_.end();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FeetechServoHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Activating Gripper ID %d...", servo_id_);

    if (servo_.Ping(servo_id_) == -1)
    {
        RCLCPP_ERROR(getLogger(), "Servo Ping Failed! ID: %d", servo_id_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Mode 0 = SMS_STS_MODE_SERVO (Position Control)
    if (servo_.InitMotor(servo_id_, SMS_STS_MODE_SERVO, 1) == 0)
    {
        RCLCPP_ERROR(getLogger(), "Failed to Enable Torque!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize command to current position to prevent jumps
    int pos_steps_initial = servo_.ReadPos(servo_id_);
    if (pos_steps_initial != -1)
    {        
        double pos_meters_initial = servo_steps_2_meters(pos_steps_initial);
        set_state(info_.joints[0].name + "/" + "position", pos_meters_initial);
        set_command(info_.joints[0].name + "/" + "position", pos_meters_initial);

        RCLCPP_INFO(getLogger(), "Initialized at servo pos: %d (%.4f m)", pos_steps_initial, pos_meters_initial);
    }
    else
    {
        RCLCPP_ERROR(getLogger(), "Failed to read initial position");
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FeetechServoHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Disabling gripper servo torque...");
    servo_.EnableTorque(servo_id_, 0); // 0 = torque off

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FeetechServoHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
     // 1. Read Position
    int pos = servo_.ReadPos(servo_id_);
    if (pos != -1) {
        set_state(info_.joints[0].name + "/" + "position", servo_steps_2_meters(pos));
    } else {
        RCLCPP_ERROR(getLogger(), "Failed to read servo POSITION");
        return hardware_interface::return_type::ERROR;
    }

    int speed = servo_.ReadSpeed(servo_id_);
    if (speed != -1) {
        // unit: steps/s -> m/s
        double m_per_step = (gripper_open_pos_ - gripper_closed_pos_) / 
                            (double)(servo_open_step_ - servo_closed_step_);
        set_state(info_.joints[0].name + "/" + "velocity", speed * m_per_step);
    } else {
        RCLCPP_ERROR(getLogger(), "Failed to read servo VELOCITY");
        return hardware_interface::return_type::ERROR;
    }

    int torque = servo_.ReadLoad(servo_id_); // -1000 to 1000, raw units for now
    if (torque != -1) {
        set_state(info_.joints[0].name + "/" + "effort", static_cast<double>(torque));
    } else {
        RCLCPP_ERROR(getLogger(), "Failed to read servo EFFORT");
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FeetechServoHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    double target_pos_meters = get_command(info_.joints[0].name + "/" + "position");
    
    if (std::isnan(target_pos_meters)) {
        return hardware_interface::return_type::OK;
    }

    bool ok = servo_.WritePosEx(servo_id_, meters_2_servo_steps(target_pos_meters), servo_speed_, servo_acceleration_);
    if (!ok) {
        RCLCPP_ERROR(getLogger(), "Failed to write position!");
        return hardware_interface::return_type::DEACTIVATE;
    }
    return hardware_interface::return_type::OK;
}

int FeetechServoHardwareInterface::meters_2_servo_steps(double pos_meters)
{
  double p_min = std::min(gripper_closed_pos_, gripper_open_pos_);
  double p_max = std::max(gripper_closed_pos_, gripper_open_pos_);
  double clamped = std::clamp(pos_meters, p_min, p_max);

  // lerp, "how far along is the command from 0% to 100% open"
  double ratio = (clamped - gripper_closed_pos_) / (gripper_open_pos_ - gripper_closed_pos_);
  
  double steps = servo_closed_step_ + ratio * (servo_open_step_ - servo_closed_step_);
  
  return static_cast<int>(steps);
}

double FeetechServoHardwareInterface::servo_steps_2_meters(int servo_pos)
{
  double ratio = (double)(servo_pos - servo_closed_step_) / (double)(servo_open_step_ - servo_closed_step_);
  return gripper_closed_pos_ + ratio * (gripper_open_pos_ - gripper_closed_pos_);
}

}  // namespace feetech_servo_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    feetech_servo_hardware_interface::FeetechServoHardwareInterface, hardware_interface::SystemInterface)