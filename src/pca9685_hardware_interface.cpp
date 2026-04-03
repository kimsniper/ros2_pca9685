#include "ros2_pca9685/pca9685_hardware_interface.hpp"

#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace ros2_pca9685
{

hardware_interface::CallbackReturn
PCA9685Hardware::on_init(const hardware_interface::HardwareInfo & info)
{
    if (SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    size_t num_joints = info_.joints.size();

    hw_commands_.resize(num_joints, 0.0);
    hw_states_.resize(num_joints, 0.0);

    servo_freq_ = 50;
    pca9685_osc_ = 25000000;
    min_offset_ms_ = 0.5;
    max_offset_ms_ = 2.5;

    pca9685_dev_ = std::make_unique<Pca9685>(
        "/dev/i2c-1",
        0x40,
        0x00
    );

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PCA9685Hardware::on_activate(const rclcpp_lifecycle::State &)
{
    pca9685_dev_->Pca9685_SetPrescale(
        servo_freq_,
        pca9685_osc_);

    pca9685_dev_->Pca9685_PwrMode(
        Pca9685::PCA9685_MODE_NORMAL);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PCA9685Hardware::on_deactivate(const rclcpp_lifecycle::State &)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> PCA9685Hardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;

    for (size_t i = 0; i < hw_commands_.size(); i++)
    {
        interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_commands_[i]);
    }

    return interfaces;
}

std::vector<hardware_interface::StateInterface> PCA9685Hardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;

    for (size_t i = 0; i < hw_states_.size(); i++)
    {
        interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_states_[i]);
    }

    return interfaces;
}

hardware_interface::return_type PCA9685Hardware::read(
    const rclcpp::Time &,
    const rclcpp::Duration &)
{
    hw_states_ = hw_commands_;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type PCA9685Hardware::write(
    const rclcpp::Time &,
    const rclcpp::Duration &)
{
    for (size_t i = 0; i < hw_commands_.size(); i++)
    {
        double angle_deg =
            hw_commands_[i] * 180.0 / M_PI;

        double pulse_ms =
            ((angle_deg / 180.0) *
            (2.0 + max_offset_ms_ - (1.0 + min_offset_ms_)))
            + (1.0 + min_offset_ms_);

        double period_ms =
            1000.0 / servo_freq_;

        double duty =
            (pulse_ms / period_ms) * 100.0;

        pca9685_dev_->Pca9685_LedPwmSet(i, duty);
    }

    return hardware_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(
    ros2_pca9685::PCA9685Hardware,
    hardware_interface::SystemInterface
)