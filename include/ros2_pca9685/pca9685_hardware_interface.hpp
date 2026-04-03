#pragma once

#include <vector>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "rclcpp/macros.hpp"

#include "ros2_pca9685/pca9685.h"

namespace ros2_pca9685
{

class PCA9685Hardware : public hardware_interface::SystemInterface
{
public:

    RCLCPP_SHARED_PTR_DEFINITIONS(PCA9685Hardware)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface>
        export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
        export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:

    std::unique_ptr<Pca9685> pca9685_dev_;

    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;

    double servo_freq_;
    double pca9685_osc_;
    double min_offset_ms_;
    double max_offset_ms_;
};

}