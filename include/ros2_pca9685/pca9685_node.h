#ifndef PCA9685_NODE_H
#define PCA9685_NODE_H

#include "ros2_pca9685/pca9685.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_pca9685/srv/set_pwm.hpp"

class Pca9685Node : public rclcpp::Node {
 public:
  Pca9685Node(const std::string& name);

 private:
  rclcpp::Service<ros2_pca9685::srv::SetPwm>::SharedPtr server_;
  std::unique_ptr<Pca9685> pca9685_dev_;

  void ServiceCallback(const ros2_pca9685::srv::SetPwm::Request::SharedPtr request,
                       const ros2_pca9685::srv::SetPwm::Response::SharedPtr response);
  
  double servo_freq_ {0.0};
  double pca9685_osc_ {0.0};
  double max_offset_ms_ {0.0};
  double min_offset_ms_ {0.0};
  double min_ms_ {0.0};
  double max_ms_ {0.0};

  static constexpr std::uint8_t MAX_CHANNEL_NUM                 = 15U;
  static constexpr double MAX_SERVO_ANGLE                       = 180U;
};

#endif  // PCA9685_NODE_H