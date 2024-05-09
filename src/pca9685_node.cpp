#include "ros2_pca9685/pca9685_node.h"

using std::placeholders::_1;
using std::placeholders::_2;

Pca9685Node::Pca9685Node(const std::string& name)
    : Node(name)
    , pca9685_dev_{std::make_unique<Pca9685>()}
{
    /* Declare parameters */
    this->declare_parameter<double>("servo_freq", 50.0);
    this->declare_parameter<double>("pca9685_osc", 25000000.0);
    this->declare_parameter<double>("max_offset_ms", 0.0);
    this->declare_parameter<double>("min_offset_ms", 0.0);

    max_offset_ms_ = this->get_parameter("max_offset_ms").as_double();
    min_offset_ms_ = this->get_parameter("min_offset_ms").as_double();

    min_ms_ = 1 + min_offset_ms_;
    max_ms_ = 2 + max_offset_ms_;

    servo_freq_ = this->get_parameter("servo_freq").as_double();
    pca9685_osc_ = this->get_parameter("pca9685_osc").as_double();

    pca9685_dev_->Pca9685_SetPrescale(servo_freq_, pca9685_osc_);
    pca9685_dev_->Pca9685_PwrMode(Pca9685::PCA9685_MODE_NORMAL);

    server_ = this->create_service<ros2_pca9685::srv::SetPwm>("/pca9685/set_pwm",
                                                              std::bind(&Pca9685Node::ServiceCallback,
                                                                        this,
                                                                        _1,
                                                                        _2));

}

void Pca9685Node::ServiceCallback(const ros2_pca9685::srv::SetPwm::Request::SharedPtr request,
                                  const ros2_pca9685::srv::SetPwm::Response::SharedPtr response)
{
    if(request->channel_num > MAX_CHANNEL_NUM)
    {
        response->is_success = false;
        response->response = "Invalid channel number";
        RCLCPP_INFO_STREAM(get_logger(), "Invalid channel number" << std::endl);
        return;
    }
    if(request->target_position > MAX_SERVO_ANGLE)
    {
        response->is_success = false;
        response->response = "Invalid angle range";
        RCLCPP_INFO_STREAM(get_logger(), "Invalid angle range" << std::endl);
        return;
    }

    /* Log details */
    RCLCPP_INFO_STREAM(get_logger(), "Actuate servo at channel number " << static_cast<int>(request->channel_num) << " to " << static_cast<double>(request->target_position) << " degrees angle position" << std::endl);

    /* Calculate duty cycle based on degree value */
    double duty_cycle {0.0};

    duty_cycle = ((request->target_position / 180.0) * (max_ms_ - min_ms_)) + min_ms_;  /* Converting 0 to 180 deg to 1ms to 2 ms */
    duty_cycle = (duty_cycle/(1000.0 / servo_freq_)) * 100.0; /* Converting to duty cycle */

    pca9685_dev_->Pca9685_LedPwmSet(request->channel_num, duty_cycle);

    response->is_success = true;
    response->response = "Servo actuation completed";
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Pca9685Node>("pca9685_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}