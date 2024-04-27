# PCA9685 - For Robot Operating System 2 (ROS2)

## Overview

This reposotory contains the PCA9685 package dedicated for ROS2. It contains the driver and node to driver LEDs/Servo motor. \
Servo motor is used for this PCA9685 application. But modification is possible to suit your needs.

## Hardware

This package has been tested with the following hardwares: \

Raspberry Pi 4B, 4GB RAM. \
PCA9685 sensor breakout board used: https://www.nxp.com/products/power-management/lighting-driver-and-controller-ics/led-controllers/16-channel-12-bit-pwm-fm-plus-ic-bus-led-controller:PCA9685 \
The Raspberry Pi 4B runs this image: https://github.com/ros-realtime/ros-realtime-rpi4-image \
Towerpro SG90 Servo motor: https://www.towerpro.com.tw/product/sg90-7/

### Preferred Environment Setup

To run this example without issues, the following environment setup are preferred.

|                  |                          |
|------------------|--------------------------|
| Operating System | ros-realtime-rpi4-image  |
| ROS2 Version     | ROS2 Humble              |

### Package Integration

**Just clone this repository on your ros2 workspace/src.

## Starting the ros2_pca9685 node

Build the package
```bash
colcon build --packages-select ros2_pca9685
```
Dont forget to source your workspace setup.bash

and execute the following command.

```bash
ros2 launch ros2_pca9685 ros2_pca9685.launch.py
```

## Publishing to the PCA9685 service

After starting the ros2_pca9685 node,

In the host PC or in the RPi SSH client terminal, does not matter, execute the following command

```bash
ros2 service call /pca9685/set_pwm ros2_pca9685/srv/SetPwm "channel_num: <pca9685 channel number>
target_position: <target position in degrees>"
```

example (Actuate servo motor connected to channel 1 of PCA9685 to 170 degrees):
```bash
ros2 service call /pca9685/set_pwm ros2_pca9685/srv/SetPwm "channel_num: 1
target_position: 170"
```

If the above command does not work, ensure that the /pca9685/set_pwm service shows when running the following command

```bash
ros2 service list
```
