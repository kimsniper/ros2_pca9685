# PCA9685 - ROS2 Driver with ros2_control Integration

## Overview

This repository contains a ROS2 package for controlling the PCA9685 PWM
driver using both:

-   A ROS2 native service-based interface (legacy control)
-   A ROS2 Control hardware interface (recommended for modern robot
    systems)

It is designed primarily for servo motor control.

------------------------------------------------------------------------

## Hardware

Tested setup:

-   Raspberry Pi 4B (4GB RAM)
-   OS: ros-realtime-rpi4-image
-   ROS2: Humble
-   PCA9685 16-channel PWM driver (I2C)
-   Servo: TowerPro SG90

Reference: - https://www.nxp.com/products/pca9685 -
https://www.towerpro.com.tw/product/sg90-7/

------------------------------------------------------------------------

## Package Modes

This package supports two operation modes:

### 1. Legacy ROS2 Service Mode

-   Node: `ros2_pca9685`
-   Control method: ROS2 Service (`/pca9685/set_pwm`)
-   Suitable for simple direct control or testing

### 2. ROS2 Control Hardware Interface (Recommended)

-   Integrates with `ros2_control`
-   Exposes:
    -   Command interfaces (position)
    -   State interfaces (position feedback = commanded value)
-   Works with `controller_manager` and standard ROS2 controllers

------------------------------------------------------------------------

## Installation

Clone into your ROS2 workspace:

``` bash
cd ~/ros2_ws/src
git clone <your-repo-url>
```

Build:

``` bash
cd ~/ros2_ws
colcon build --packages-select ros2_pca9685
```

Source:

``` bash
source install/setup.bash
```

------------------------------------------------------------------------

## Running Legacy Service Node

Launch driver node:

``` bash
ros2 launch ros2_pca9685 ros2_pca9685.launch.py
```

Check service:

``` bash
ros2 service list
```

Call service:

``` bash
ros2 service call /pca9685/set_pwm ros2_pca9685/srv/SetPwm "channel_num: 1
target_position: 170"
```

------------------------------------------------------------------------

## ROS2 Control Setup

### Important Concept

The hardware interface **does NOT start controller_manager
automatically**.

User is responsible for:

-   Starting `controller_manager`
-   Loading controllers (e.g. `joint_trajectory_controller`,
    `forward_command_controller`)
-   Providing URDF + ros2_control tags

------------------------------------------------------------------------

## Step 1: Add ros2_control to URDF

Example snippet:

``` xml
<ros2_control name="PCA9685System" type="system">
  <hardware>
    <plugin>ros2_pca9685/PCA9685Hardware</plugin>
  </hardware>

  <joint name="servo_joint_1">
    <command_interface name="position" />
    <state_interface name="position" />
  </joint>
</ros2_control>
```

------------------------------------------------------------------------

## Step 2: Start controller_manager

``` bash
ros2 run controller_manager ros2_control_node   --ros-args --params-file config/controllers.yaml
```

------------------------------------------------------------------------

## Step 3: Load Controllers

``` bash
ros2 control load_controller joint_state_broadcaster
ros2 control load_controller forward_command_controller
```

Activate:

``` bash
ros2 control switch_controller   --activate forward_command_controller   --activate joint_state_broadcaster
```

------------------------------------------------------------------------

## Step 4: Send Commands

``` bash
ros2 topic pub /forward_command_controller/commands std_msgs/msg/Float64MultiArray "data: [1.57, 0.78]"
```

------------------------------------------------------------------------

## Internal Behavior

-   write(): converts rad to PWM duty cycle
-   read(): echoes command (no encoder feedback)

------------------------------------------------------------------------

## Architecture

MoveIt / User Node\
|\
controller_manager\
|\
ros2_control hardware interface\
|\
PCA9685 PWM Driver\
|\
Servo Motor

------------------------------------------------------------------------

## License

BSD 3-Clause License

Copyright (c) 2026 Mezael Docoy
