cmake_minimum_required(VERSION 3.8)
project(ros2_pca9685)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPwm.srv"
)

include_directories(include)

add_executable(
  ros2_pca9685_node
  src/pca9685.cpp
  src/pca9685_hal.cpp
  src/pca9685_node.cpp
)

ament_target_dependencies(
  ros2_pca9685_node
  rclcpp
)

target_link_libraries(
  ros2_pca9685_node
  i2c
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  ros2_pca9685_node
  "${cpp_typesupport_target}"
)

add_library(
  pca9685_hardware_interface
  SHARED
  src/pca9685.cpp
  src/pca9685_hal.cpp
  src/pca9685_hardware_interface.cpp
)

ament_target_dependencies(
  pca9685_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
)

target_link_libraries(
  pca9685_hardware_interface
  i2c
)

pluginlib_export_plugin_description_file(
  hardware_interface
  plugin.xml
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  TARGETS
  ros2_pca9685_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
  pca9685_hardware_interface
  DESTINATION lib
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
