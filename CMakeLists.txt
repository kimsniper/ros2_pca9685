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

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPwm.srv"
)

include_directories(include)

add_executable(ros2_pca9685_node src/pca9685.cpp src/pca9685_hal.cpp src/pca9685_node.cpp)
ament_target_dependencies(ros2_pca9685_node rclcpp)
target_link_libraries(ros2_pca9685_node i2c)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  ros2_pca9685_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ros2_pca9685_node "${cpp_typesupport_target}")

ament_package()
