cmake_minimum_required(VERSION 3.5)
project(sync_sensors)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_executable(sync_sensors src/sync_sensors.cpp)
ament_target_dependencies(sync_sensors rclcpp sensor_msgs message_filters)

install(TARGETS
  sync_sensors
  DESTINATION lib/${PROJECT_NAME})

ament_package()
