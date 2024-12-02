cmake_minimum_required(VERSION 3.8)
project(lidar_camera_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(urdf REQUIRED)
find_package(velodyne REQUIRED)
find_package(realsense2_camera REQUIRED)
find_package(realsense2_description REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
  
install(
  DIRECTORY config data launch urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

add_executable(sync_sensors src/sync_sensors.cpp)
ament_target_dependencies(sync_sensors rclcpp sensor_msgs message_filters)

install(TARGETS
  sync_sensors
  DESTINATION lib/${PROJECT_NAME})

add_executable(calibration src/calibration.cpp)

target_include_directories(calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

target_link_libraries(calibration
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)

ament_target_dependencies(calibration
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
)

install(TARGETS calibration
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
