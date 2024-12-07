cmake_minimum_required(VERSION 3.8)
project(lidar_camera_calibration)

add_compile_options(-std=c++17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(urdf REQUIRED)
find_package(velodyne REQUIRED)
find_package(realsense2_camera REQUIRED)
find_package(realsense2_description REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options)
find_package(Iridescence REQUIRED)
find_package(cv_bridge REQUIRED)

ament_auto_find_build_dependencies()

add_definitions(-DCV_BRIDGE_INCLUDE_H)

find_package(OpenMP)
if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_library(lidar_camera_calibration SHARED
  src/camera/create_camera.cpp
  src/calc_tools/common/frame.cpp
  src/calc_tools/common/frame_cpu.cpp
  src/calc_tools/common/ivox.cpp
  src/calc_tools/common/cloud_covariance_estimation.cpp
  src/calc_tools/common/time_keeper.cpp
  src/calc_tools/common/estimate_fov.cpp
  src/calc_tools/preprocess/preprocess.cpp
  src/calc_tools/preprocess/generate_lidar_image.cpp
  src/calc_tools/preprocess/static_point_cloud_integrator.cpp
  src/calc_tools/preprocess/pcl_densifier.cpp
  src/calc_tools/common/visual_lidar_data.cpp
)

target_include_directories(lidar_camera_calibration PUBLIC
  include
  ../dep_repos/nanoflann/include
  ../dep_repos/nlohmann-json/include
  ../dep_repos/Sophus
  ${Boost_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(lidar_camera_calibration
  fmt
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  gtsam
  gtsam_unstable
  ${OpenCV_LIBRARIES}
  Iridescence::Iridescence
)
  
install(
  DIRECTORY config data launch urdf meshes rviz include
  DESTINATION share/${PROJECT_NAME}
)

add_executable(sync_sensors src/sync_sensors.cpp)
target_include_directories(sync_sensors PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(sync_sensors rclcpp sensor_msgs message_filters)
install(TARGETS sync_sensors DESTINATION lib/${PROJECT_NAME})

add_executable(zhangs src/zhangs.cpp)
ament_target_dependencies(zhangs rclcpp sensor_msgs OpenCV)

target_include_directories(zhangs PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(zhangs ${OpenCV_LIBRARIES})

install(TARGETS
  zhangs
  DESTINATION lib/${PROJECT_NAME})

add_executable(calibration src/calibration.cpp)

target_include_directories(calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
)

target_link_libraries(calibration
  ${OpenCV_LIBS}
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
  gtsam
  gtsam_unstable
)

ament_target_dependencies(calibration
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
)
install(TARGETS calibration DESTINATION lib/${PROJECT_NAME})

add_executable(preprocess src/preprocess_ros2.cpp)
target_include_directories(preprocess PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(preprocess 
  lidar_camera_calibration
  gtsam
  gtsam_unstable
)
ament_target_dependencies(preprocess
  rclcpp
  sensor_msgs
  message_filters
  rosbag2_cpp
  cv_bridge
)
install(TARGETS preprocess DESTINATION lib/${PROJECT_NAME})

add_executable(find_correspondences src/find_correspondences.cpp)
target_include_directories(find_correspondences PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(find_correspondences 
  lidar_camera_calibration
  gtsam
  gtsam_unstable
)
ament_target_dependencies(find_correspondences rclcpp)
install(TARGETS find_correspondences DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS lidar_camera_calibration/find_matches_superglue.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(lidar_camera_calibration)
ament_export_dependencies(
  rclcpp
  Eigen3
  OpenCV
  Boost
  PCL
  Ceres
  GTSAM
  tf2_ros
  sensor_msgs
  geometry_msgs
)
ament_auto_package()
