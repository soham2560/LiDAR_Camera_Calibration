# LiDAR Camera Calibration
ROS2 Setup to perform LiDAR-Camera Calibration for Mobile Robotics Course (CS7.503) at IIITH

- Professor: [K. Madhava Krishna](https://faculty.iiit.ac.in/~mkrishna/)

## Table of contents

- [LiDAR Camera Calibration](#lidar-camera-calibration)
  - [Table of contents](#table-of-contents)
  - [Docker Setup](#docker-setup)
  - [Hardware Setup](#hardware-setup)

## Docker Setup
- To pull latest docker image
    ```bash
    docker pull ghcr.io/soham2560/humble-garden:latest
    ```
- To start container
    - Open Command Pallete with `Ctrl+Shift+P`
    - Select Option to Rebuild and Reopen Container
    - You can use `Import Libs` button once container has opened to import dependent libs
    - Use `Build WS` button to build workspace
  
  Note: To access these buttons you may need to enable [VSCode Action Button Extension](https://marketplace.visualstudio.com/items?itemName=seunlanlege.action-buttons) through the Extensions Tab in VSCode, the extension should download automatically on container startup

## How to Use
- Import, Build and Source the Workspace
- Ensure Hardware is setup properly and powered on
- Use following command to launch
  ```bash
  ros2 launch robot_bringup robot.launch.py use_rviz:=True
  ```

## Hardware Setup
The repository has currently been setup with
- 3D LiDAR: Velodyne VLP16
  - LiDAR IP: 172.16.101.71
  - Host IP: 172.16.101.77

Note: The README's in this repository are inspired by [this](https://github.com/TheProjectsGuy/MR21-CS7.503) and [this](https://github.com/ankitdhall/lidar_camera_calibration)