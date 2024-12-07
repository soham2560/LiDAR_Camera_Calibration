# LiDAR Camera Calibration
ROS2 Setup to perform LiDAR-Camera Calibration for Mobile Robotics Course (CS7.503) at IIITH

- Professor: [K. Madhava Krishna](https://faculty.iiit.ac.in/~mkrishna/)

## Table of contents

- [LiDAR Camera Calibration](#lidar-camera-calibration)
  - [Table of contents](#table-of-contents)
  - [Hardware Setup](#hardware-setup)
  - [Docker Setup](#docker-setup)
  - [How to Use](#how-to-use)


## Hardware Setup
The repository has currently been setup with
- RGB Camera: Realsense D455
  - No Additional Setup is required for this
- 3D LiDAR: Velodyne VLP16
  - LiDAR IP: 172.16.101.71
  - Host IP: 172.16.101.77
- RGB Camera(Deprecated): Logitech C270
  - Setup UDEV Rules by running the `c270.sh` script in the `scripts` folder on **HOST**, and not in container
    ```bash
    sudo bash <path to script>/c270.sh
    ```

## Docker Setup
- To pull latest docker image
    ```bash
    docker pull ghcr.io/soham2560/humble-lidar-cam:latest
    ```
- To start container
    - Open Command Pallete with `Ctrl+Shift+P`
    - Select Option to Rebuild and Reopen Container
    - Use `Build WS` button to build workspace
  
  Note: To access these buttons you may need to enable [VSCode Action Button Extension](https://marketplace.visualstudio.com/items?itemName=seunlanlege.action-buttons) through the Extensions Tab in VSCode, the extension should download automatically on container startup

## How to Use
- Build and Source the Workspace
- Ensure Hardware is setup properly and powered on
- Use following command to launch
  ```bash
  ros2 launch lidar_camera_calibration calibration.launch.py use_rviz:=True
  ```

Note: The README's in this repository are inspired by [this](https://github.com/TheProjectsGuy/MR21-CS7.503) and [this](https://github.com/ankitdhall/lidar_camera_calibration)