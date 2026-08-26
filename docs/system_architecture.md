# System Architecture

## Objective

This project develops a simulation and validation platform for Advanced
Driver Assistance Systems (ADAS) using CARLA, Autoware, and ROS 2.

## Validated Architecture

CARLA 0.10.0
    |
    | RPC / TCP 2000
    v
CARLA Python API 0.10.0
    |
    v
autoware_carla_interface
    |
    v
ROS 2 Jazzy
    |
    v
Autoware 1.9.0

## Simulation Configuration

- CARLA: 0.10.0
- Autoware: 1.9.0
- ROS 2: Jazzy
- Python: 3.12
- Map: Town10HD_Opt
- Ego vehicle: Lincoln MKZ
- ROS middleware: Fast DDS
- Development environment: Ubuntu 24.04 LTS-based Docker environment

## Sensors

The current lightweight sensor configuration includes:

- Front RGB camera
- Top LiDAR
- GNSS
- IMU

Verified ROS 2 interfaces include:

- `/sensing/camera/CAM_FRONT/image_raw`
- `/sensing/camera/CAM_FRONT/camera_info`
- `/sensing/lidar/top/pointcloud_before_sync`
- `/sensing/gnss/pose_with_covariance`
- `/sensing/imu/tamagawa/imu_raw`
- `/vehicle/status/velocity_status`
- `/vehicle/status/steering_status`
- `/vehicle/status/gear_status`

## Current Milestone

CARLA 0.10.0 and Autoware 1.9.0 communication has been established.

A Lincoln MKZ has been spawned as the Autoware `ego_vehicle` in
Town10HD_Opt, and its sensor and vehicle-state interfaces are available
through ROS 2.

## Next Phase

The platform will be used to develop reproducible ADAS scenarios and
evaluate vehicle behavior using quantitative engineering metrics.
