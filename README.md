# ADAS-in-Autonomous-Car

## Graduate-Level ADAS Simulation, Integration, and Validation

A simulation-focused ADAS engineering project integrating CARLA, Autoware, ROS 2, and Python to study sensing, perception, planning, control, visualization, and quantitative validation.

## Current Platform

| Component | Version / Configuration |
|---|---|
| Simulator | CARLA 0.10.0 |
| Autonomous-driving stack | Autoware 1.9.0 |
| ROS | ROS 2 Jazzy |
| Python | 3.12 |
| Map | Town10HD_Opt |
| Ego vehicle | Lincoln MKZ |
| ROS middleware | Fast DDS |
| Development environment | Ubuntu 24.04 LTS-based Docker environment |

## CARLA to Autoware Architecture

CARLA 0.10.0
  -> CARLA Python API 0.10.0
  -> autoware_carla_interface
  -> ROS 2 Jazzy
  -> Autoware 1.9.0
  -> Lincoln MKZ

## Current Validated Integration

- CARLA 0.10.0 server running successfully
- Docker-to-CARLA communication over TCP port 2000
- CARLA Python API 0.10.0 working with Python 3.12
- ROS 2 Jazzy communication using Fast DDS
- Autoware 1.9.0 CARLA interface running
- Town10HD_Opt loaded
- Lincoln MKZ spawned as ego_vehicle
- Front RGB camera interface
- Top LiDAR interface
- GNSS interface
- IMU interface
- Vehicle status interfaces

## Verified ROS 2 Topics

`	ext
/sensing/camera/CAM_FRONT/image_raw
/sensing/camera/CAM_FRONT/camera_info
/sensing/lidar/top/pointcloud_before_sync
/sensing/gnss/pose_with_covariance
/sensing/imu/tamagawa/imu_raw
/vehicle/status/velocity_status
/vehicle/status/steering_status
/vehicle/status/gear_status
/vehicle/status/control_mode
/vehicle/status/actuation_status
`",
",


| # | Function | Engineering Concept | Status |
|---|---|---|---|
| 01 | Automatic Emergency Braking | Collision mitigation / TTC | Planned |
| 02 | Adaptive Cruise Control | Longitudinal control | Planned |
| 03 | Lane Keeping Assist | Lateral control | Planned |
| 04 | Lane Departure Warning | Safety monitoring | Planned |
| 05 | Automatic Lane Change | Behavior planning | Planned |
| 06 | Blind Spot Monitoring | Surrounding-object awareness | Planned |
| 07 | Pedestrian AEB | Vulnerable-road-user protection | Planned |
| 08 | Traffic Light Response | Perception and decision making | Planned |
| 09 | Obstacle Avoidance | Motion planning | Planned |
| 10 | Intersection Collision Avoidance | Conflict prediction | Planned |

## Validation Methodology

Each ADAS scenario will follow:

Requirement
-> Scenario definition
-> Sensor inputs
-> Perception / state estimation
-> Decision / planning
-> Control
-> Vehicle response
-> Quantitative metrics
-> Validation result

Candidate metrics include TTC, minimum separation, stopping distance, reaction time, velocity error, gap error, lateral error, heading error, trajectory deviation, and control latency.

## Engineering Focus

- ADAS system architecture
- Sensor-to-control reasoning
- Scenario-based simulation
- Safety metrics
- Vehicle behavior analysis
- ROS 2 integration
- Reproducibility
- Quantitative validation
- Documentation of assumptions and limitations

## Current Status

Platform integration is validated. The next milestone is Automatic Emergency Braking (AEB) scenario development and quantitative evaluation.
