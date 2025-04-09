# Hardware

## Main Boards
- AGX Orin Developer Kit, 64GB
- Kria KR260

## Sensors
- [Xsens Mti-680g IMU](https://www.movella.com/products/sensor-modules/xsens-mti-680g-rtk-gnss-ins)
- Internal Lidar IMU
- Hesai OT128 LiDAR
- Reach RS2 Rover & Base (RTK + GNSS Module + LoRa Receiver)

## High Level Connections scheme
![logic-connection-scheme](/docs/media/logic-connection-scheme.jpeg)
### CANBus & CANOPEN
![CANBUS-CANOPEN-scheme](/docs/media/CANBUS-CANOPEN-scheme.png)

# Software
TODO: rifare sta roba, fa schifo e l'ho usata come placeholder

## System specifications
- ORIN
  - [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
  - [Jetson 6](https://developer.nvidia.com/embedded/jetpack-sdk-60) (based on ubuntu 22)
- KRIA
  - Custom YOCTO Linux build

## Layers
### 0. Sensors
- Sensor drivers
- Communication protocols

### 1. Perception
- Processes camera and LiDAR data independently
- Components:
  - Camera pipeline
  - LiDAR pipeline
  - Sensor fusion module

### 2. Localization and Mapping
- Implements Simultaneous Localization and Mapping (SLAM)
- Tracks vehicle position and builds environmental map

### 3. Planning
- Generates optimal trajectory based on current position and cone locations
- Components:
  - partial trajectory generation (local planner)
  - complete optimized trajectory (global planner)

### 4. Control
- Implements vehicle control algorithms

### 5. Actuation
- Translates control signals to motor actions
- Components:
  - Motor signal generation
  - Actuator control
