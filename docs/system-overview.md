# Hardware

## Main Board
AGX Orin, 64GB

## Sensors
- [Xsens Mti-680g IMU](https://www.movella.com/products/sensor-modules/xsens-mti-680g-rtk-gnss-ins)
- Internal Lidar IMU
- Camera -> to define
- Ouster OS1 Lidar
- Reach RS2 Rover & Base (RTK + GNSS Module)
- Wheel Speed Sensor

# Software
TODO: rifare sta roba, fa schifo e l'ho usata come placeholder
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

## Data Flow
Sensor → Perception → Localization → Planning → Control → Actuation

## Architecture Principles
- Modular design
- Independent layer processing
- Clear hierarchical data flow