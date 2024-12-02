# orin-drive
Software stack built during 2024-25 season

## Overview
This software stack is designed for an autonomous racing car, structured into six hierarchical layers that process sensor data and control vehicle movement.

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