## Why?
This file contains some useful terms used frequently in the workshop and during test days. Some of them are MMR-Specific

## Car Design vocabulary


## Infi vocabulary

- **IMU**: Inertial Measurement Unit, a device that measures the specific force, angular velocity, and magnetic field surrounding the sensor. Our IMU is Xsens Mti-680G, a 9 Degrees of Freedom (DoF) IMU.
- **IMU Bias**: The difference between the measured value and the true value of a sensor. It can be caused by various factors, including temperature changes, aging, and manufacturing tolerances.
- **INS**: Inertial Navigation System, a navigation aid that uses a computer, motion sensors, and rotation sensors to continuously calculate the position, orientation, and velocity of a moving object without the need for external references.
- **GNSS**: Global Navigation Satellite System, a satellite system that provides autonomous geo-spatial positioning with global coverage.
- **RTK**: Real-Time Kinematic, a technique used in satellite navigation to enhance the precision of position data derived from satellite-based positioning systems. Precision is usually better than 1 cm + 1 ppm.
- **LiDAR**: Light Detection and Ranging, a remote sensing method that uses light in the form of a pulsed laser to measure ranges (variable distances) to the Earth.
- **ODOMETRY**: The use of data from motion sensors to estimate change in position over time. It is used in robotics and navigation to determine the position of a vehicle or robot relative to a starting point.
- **KF**: Kalman Filter, an algorithm that uses a series of measurements observed over time, containing statistical noise and other inaccuracies, and produces estimates of unknown variables that tend to be more precise than those based on a single measurement alone.
- **EKF**: Extended Kalman Filter, a version of the Kalman filter that linearizes about an estimate of the current mean and covariance.
- **UKF**: Unscented Kalman Filter, a recursive algorithm for estimating the state of a nonlinear dynamic system from a series of noisy measurements.
- **SLAM**: Simultaneous Localization and Mapping, a method used in robotics and computer vision to build a map of an unknown environment while simultaneously keeping track of the agent's location within it.