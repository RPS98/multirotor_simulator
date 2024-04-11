# Multirotor Simulator in C++

This library contains the following main components:
- Dynamics
- Control
- State

## Dynamics
The main components of the dynamics are the following:
- [State](State.md)
- [Actuation](Actuation.md)
- [Kinematics](Kinematics.md)
- [Dynamics](Dynamics.md)

## Control
This library implement a collection of controllers for multirotor vehicles.

- PID Controller por position and velocity references.
- Geometric Controller based on Differential Flatness (DF) for trajectory tracking.
- Incremental Nonlinear Dynamic Inversion (INDI) controller for attitude control.

## IMU and Inertial Odometry
This library implements a simple IMU simulator, adding noise to the angular velocity and linear acceleration measurements, and an inertial odometry estimator, which integrates the linear acceleration measurements to estimate state.

## References
Beard, R. W. (2008). [Quadrotor dynamics and control](https://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=2324&context=facpub). Brigham Young University, 19(3), 46-56.

Smeur, Ewoud & Chu, Q.P. & Croon, Guido. (2015). [Adaptive Incremental Nonlinear Dynamic Inversion for Attitude Control of Micro Air Vehicles](https://pure.tudelft.nl/ws/files/9188985/Adaptive_Incremental_Nonlinear_Dynamic_Inversion_or_Attitude_Control_of_Micro_Aerial_Vehiclesa.pdf). Journal of Guidance, Control, and Dynamics. 39. 1-12. 10.2514/1.G001490.

D. Mellinger and V. Kumar, [Minimum snap trajectory generation and control for quadrotors](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=5980409), 2011 IEEE International Conference on Robotics and Automation, Shanghai, China, 2011, pp. 2520-2525, doi: 10.1109/ICRA.2011.5980409.