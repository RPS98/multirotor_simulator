# Multirotor Kinematics

Given a delta time $\Delta t$, each multirotor [state](State.md) derivative must be calculated to get the new state.

```math
\dot{x}
=
\begin{bmatrix}
\dot{x} \\
\dot{q} \\
\dot{v} \\
\dot{\omega}
\end{bmatrix}
=
\begin{bmatrix}
\vec{v} \\
\frac{1}{2} \cdot \vec{\omega} \times q \\
\frac{1}{m} \cdot \vec{F} \\
I^{-1} \cdot (\vec{\tau} - \vec{\omega} \times I \vec{\omega}) 
\end{bmatrix}
```

<span style="font-size: 20px;">Multirotor Kinematics Table of Contents </span>

- [1. Motors Angular Velocity Derivative](#1-motors-angular-velocity-derivative)
- [2. Vehicle Angular Velocity Derivative](#2-vehicle-angular-velocity-derivative)
- [3. Vehicle Linear Velocity Derivative](#3-vehicle-linear-velocity-derivative)
- [4. Vehicle Orientation Derivative](#4-vehicle-orientation-derivative)
- [5. Vehicle Position Derivative](#5-vehicle-position-derivative)

## 1. Motors Angular Velocity Derivative

Each motor can be modeled as a first order system, with a time constant $\tau$, an angular velocity maximum $\omega_{max}$ and a minimum $\omega_{min}$:

```math
\frac{d\omega}{dt} = \frac{1}{\tau} \cdot (\omega_{des} - \omega)
```

Where:
- $\omega_{des}$ is the desired angular velocity of the motor between $\omega_{min}$ and $\omega_{max}$.
- $\omega$ is the angular velocity of the motor.
- $\tau$ is the time constant of the motor dynamics.

## 2. Vehicle Angular Velocity Derivative

Motors angular velocity changes produce a torque on the multirotor. Also, environment forces and torques produce a torque too. 

The angular momentum of a rigid body is given by the product of the inertia matrix and the angular velocity vector:

```math
\vec{L} = I \cdot \vec{\omega}_{B}
```

Where:
- $\vec{L}$ is the angular momentum of the multirotor
- $I$ is the inertia matrix of the multirotor
- $\vec{\omega}_{B}$ is the angular velocity of the multirotor in the body frame

Due to the Coriolis effect (or gyroscopic torque), the angular momentum derivative is:

```math
\frac{d\vec{L}}{dt} = I \cdot \frac{d\vec{\omega}}{dt} + \vec{\omega}_{B} \times (I \cdot \vec{\omega}_{B})
```

Also, the angular momentum derivative is equal to the sum of all external torques acting on the rigid body:

```math
\frac{d\vec{L}}{dt} = I \cdot \frac{d\vec{\omega}}{dt} = \sum_{i=1}^{n} \vec{\tau}_i
```

Where:
- $\vec{\tau}_i$ is the torque vector produced by the $i$-th external force or torque.

Reordering the terms, the angular velocity derivative can be calculated as:

```math
\frac{d\vec{\omega}}{dt} = I^{-1} \cdot [\sum_{i=1}^{n} \tau_i - \vec{\omega}_{B} \times (I \cdot \vec{\omega}_{B})]
```

The sum of all external torques acting on the rigid body is exposed in [dynamics section](Dynamics.md).

## 3. Vehicle Linear Velocity Derivative

The velocity is calculated using the acceleration:

```math
\dot{\vec{v}} = \vec{a}
```

Where:
- $\vec{a}$ is the acceleration of the multirotor.

The acceleration is calculated using Newton's second law of motion:

```math
\vec{a} = \frac{1}{m} \cdot \vec{F}
```

Where:
- $m$ is the mass of the multirotor.
- $\vec{F}$ is the total force applied to the multirotor.

The total force applied to the multirotor is the sum of the forces applied to the multirotor by the rotors and the forces applied to the multirotor by the environment, exposed in the previous section.


## 4. Vehicle Orientation Derivative

The attitude derivate is calculated using the angular velocity (quaternion derivative), with the Hamiltonian product:

```math
\dot{q} = \frac{1}{2} \cdot q \times \vec{\omega}
```

Where:
- $q$ is the attitude of the multirotor.
- $\vec{\omega}$ is the angular velocity of the multirotor.


## 5. Vehicle Position Derivative

The position is calculated using the velocity:

```math
\dot{\vec{p}} = \vec{v}
```

Where:
- $\vec{v}$ is the lineal velocity of the multirotor.