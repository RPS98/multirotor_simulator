# Multirotor State

Multirotor is a rigid body, which mass center is the origin of the body frame. 

## Kinematics
the state from the point of view of kinematics can be defined as:

* **position**

```math
p = [x, y, z]
```

* **attitude** 

```math
q = [\phi, \theta, \psi] = [w, q_1, q_2, q_3]
```

* **velocity** 

```math
v = [v_x, v_y, v_z]
```

* **angular velocity** 

```math
\omega = [\omega_x, \omega_y, \omega_z]
```

* **linear acceleration** 

```math
a = [a_x, a_y, a_z]
```

* **Angular acceleration** 

```math
\alpha = [\alpha_x, \alpha_y, \alpha_z]
```

## Dynamics
The state from the point of view of dynamics can be defined as sum of forces and moments acting on the multirotor:

* **Force** 

```math
F = [F_x, F_y, F_z]
```

* **Torque** 

```math
\tau = [\tau_x, \tau_y, \tau_z]
```

## Actuation
The state from the point of view of actuation can be defined as the state of the motors:

* **Motor angular velocity** 

```math
\omega_m = [\omega_1, \omega_2, ..., \omega_n]
```

* **Motor angular acceleration**

```math
\alpha_m = [\alpha_1, \alpha_2, ..., \alpha_n]
```