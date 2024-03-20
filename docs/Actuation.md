# Multirotor Actuation

The are several ways to define the actuation of a multirotor. One of the lowest level of abstraction is the motor angular velocity, which is the input of the motors.

Some of the most common actuation modes are, from lowest to highest level of abstraction:

### **Motors angular velocity** 

```math
\omega_m = [\omega_1, \omega_2, \omega_3, ..., \omega_n]
```

where:
  * $\omega_i$ is the angular velocity of the $i$-th motor (rad/s)

### **Acro mode** 

```math
Acro = [T, p, q, r]
```

where:
  * $T$ is the collective thrust (N)
  * $p$ is the roll rate (rad/s)
  * $q$ is the pitch rate (rad/s)
  * $r$ is the yaw rate (rad/s)

### **Attitude mode** 

```math
Attitude = [T, \phi, \theta, \psi]
```

where:
  * $T$ is the collective thrust (N)
  * $\phi$ is the roll angle (rad)
  * $\theta$ is the pitch angle (rad)
  * $\psi$ is the yaw angle (rad)

### **Trajectory mode**

```math
Trajectory = [p, v, a, \psi]
```

where:
  * $p$ is the position (m)
  * $v$ is the velocity (m/s)
  * $a$ is the linear accelertion (m/s^2)
  * $\psi$ is the yaw angle (rad)

### **Speed mode** 

```math
v = [v_x, v_y, v_z, r]
```

where:
  * $v_x$ is the velocity in the x-axis (m/s)
  * $v_y$ is the velocity in the y-axis (m/s)
  * $v_z$ is the velocity in the z-axis (m/s)
  * $r$ is the yaw rate (rad/s)

### **Position mode** 

```math
p = [x, y, z, \psi]
```

where:
  * $x$ is the position in the x-axis (m)
  * $y$ is the position in the y-axis (m)
  * $z$ is the position in the z-axis (m)
  * $\psi$ is the yaw angle (rad)