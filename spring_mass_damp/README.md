# Spring Mass Damper System

## Dynamics
The SMDS consists of a mass attached to a wall by both a spring and a damper. The forces of motion are in the horizontal ($+x$, $-x$) direction, eliminating the effects of gravity on the equations. In [sys diagram] The figure below shows this system as well as the constants associated with both the spring and the damper.

![sys diagram](./images/Mass-Spring-Damper.png | width=100)



## Files
- `images`: plots generated from sys id and reachability
- `spring-net.mat`: trained neural network for spring mass damper system
- `spring_mass_damp.slx`: simulink model of the system that uses integrals
- `spring_mass_damp_tf.slx`: simulink model of the system that uses a transfer function
- `spring_nn_sys_id`: system identification of the system using neural networks
- `spring_og_sys_id`: system identification of the system using classic techniques.
