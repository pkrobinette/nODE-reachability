# Spring Mass Damper System

## Dynamics
The SMDS consists of a mass attached to a wall by both a spring and a damper. The forces of motion are in the horizontal ($+x$, $-x$) direction, eliminating the effects of gravity on the equations. The figure below shows this system as well as the constants associated with both the spring and the damper.
<p align="center">
<img src="./images/Mass-Spring-Damper.png" alt="sys diagram" width="300" height="200">
<p>

> `k` : spring constant
> `m` : mass of block
> `b` : damping coefficient
> `F` : Force acting on block
> `x` : displacement of block

The equations of motion for the system are derived as follows:
\begin{equation}
    \sum F = ma
\end{equation}
\begin{equation}
    F - bv - kx = ma
\end{equation}
\begin{equation}\label{eq:dyna}
    F(t) = m\frac{d^2x}{dt^2} + b\frac{dx}{dt} + kx(t)
\end{equation}

## Files
- `images`: plots generated from sys id and reachability
- `spring-net.mat`: trained neural network for spring mass damper system
- `spring_mass_damp.slx`: simulink model of the system that uses integrals
- `spring_mass_damp_tf.slx`: simulink model of the system that uses a transfer function
- `spring_nn_sys_id`: system identification of the system using neural networks
- `spring_og_sys_id`: system identification of the system using classic techniques.
