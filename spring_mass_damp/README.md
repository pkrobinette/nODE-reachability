# Spring Mass Damper System

## Dynamics
The SMDS consists of a mass attached to a wall by both a spring and a damper. The forces of motion are in the horizontal ($+x$, $-x$) direction, eliminating the effects of gravity on the equations. The figure below shows this system as well as the constants associated with both the spring and the damper.
<p align="center">
<img src="./images/Mass-Spring-Damper.png" alt="sys diagram" width="300" height="200">
<p>

> `k` : spring constant
> 
> `m` : mass of block
> 
> `b` : damping coefficient
> 
> `F` : Force acting on block
> 
> `x` : displacement of block

<p align="center">
<img src="http://www.sciweavers.org/tex2img.php?eq=%0A%5Cbegin%7Bequation%7D%0A%20%20%20%20%5Csum%20F%20%3D%20ma%0A%5Cend%7Bequation%7D%0A%5Cbegin%7Bequation%7D%0A%20%20%20%20F%20-%20bv%20-%20kx%20%3D%20ma%0A%5Cend%7Bequation%7D%0A%5Cbegin%7Bequation%7D%5Clabel%7Beq%3Adyna%7D%0A%20%20%20%20F%28t%29%20%3D%20m%5Cfrac%7Bd%5E2x%7D%7Bdt%5E2%7D%20%2B%20b%5Cfrac%7Bdx%7D%7Bdt%7D%20%2B%20kx%28t%29%0A%5Cend%7Bequation%7D&bc=White&fc=Black&im=jpg&fs=12&ff=modern&edit=0" align="center" border="0" alt="\begin{equation}    \sum F = ma\end{equation}\begin{equation}    F - bv - kx = ma\end{equation}\begin{equation}\label{eq:dyna}    F(t) = m\frac{d^2x}{dt^2} + b\frac{dx}{dt} + kx(t)\end{equation}" width="372" height="103" /><p>
  
## Files
- `images`: plots generated from sys id and reachability
- `spring-net.mat`: trained neural network for spring mass damper system
- `spring_mass_damp.slx`: simulink model of the system that uses integrals
- `spring_mass_damp_tf.slx`: simulink model of the system that uses a transfer function
- `spring_nn_sys_id`: system identification of the system using neural networks
- `spring_og_sys_id`: system identification of the system using classic techniques.
