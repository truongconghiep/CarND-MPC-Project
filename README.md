# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Control model
 In this project a kinematic model is deployed to control a car on the lake track. To keep the simplicity complex interactions, like tire forces or slipping forces are neglected. The architecture of a model predictive controller is shown below:
 
![Screenshot](image/MPC.png)

where:

State parameters

 * x, y: Car position
 * psi: actual direction
 * v: velocity
 * cte: Cross-Track error
 * epsi: Orientation error
 
 Conrol inputs(actuation):
 
 * a: acceleration
 * delta: steering angle
 
The model predictive control optimizes control inputs [delta, a] regarding to cost function, which is sum of quadratic errors of CTE and psi.
 
## Timestep Length and Elapsed Duration

Number of timesteps N and elapsed time dt determine the prediction horizon. The number of timesteps have a big impact on the stability of the controller. With a number of timesteps smaller than 5, the controller is completely oscillating. The same result was also observed with a number of timesteps bigger than 15. One more thing was observed during tuning N is that the car acceleration is increasing when the number of timesteps increases. After trying with N in range from 5 to 15 and timestep in range from 50 to 100 milliseconds, I choose 10 timesteps and 100 ms steptime to have a good stability of the controller.
 
  
 
 
