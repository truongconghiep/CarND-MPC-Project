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

Number of timesteps N and elapsed time dt determine the prediction horizon. The number of timesteps have a big impact on the stability of the controller. With a number of timesteps smaller than 5, the controller is completely oscillating. The same result was also observed with a number of timesteps bigger than 15. One more thing was observed during tuning N is that the car acceleration is increasing when the number of timesteps increases. After trying with N in range from 5 to 15 and timestep in range from 50 to 100 milliseconds, I choose 15 timesteps and 100 ms steptime to have a good stability of the controller.

The timestep dt affects quite a lot on the system. The smaller dt, the better system response but then it requires a bigger number of timesteps, which requires higher computational ability. Since the latency of the system is 100 ms, the minimum timestep should not below this boundary.

To keep the controller stable, following cost weights are introduced in the cost function.

```
/* MPC cost weights */
#define CTE_COST_WEIGHT                 (double)2000
#define EPSI_COST_WEIGHT                (double)2000
#define DELTA_COST_WEIGHT_PROPORTIONAL  (double)35
#define A_COST_WEIGHT_PROPORTIONAL      (double)35
#define DELTA_COST_WEIGHT_DIFFERENT     (double)5000000
#define A_COST_WEIGHT_DIFFERENT         (double)20000
```

Cost function:

```
/* The part of the cost based on the reference state.*/
for (unsigned int t = 0; t < N; t++) 
{
  fg[0] += CTE_COST_WEIGHT * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
  fg[0] += EPSI_COST_WEIGHT * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
  fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
}
/* Minimize the use of actuators.*/
for (unsigned int t = 0; t < N - 1; t++) 
{
  fg[0] += DELTA_COST_WEIGHT_PROPORTIONAL * CppAD::pow(vars[delta_start + t], 2);
  fg[0] += A_COST_WEIGHT_PROPORTIONAL * CppAD::pow(vars[a_start + t], 2);
}

/* Minimize the value gap between sequential actuations.*/
for (unsigned  int t = 0; t < N - 2; t++) 
{
  fg[0] += DELTA_COST_WEIGHT_DIFFERENT * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
  fg[0] += A_COST_WEIGHT_DIFFERENT * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}
```


## Polynomial Fitting and MPC Preprocessing
The waypoints provided by the simulation is transformed into car coordination system as below

```
/* shift car reference angle to 90 degree*/ 
for (uint_t i = 0; i < ptsx.size(); i++)
{
   double shift_x = ptsx[i] - px;
   double shift_y = ptsy[i] - py;
   ptsx[i] = (shift_x * cos(0 - psi) - shift_y * sin( 0 - psi));
   ptsy[i] = (shift_x * sin(0 - psi) + shift_y * cos( 0 - psi));
}
Eigen::Map<Eigen::VectorXd> ptsx_transform(&ptsx[0], 6);
Eigen::Map<Eigen::VectorXd> ptsy_transform(&ptsy[0], 6);
```

Then they are fitted to a third order polynomial

```
auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);
```

## Model Predictive Control with Latency

To deal with the latency of actuation, a small delay is implemented to predict the system state after the latency, which is new initial state for MPC. The prediction of new state after latency is implemented as followed:

```
/* Actuator delay in milliseconds.*/
const int actuatorDelay = 100;

/* Actuator delay in seconds.*/
const double delay = actuatorDelay / 1000.0;
		  
/* Initial state.*/
const double x0 = 0;
const double y0 = 0;
const double psi0 = 0;
const double cte0 = coeffs[0];
const double epsi0 = -atan(coeffs[1]);
const double psides0 = atan(3 * coeffs[3] * x0 * x0 + 2 * coeffs[2] * x0 + coeffs[1]);

/* State after delay.*/
double x_delay = x0 + ( v * cos(psi0) * delay );
double y_delay = y0 + ( v * sin(psi0) * delay );
double psi_delay = psi0 - ( v * delta * delay / mpc.Lf );
double v_delay = v + a * delay;
double cte_delay = cte0 + ( v * sin(epsi0) * delay );
double epsi_delay = psi0 - psides0 +  v * (delta * delay / mpc.Lf);


/* Define the state vector.*/
Eigen::VectorXd state(6);
state << x_delay, y_delay, psi_delay, v_delay, cte_delay, epsi_delay;
```

## Result

With my implementation the simulation car can drive itself around the lake track safely. It can also reach a max speed of 71 mph on the straight line. A short demostration video is provided [here.](https://www.youtube.com/watch?v=rN-fmsUrDbU)


 
  
 
 
