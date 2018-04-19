# CarND-Path Planning Project
Self-Driving Car Engineer Nanodegree Program

---

## Project Objectives

Implement a path planning algorithm to enable a self driving car to navigate around a track provided by the Udacity simulator and safely perform lane changes.

[//]: # (Image References)
[image1]: ./Images/StateDefine.PNG
[image2]: ./Images/Errors.PNG
[image3]: ./Images/StateEqns.PNG
[image4]: ./Images/Weights.PNG
[image5]: ./Images/MeasurementPrediction.PNG
[image6]: ./Images/UKFupdate1.PNG
[image7]: ./Images/NIS.PNG
[image8]: ./Images/ChiSquare.PNG

## Behavior Planner Overview

The overall control architecture of a self-driving car is shown below. The key 4 elements - Perception, Localization, Control and Path-Planning. Individual modules of the behavior planner are highlighted within the blue box. The localization and prediction modules feed into the behavior planner module which sends its information to the Trajectory planning module. The focus of this project is on planning a trajectory for the car to traverse that is ideal in terms of speed, acceleration and jerk. 

![alt text][image1]

The cross track error and orientation error can be represented as shown below. The dashed white line indicates cross track error. 

![alt text][image2]

There are typically 3 control inputs in a vehicle - throttle, brake and steer. Throttle and brake can be combined into a single input with bounds [-1,1]. A negative value implies braking. Therefore, the system has 2 control inputs [steer, throttle].

The state equations are given below:

![alt text][image3]

## Algorithm Overview

There are four pieces to a self driving car - Perception, Localization, Path Planning and Control. The focus of this project is on control. It is assumed that the path planning algorithm would yield a desired set of way-points for the car to traverse. The MPC algorithm would project the vehicle's state in the future and minimize the error between the vehicle's trajectory and desired path. 

The desired way-points from the path planning algorithm are in the global map co-ordinates. In order to calculate the cross track error as well as the orientation error, it is convenient to convert these way points to the car's co-ordinate system. The code below performs this transformation. 

```
for (int i=0;i<ptsx.size();i++) { // Transform way points from map coordinate to car coordinate system
			
	double shift_x = ptsx[i]-px; // translation move brings the point to car co-ordinates origin
	double shift_y = ptsy[i]-py;

	ptsx[i] = (shift_x*cos(0-psi) - shift_y*sin(0-psi)); // rotational move
	ptsy[i] = (shift_x*sin(0-psi) + shift_y*cos(0-psi));  		  
}

```
Once the way points are converted to the car's co-ordinate system, a 3rd order polynomial is fit to generalize the way-points. This polynomial would form the desired/reference trajectory that the MPC algorithm wants the car to follow. 

The car's current states are received from the simulator. The car's current states along with the polynomial coefficients are passed to the MPC algorithm. The MPC algorithm projects the car's states into the future and calculates the desired steer and throttle actuation that minimizes the cross track and orientation errors. 

The received commands from the MPC algorithm were fed into the simulator and verified to ensure the car runs smooth without veering off the desired path or outside the curb. 

## MPC Algorithm

The first step of the MPC algorithm was to define the horizon and time step over which the algorithm optimizes the error. 

* The problem is solved in a receding horizon fashion. While the look ahead info helps in optimizing the actuator commands, too long of a horizon is not useful either. In real world, there is a fair bit of uncertainty in what comes ahead. The calculated actuations are applied for the current time step and the optimization is repeated from the new time step to the end of the horizon. 

* The MPC algorithm essentially breaks down a continuous time problem into the N discrete steps and converts it into an optimization problem. N is given as time horizon (H) divided by time step (dt). The total states that the algorithm optimizes is number of states (6) * N + actuations at each time step. The longer the horizon, higher is the computational effort. 

Taking into account the above two constraints, a prediction horizon of 1 second with a time step dt of 0.1 second worked well for the problem. 

```
size_t N = 10;
double dt = 0.1;

```
The next step applies the constraints for the states and acutations. The differential equations described above provide the state constraints. The steer was limited to an absolute value of 25 degrees and the pedal was constrained between +1 and -1.

```
// The upper and lower limits of delta are set to -25 and 25
// degrees (values in radians).Feel free to change this to something else.
	
for (int i = delta_start; i < a_start; i++) {
vars_lowerbound[i] = -0.436332; // 25*pi/180
vars_upperbound[i] = 0.436332;
}

// Acceleration/decceleration upper and lower limits.
// NOTE: Feel free to change this to something else.
for (int i = a_start; i < n_vars; i++) {
vars_lowerbound[i] = -1.0; // full brake
vars_upperbound[i] = 1.0; // full throttle
}

```
Interior Point Optimizer (Ipopt - https://projects.coin-or.org/Ipopt) was used to solve the framed optimization problem. Ipopt is an open source software package for large scale non-linear optimization. Ipopt requires we give it the jacobians and hessians directly - it does not compute them for us. Hence, we need to either manually compute them or have a library do this for us. Luckily, there is a library called CppAD which does exactly this. By using CppAD we don't have to manually compute derivatives, which is tedious and prone to error.

```
// place to return solution
CppAD::ipopt::solve_result<Dvector> solution;

// solve the problem
CppAD::ipopt::solve<Dvector, FG_eval>(
  options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
  constraints_upperbound, fg_eval, solution);

```
FG_eval contains the cost function and sets up the constraints in the desired fashion for Ipopt. The cost function used for the optimization is shown below. As seen, there are three parts to the cost function. The first part deals with error minimization. A velocity error term is introduced to ensure the car does not simply stop once the cross track and orientation errors are minimized. The second part of the cost function is to constrain excessive use of the actuators. The third part of the cost function is to constrain relative change in actuators. This minimizes abrupt changes in the actuator positions, especially the steer angle.  

```
// Reference State Cost

	// The part of the cost based on the reference state.
	for (int t = 0; t < N; t++) {
	  fg[0] += 5*CppAD::pow(vars[cte_start + t]-cte_des, 2);
	  fg[0] += 5*CppAD::pow(vars[epsi_start + t]-epsi_des, 2);
	  fg[0] += 0.5*CppAD::pow(vars[v_start + t] - v_des, 2);
	}

	// Minimize the use of actuators.
	for (int t = 0; t < N - 1; t++) {
	  fg[0] += 50*CppAD::pow(vars[delta_start + t], 2);
	  fg[0] += 50*CppAD::pow(vars[a_start + t], 2);
	}

	// Minimize the value gap between sequential actuations.
	for (int t = 0; t < N - 2; t++) {
	  fg[0] += 5000*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
	  fg[0] += 100*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
	}


```

The values shown above worked well for this project. 

## Latency

In real world, there is always some latency in response time of actuators. One of the strong suites of the MPC algorithm is the fact that this latency can be seamlessly integrated into the control structure. In this case, a latency of 100 milliseconds is assumed. The states are estimated using the motion model over the next 100 milliseconds. This estimated state is then passed over to the MPC algorithm to calculate the optimized actuations at the next time step. The code below handles the actuator latency for this project. 

```
//Before calculating delta and accel via MPC, predict states at latency dt		  
// We are already in the car coordinate system.. x,y and psi are 0

double x_l=0+v*cos(0)*latency;
double y_l=0+v*sin(0)*latency;
double psi_l=0+(v/Lf)*(steer_rad)*latency;
double v_l=v+accel*latency;

double epsi=0-atan(coeffs[1]); // psi - psi_des at current time step		  
double cte_l=polyeval(coeffs,0)-0+v*sin(epsi)*latency;

double epsi_l=0-atan(coeffs[1])+(v/Lf)*(steer_rad)*latency;		  

Eigen::VectorXd state(6);
state<<x_l,y_l,psi_l,v_l,cte_l,epsi_l;

```
## Closure

A Model Predictive Controller was implemented on a self driving car for navigating around the Udacity provided test track. The gains were tuned in order to maximize speed across the track. The car top speed was around 45 mph. The car navigated very well within the test track without going off the curbs under all circumstances. Multiple laps yielded the same result thus verifying repeatability. 






