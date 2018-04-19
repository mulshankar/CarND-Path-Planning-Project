# CarND-Path Planning Project
Self-Driving Car Engineer Nanodegree Program

---

## Project Objectives

Implement a path planning algorithm to enable a self driving car to navigate around a track provided by the Udacity simulator and safely perform lane changes.

[//]: # (Image References)
[image1]: ./Images/BehaviorPlannerOverview.png
[image2]: ./Images/TrackOverview.PNG
[image3]: ./Images/FrenetLaneChange.png
[image4]: ./Images/Weights.PNG
[image5]: ./Images/MeasurementPrediction.PNG
[image6]: ./Images/UKFupdate1.PNG
[image7]: ./Images/NIS.PNG
[image8]: ./Images/ChiSquare.PNG

## Trajectory Generation Overview

The overall control architecture of a self-driving car is shown below. The key 4 elements - Perception, Localization, Control and Path-Planning. Individual modules of the behavior planner are highlighted within the blue box along with information flow. The focus of this project is on planning a trajectory for the car to traverse that is ideal in terms of speed, acceleration and jerk. The trajectory planner takes information from the localization module, prediction module and behavior planner modules. 

![alt text][image1]

## Test Track

The simulator track is 6945.554 meters around (about 4.32 miles). If the car averages near 50 MPH, then it should take a little more than 5 minutes for it to go all the way around the highway. The highway has 6 lanes total - 3 heading in each direction. Each lane is 4 m wide and the car should only ever be in one of the 3 lanes on the right-hand side. The car should always be inside a lane unless doing a lane change. 

![alt text][image2]

## Algorithm Overview

Before diving into the algorithm, it is important to introduce the concept of Frenet coordinates - s and d. Frenet coordinates simplify the car position from cartesian (x,y) coordinates to road coordinates. The 's' value is the distance along the direction of the road. The first waypoint has an s value of 0 because it is the starting point. The d vector has a magnitude of 1 and points perpendicular to the road in the direction of the right-hand side of the road. The d vector can be used to calculate lane positions. Frenet coordinates along with time can accurately capture the desired safe trajectory for the vehicle to navigate. 

![alt text][image3]

While frenet coordinates are good for trajectory planning, the localization module still is in the global X,Y coordinates. Therefore, once the trajectory is planned in the frenet coordinate system, it is converted into XY coordinate system for the car's motion control module. The algorithm can be sub-sectioned into 2 parts - (1) First part decides on lane (2) Second part plans the trajectory for the desired lane

* The car initially starts in lane 1 ( 0- left, 1 -middle and 2-right lanes)

* The localization module reports the car's current x,y,s,d,yaw and speed information. 

* The first check is performed using information provided by the sensor fusion module. Information about cars in the current lane of travel is processed. A threshold of 30 m was set to identify if the test vehicle is too close to a vehicle right in front of it. 

```
for (int i=0;i<sensor_fusion.size();i++)	
{
	double other_car_d=sensor_fusion[i][6]; // get the 'd' coordinate of the car

	if (other_car_d<=4+(4*lane) && other_car_d>=(4*lane))	{ // assuming our car is in center of lane, check + - 2 m in given lane
		
		double other_car_vx=sensor_fusion[i][3];
		double other_car_vy=sensor_fusion[i][4];
		double other_car_speed=sqrt(other_car_vx*other_car_vx+other_car_vy*other_car_vy);
		
		double other_car_s=sensor_fusion[i][5];
		
		other_car_s=other_car_s+ prev_path_size*0.02*other_car_speed; // predict where the car will be at the end of its current planned path
		
		if ((other_car_s > car_s) && (other_car_s-car_s<30)) {					
			car_ahead=true;
		}					
	}			
}

```
* A simple switch case block decides on available options for lane changes

* If there is car ahead in the lane that is too close, the first step is to reduce speed and then explore if a lane change maneuver is safe to perform. The check uses sensor fusion information of cars in the "desired" lane. A velocity constraint was also added to minimize jerk or acceleration. For example, the car should not perform lane change at 50 mph. 

* Once the lane is decided, the next step is to generate a safe trajectory for the car to traverse. While the trajectory is planned for a distance of 90 m, the base loop runs at 20 ms. The car probably travels for about the first few meters. The remaining points from the previous planned path is retained. 

* A set of anchor points or way-points are generated at wide-spaced intervals (30 m in our case) in the frenet coordinate system. A helper functio getXY converts these way-points into the cartesian coordinate X,Y system

* These new way-points are now appended to the untraversed points of the previous planned trajectory. This ensures a smooth continuous path. 

* The spline library was used to fit a polynomial through these points. Spline ensures the generated polynomial goes through every single way-point.

* A simple calculation is then done based on current velocity, loop time and distance to compute the desired x,y coordinates.

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






