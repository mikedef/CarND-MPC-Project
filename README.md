# CarND-Controls-MPC 

## Michael DeFilippo

#### Please see my [project code](https://github.com/mikedef/CarND-MPC-Project/tree/master/src) for any questions regarding implementation.
---

# Overview
This repository contains all the code needed to complete the final project for the MPC course in Udacity's Self-Driving Car Nanodegree.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

---
## Model Predictive Control
In this project you'll implement Model Predictive Control to drive the car around the track. This time however you're not given the cross track error, you'll have to calculate that yourself! Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.

### The Model
#### Student describes their model in detail. This includes the state, actuators and update equations.
The kinematic model takes in parameters from the simulator to define the cars state
` state << x, y, psi, v, cte, epsi;`
* x:    x-position of the vehicle in global coordinates
* y:    y-position of the vehicle in global coordinates
* psi:  vehicles orientation
* v:    vehicles velocity
* cte:  cross track error
* epsi: orientation error

Actuator outputs are acceleration (a) and steering angle (delta). The model combines the vehilce state and actuations from the previous timestep to calculate the state for the current timestep. 
Recall the equations for the model:                              
* x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt                         
* y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt                         
* psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt                   
* v_[t+1] = v[t] + a[t] * dt                                       
* cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt             
* epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt       

### Timestep Length and Elapsed Duration (N & dt)
#### Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

For Timestep Duration (dt) I used 0.1 seconds to match the delay of 100ms of the simulator. For Tuning Timesteps(N) I ended up using 10 which equates to a 1 second duration between sensor readings and correction. I attempted differend values for N and dt such as 15 and 0.2 or 20 and 0.5, but both ended up in producing erratic driving of the vehicle. 

### Polynomial Fitting and MPC Preprocessing
#### A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

The waypoints are preprocessed such that the waypoints from the simulator end up in the vehicles perspective. This makes it easier to fit a polynomial to the waypoints since the vehicle's x-y coordinates are now the origin and the orientation angle is also transformed to zero. 

### Model Predictive Control with Latency
#### The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

In MPC.cpp I deal with the 100 ms latency by using previous actuations in the equations for acceleration and steering angle (see MPC.cpp lines 116-119). Also as per the lessons on MPC, I included cost functions weighing the equations. Also I added a cost function (speed_steer_cost_function) that punishes based on the speed and steering to help the vehicle during steering. 
