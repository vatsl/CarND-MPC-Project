# Model Predictive Controller (MPC)

---

In this project I have tried to improve upon the [PID project](https://github.com/coldKnight/CarND-PID-Control-Project) by implementing a MPC to drive the car around the track. The result was a much smoother driving.

## Output

Click to view the video

[![Final results with the implementation](http://img.youtube.com/vi/LhtFcLbGKeU/0.jpg)](http://www.youtube.com/watch?v=LhtFcLbGKeU)


## The Model
The Model Predictive Controller (MPC) calculates the trajectory, actuations and sends back
 steering to the simulator. <br/>
The state vector of the vehicle is given as:
```
x - Vehicle position in forward direction
y - Vehicle position in lateral direction
psi - Angle of the vehicle (yaw angle)
v - Vehicle's speed
cte - cross-track error
epsi - orientation error

And the actuators are:
delta - Steering angle (radians)
a - acceleration

Lf - the distance between the center of mass of the vehicle and the front wheels.
```

The model is expressed by the following equations:

```
      x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      psi_[t+1] = psi[t] + v[t] * delta[t] / Lf * dt
      v_[t+1] = v[t] + a[t] * dt
      cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

## Polynomial Fitting and MPC Preprocessing
The waypoints are transformed to vehicle coordinate system by translation and rotation. 
X axis aligns with the heading direction. This transformation allows to perform 
calculations consistently in vehicle coordinate system.
```
   car_x = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi);
   car_y = (ptsy[i] - py) * cos(psi) - (ptsx[i] - px) * sin(psi);
```

A third degree polynomial was used to compute the trajectory of the car. As mentioned in the 
lectures, most of the real world roads can be fitted with a third degree polynomial.
A second degree polynomial might have led to underfitting, whereas a greater degree 
polynomial might lead to overfitting.

## Timestep Length and Elapsed Duration (N & dt)
Timestep Length and Frequency were chosen by trial and error.
I started with 10 timesteps (`N`) of 0.1 duration (`dt`) with a speed of 30 mph. 
However, there was a lot of erratic driving behavior with these values. Increasing thr value
 of `N` made matters worse. Increasing the value of `dt` led to some improvements.
Soon I figured that the faster we want to go, the futher we must be able to look and a 
higher `dt` made the driving smoother.
With this in mind, I finally set the `N` to 10 and `dt` to 0.2 as that gave me the best
driving behavior at a speed of 30 mph.

## Model Predictive Control with Latency
A latency of 100ms is artificially added before sending actuations to the simulator to simulate
real world conditions. 
Failure to handle the latency problem might lead to unrealistic trajectories and erratic driving
 behavior.
 
Use the update equations and model errors in order to factor latency in the state vector. 

```
    Lf=2.67, latency=0.1 sec
    x_dl = (0.0 + v * latency);
    y_dl = 0.0;
    psi_dl = 0.0 + v * steer_value_input / Lf * latency;
    v_dl = 0.0 + v + throttle_value_input * latency;
    cte_dl = cte + (v * sin(epsi) * latency);
    epsi_dl = epsi + v * steer_value_input / Lf * latency;
```

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.


## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
