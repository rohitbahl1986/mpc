# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---


## Rubric Points

### The Model
The model used for this simulation is the Kinematic Model i.e. it ignores the tire forces, gravity and mass. Therefore, it trades off accuracy for speed and simplification of the implemetation. Following are the equations of the model:

```
x[t+1] = x[t] + v[t] * cos(psi[t]) * delta_t
y[t+1] = y[t] + v[t] * sin(psi[t]) * delta_t
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * delta_t
v[t+1] = v[t] + a[t] * delta_t
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * delta_t
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * delta_t

```

### Timestep Length and Elapsed Duration (N & dt)
The final values of N (10) and dt(0.1), where choose based on trial and error. For N < 10 (say 5) and dt < 0.1 (say 0.05), the car goes completely of-course some time after starting. For N > 10 (say 15) and dt > 0.1 (say 0.15), the car starts to touch the curb quite a few times. When N == 10 and dt == 0.1, the car drives smoothly although not very fast, specially at the corners.

### Polynomial Fitting and MPC Preprocessing
The simulator provides the waypoints in the map's coordinate system. These are first transformed into cars perspective in main.cpp. Then the coefficients of a 3rd order polynomial are calculated which fit the waypoints and are eventually used to calculate the cte and epsi. 

### Model Predictive Control with Latency
To take into account the latency of applying the actuation commands, we predict the future state of the vehicle using the original kinematic equations with the delta_t equal to the actuation delay. This state is then fed into the Solve method in place of the initial (0) state.

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


## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


