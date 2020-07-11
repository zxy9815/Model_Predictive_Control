# MPC-Controller

Self-Driving Car Engineer Nanodegree Program

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
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page).
* **Ipopt and CppAD:** Please refer to [this document](./install_Ipopt_CppAD.md) for installation instructions. Or simply intstall the two using `install_Ipopt_CppAD.sh`.

## Simulator.

* You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Read the [DATA.md](./doc/DATA.md) for a description of the data sent back from the simulator.

## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Model Predictive Control

### 1. The Model

The state variable in the model is constructed as below,

```
state = [ x
          y
          psi
          v
          cte
          epsi ]
```

The model used is a Kinematic model neglecting the complex interactions between the tires and the road. The model equations are as follow:

```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

where,

- `x, y` : Car's position.
- `psi` : Car's heading.
- `v` : Car's velocity.
- `cte` : Cross track error.
- `epsi` : Heading error.

In addition to those above, `Lf` is the distance between the car of mass and the front wheels (this is provided by Udacity's seed project). The other two values are the model output:

- `delta` : Steering angle.
- `a` : Car's acceleration (throttle).

The objective is to find the steering angle `delta` and the acceleration `a` by means of minimizing the value of a cost function which is the combination of several factors:

- Square sum of `cte` and `epsi`. It could be found [here](./src/mpc.cpp#L50).
- Square sum of the difference actuators to penalize a lot of actuator's actions. It could be found [here](./src/mpc.cpp#L55).
- Square sum of the difference between two consecutive actuator values to penalize sharp changes. It could be found [here](./src/mpc.cpp#L63).

[The weights assigned to each of these factors](./src/mpc.cpp#L40) were tuned manually to obtain a stable and smooth track ride closest to the reference trajectory.

### 2. Timestep Length and Elapsed Duration (N & dt)

I tried several combinations of N and dt for comparison as shown in the below table,

|   N     |   dt     |
| ------- | -------- |
|    25   |   0.01   |
|    10   |   0.05   |
|    10   |   0.1    |

The number of points `N` and the time interval `dt` determine the prediction horizon. The number of points impacts the controller performance as well. I tried to keep the horizon around the same time the waypoints were on the simulator. With too many points the controller starts to run slower, and some times it went wild very easily. After these trials, I finally picked the last combination which contributes a smoother drive.

### 3. Polynomial Fitting and MPC Preprocessing

The waypoints provided by the simulator are transformed to the car coordinate system at [./src/main.cpp](./src/main.cpp#L122). Then a 3-order polynomial is fitted to the transformed waypoints. These polynomial coefficients are used to calculate the `cte` and `epsi` later on. They are used by the solver as well to create a reference trajectory.

### 4. Model Predictive Control with Latency

To handle actuator latency, the state values are calculated using the model and the delay interval. These values are used instead of the initial one. The code implementing that could be found at [./src/main.cpp](./src/main.cpp#L147).
