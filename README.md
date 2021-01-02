# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[p_gain]: ./images/p_tuning.png "Proportional Gain Tuning"
[i_gain]: ./images/i_tuning.png "Integral Gain Tuning"
[d_gain]: ./images/d_tuning.png "Derivative Gain Tuning"

## Goal

The goal of the PID control project is to implement a PID controller to set the steering angle of the car. The controller must be able to safely navigate the car around the track and obey traffic laws such as driving within the lane lines.

## Reflection

The PID controller is a type of control algorithm that calculates the error between a process variable setpoint, such as the desired vehicle position within a lane, and the measures process variable, such as the actual position. The output of the PID controller specifies a controlled variable, such as the steering angle of the vehicle. There are three key components to a PID controller, which git it its name, the Proportional control element, the Integral control element, and the Derivative control element. The effect of these componenets of the controller are as follows:

---

#### Proportional Element

The proportional component produces an output that is directly proportional to the current error between the process variable and the setpoint. The proportional gain `Kp` can be tuned to control how quickly the controller responds to the error, however as this gain is increased the controller overshoots the setpoint. A large `Kp` will produce an oscillating output and even an unstable controller that can't converge on the setpoint. The implementation of the proportional component is simply multiplying `Kp` by the CTE to produce its component of the controller output.

![alt text][p_gain]

---

#### Integral Element

The integral component is responsible for producing an output that's proportional to both the magnitude of the error between the process variable and the setpoint as well as the duration of the error. In other words, as the error accumulates over time, the integral component will contribute more and more to the controller output value to drive the error towards zero. The integral gain `Ki` can also be tuned to control how quickly the controller responds by amplifying accumulation of the error. A large integral gain will push the controller to react quickly to sustained error, however it can also produce an oscillating or unstable controller when this gain is too large. The implementation of the integral component is done by summing the CTE with itself over each timestep and multiplying this by `Ki`.

![alt text][i_gain]

---

#### Derivative Element

The derivative component is responsible for giving the controller a predictive quality by calculating the slope of the error over time. A rapidly changing error between the process variable and the setpoint, such as a sudden deviation between the two, will provoke the derivative component to drive the controller output towards the direction of a smaller change in error and presumably closer to the setpoint. The derivative element of the PID controller can be tuned by adjusting the derivative gain `Kd` to increase or decrease the controllers sensitivity changes in error. The implementation of the derivative component is done by subtracting the current CTE from the CTE of the previous timestep to calculate the change in CTE. This value is then multiplied by `Kd` to complete the derivative component of the controller output.

![alt text][d_gain]

---

#### PID Gain Tuning

The final PID gain values were selected by manual trial and error to produce smooth steering control and a minimal cross track error.
- Kp = 0.1
- Ki = 0.005
- Kd = 3.0

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)