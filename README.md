# PID Controller
Self-Driving Car Engineer Nanodegree Program

In this project I implemented a PID controller to steer an autonomous driving car around a track in a simulator. The code is written in C++. This work is part of the Self-Driving Car Engineer Nanodegree Program at Udacity.

---

[//]: # (Image References)

[image1]: ./out_images/image2.png

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


## Implementation
 ### Understanding of PID controller:
 
 * The overall control function 
 
 \begin{equation}
 u(t)= kp*e(t) + ki* \int_{0}^{t}e(t')dt' +kd* \frac{de(t)}{dt}
\end{equation}

Where in our project, we take the negative value of the cross track error (CTE) as error "e(t)". The proportional part of the control law make the vehicle turns when it is away from the center of the road. The derivative term provides a better damping so it steers gracefully, while the integral term helps to reduce the steady state error of the system. In this project, we need to tune the parameters kp,kd and ki either manually or automatically.  The following data are what I tuned for this project, which worked well.  And also, I impletemented the PID controller for both steering angle and speed of the car in the simulator.  I chose manual method, following the idea of the twiddle algorithm. 

|      | steering angle | speed |
| ----------- | ----------- |----------- |
| kp   | 0.12        | 0.1     |
| kd   | 1.0       |0.0    |
| ki   | 0.0001     |0.002  |

* Implementation of the above equation

```cpp
void PID::Init(double Kp, double Ki, double Kd) {
	/*Set Kp, Ki and Kd to initial values passed by controller. These are passed from main*/
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	/*Set up inital p, i and d error to zero.*/
	p_error = 0;
	i_error = 0;
	d_error = 0;
}
void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}
double PID::TotalError() {
	return (-(Kp * p_error) - (Ki * i_error) - (Kd * d_error));
}
```
 
* Implementation in the simulator
![alt text][image1]  

* The video sample of this implementation is as follows
Check youtube video [![here](https://youtu.be/OWmCYxaqbnQ)](https://youtu.be/OWmCYxaqbnQ)



## Reflections

In this project, I impleted PID controller for self-driving car, the core part of the code is not difficult to write, but the parameters for the PID controllers were not easy to tune. I have tried many times, and also refer to the resources online, Finally, I was able to find parameters that worked well. 

