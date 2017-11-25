# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Introduction
In this project I implemented a PID controller in c++ in order to drive a car autonomously
around a track with term 2 simulator.

For optimization of the P,I,D coefficients I used a custom version of twiddle algorithm for the simulator.

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

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid` or `./pid twiddle`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Reflection

* P - Proportional

  The proportional gain determines the proportion of the output response compared to the error.
  The bigger the value is, the bigger the output response is, the faster the system will minimize the cross track error
  and go back to the desirable value(cte = 0, center of track), though larger values will make the car oscillate 
  out of control and crash.
  This is the most important parameter, the one that contributes the most.
  If it's the only parameter set, the car will oscillate from one side to the other since 
  the car position will not be parallel when it reaches CTE of 0, so it will continue a bit 
  to the other side before starting to get back and so on.
  I was not surprised at all by the effect of the proportional controller.

* I - Integral

  The integral gain determines the proportion of the output response compared to the error magnitude and duration.
  The integral component sums up all the errors from the past, all errors accumulate, so even a slight error can get big over time.
  The controller helps to make sure the system is steady for the long run, as it tries to minimize the long term error to zero.
  It helps the P controller to converge faster towards the goal and it can mask noise or inherent bias in the system.
  In our simulator the best gain is 0 since we don't have much noise or bias.
  If running with a different gain, you can see that the i_error is going up sharply,
  then down sharply and again, basically it's overshooting and the car oscillates badly until it crashes.
  I didn't expect the big impact that a small change in integral gain would have on the car.

* D - Derivative 

  The derivative gain determines the proportion of the output response compared to the rate of change of the error.
  The beauty of it is that it basically trying to minimize CTE based on "future predictions" since it's measuring the rate of change.
  It acts as a stabilizer and helper for the other controllers, helps to converge faster, more accurate and reduce the oscillations.
  Derivative gain is very sensitive to noise.
  If only this parameter is set, you will see the car driving on the side of the road, always turns at the last minute, until it misses and crashes.
  The derivative gain stabilized the car pretty good as I expected.
  
The final parameters were adjusted with a custom made version of twiddle algorithm that works with the simulator.
On my laptop, it took many hours to get the values I have now.
Please run the programs like this if you want to test it: `./pid twiddle`