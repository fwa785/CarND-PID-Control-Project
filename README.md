# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Introduction

This project uses PID algorithm to control the steering of the vehicle so the vehicle can stay in the track. 
The source code can be found at [here](https://github.com/fwa785/CarND-PID-Control-Project).

The steering is set to: steering = -Kp * cte - Kd * (cte-prev_cte) - Ki * SUM(cte). There are three components:
* P component for proportional adjustment: -Kp * cte
* I component for integral adjustment: -Ki * SUM(cte)
* D component for differential adjustment: -Kd *(cte-prev_cte)

## P-Component

P-Component is the proportional adjustment of the control. It adjusts the steering 
proportional to the cte(cross track error): -Kp * cte. Kp is the coefficient for the P-Component. 

The bigger Kp and cte, the bigger the P-Component is to correct the cte. If Kp is chosen too small, 
P-component won't be able to reduce cte efficiently. If Kp is chosen too big, the P-component will be 
overshoot make cte pass the target and go to the other direction.

When I adjusted the Kp, as expected, if Kp is too small, the car drifted off the track. If Kp is too big,
the car oscilate a lot on the track and eventually ran off the track. With a right Kp, the car stayed on
the track longer with small oscilation.

## D-Component
D-Component is the differential adjustment of the control. It adjusts the steering
according to the difference of cte and the previous recorded cte: -Kd * (cte - prev_cte). Kd is the 
coefficient for the D-Component.

The D-component will reduce the overshoot caused by the P-Component. When the D-component is chosen well,
the oscillation caused by the P-component will reduce. However, if Kd is too big, it can cause the 
vehicle to oscillate again.

## I-Component
I-Component is the integral adjustment of the control. It adjusts the steering according to the sum of 
the cte: -Ki * SUM(cte). This component is to offset some constant drift of the control, for example, 
if there are wind blowing the vehicle to one direction constantly. For the simulation, there is not 
much constant bias, so this component should be very small. Actually I found zero I-component works 
pretty well.

## Tune the Coefficients Parameters

### Manual Method 
First, I used manual method to find the right coefficients. 

I use the following steps to manually find the coefficients:
* Initialize all the coefficients as zero
* Set the throttle of the vehicle as low as 0.1, so the vehicle runs at low speed
* Increase P-Component coefficient Kp until the vehicle stay on the track most of the time
* Kp is set to 0.15, and the vehicle stay on the track
* Increase the throttle of the vehicle back to 0.3. The vehicle starts to oscillate on the track 
and gets off the track very soon
* Increase the D-Component coefficient Kd until the vehicle doesn't oscillate too much and stays on
the track
* Kd is set to 1.35, and the vehicle stay on the track
* Further increase the throttle of the vehicle to 0.5. The vehicle starts to oscillate a lot again
* Increase the I-Component coefficient Ki to make the oscillation damp quicker. The Ki can't be too
big, because there is not much system bias to correct. It's also not very obvious the Ki makes too much
difference
* Eventually Ki is set to 0.002. The oscillation of the vehicle still is pretty big, but the vehicle stays
on the track most of the time

### Twiddle
Then, I set the throttle back to 0.3, and used twiddle algorithm from the class to find tune the 
coefficients. The coefficients are initialized to the parameters I found manually: [0.15, 0.002, 1.35] so 
twiddle algorithm can converge quicker.

The algorithm implemented as below in C++. It basically convert the python code from class to C++.


          if (dp[0] + dp[1] + dp[2] > TOL) {
            iteration++;
            if (iteration >= TWIDDLE_ITERATIONS) {
              err += cte * cte;
            }
            if (iteration == TWIDDLE_ITERATIONS * 2) {
              err = err / TWIDDLE_ITERATIONS;
              if (best_err < 0) { // the best_err has not setup yet
                best_err = err;
                index = 0;
                state = 0;
              }
              else {
                if (state == 0) {
                  if (err < best_err) {
                    dp[index] *= 1.1;
                    best_err = err;
                  }
                  else {
                    p[index] -= 2 * dp[index];
                    state = 1;
                  }
                }
                else {
                  if (err < best_err) {
                    best_err = err;
                  }
                  else {
                    p[index] += dp[index];
                    dp[index] *= 0.9;
                  }
                  state = 0;
                }

                if (state == 0) {
                  index = (index + 1) % 3;
                }
              }

              std::cout << "parameters: " << p[0] << "," << p[1] << "," << p[2] << std::endl;
              std::cout << "dps: " << dp[0] << "," << dp[1] << "," << dp[2] << std::endl;
              std::cout << "best_err: " << best_err << std::endl;
              std::cout << "index: " << index << " state: " << state << std::endl;

              msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              iteration = 0;
              err = 0;
              if (state == 0) {
                p[index] += dp[index];
              }
              pid.Init(p[0], p[1], p[2]);
            }
          }

With twiddle algorithm, the coefficients are selected as: [0.2, 0.0001, 1.45548]. Visually
there is not much difference I could tell from the coefficients I chose manually. In fact, I feel
the vehicle oscillates more so I reduced the Kp back to 1.5 as my final choice.

## Result
With [Kp, Ki, Kd] set to [1.5, 0.0001, 1.45548] and the throttle set to 0.3. The vehicle runs pretty well on 
the track.

[![IMAGE Final Result](https://img.youtube.com/vi/kVNcX8B06UY/0.jpg)](https://www.youtube.com/watch?v=kVNcX8B06UY&feature=youtu.be)

## PID Control for Throttling
When the speed increases, the same coefficient parameters don't work well. When throttling is set
to 0.5, with the same coefficient paramters the vehicle oscillates a lot, although stays on the
track mostly.

I tried to use PID to control the throttling. The idea is when the steering is high, the speed
should be lower so the vehicle doesn't make turns too sharp and loss control. So I feed the 
absolute value of the steering to replace the cte to the PID for throttling. The throttling is 
set to MAX_THROTTLE - abs(the PID controll value). To prevent a nagative throttling, I set a 
MIN_THROTTLE value so the throttling value won't be less than MIN_THROTTLE.


          throttle_pid.UpdateError(fabs(steer_value));
          throttle_value = MIN_THROTTLE;
          if (MAX_THROTTLE - fabs(throttle_pid.TotalError()) > throttle_value) {
            throttle_value = MAX_THROTTLE - fabs(throttle_pid.TotalError());
          }

I used the twiddle algorithm to find the Kp, Ki and Kd for the throttling PID. With the PID 
throttling control, the vehicle stays a little better on the track on higher maximum 
throttling, but the improvement is limited. So by default, I still set the throttling to the
constant value 0.3.

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
4. Run it: `./pid`. 
