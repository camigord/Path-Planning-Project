# Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

---
<img src="./assets/video1.gif?raw=true" width="320"><img src="./assets/video2.gif?raw=true" width="320">

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

---

## Dependencies

* cmake >= 3.5
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
---

## Basic Build Instructions

1. Clone this repo.
2. Execute `./clean.sh`
3. Execute `./build.sh`
4. Execute `./run.sh`

---

## Project Rubric Criteria

#### 1. The car is able to drive at least 4.32 miles without incident.

  The first video above shows that the vehicle has been driving without incidents for more than 7 miles. The details to achieve this performance will be described below.

#### 2. The car drives according to the speed limit.

  In line 210 of _main.cpp_ we define `target_velocity = 49.5 m/s`. This will be the maximum allowed velocity for the vehicle and will take care of fulfilling this criteria. The logic behind the state machine will take care of encouraging the vehicle to drive as fast as possible without breaking the speed limit or colliding with other vehicles.

#### 3. Max Acceleration and Jerk are not Exceeded.

  We take care of constraining acceleration and jerk in lines 272 - 277 from _main.cpp_. Regardless of the velocity suggested by the planner, we will only increase/decrease the vehicle's velocity (`ref_vel`) by small steps:
  ```
  if(suggested_velocity > ref_vel && ref_vel < target_velocity){
    ref_vel += 0.224;       // Equivalent to 5 m/s^2
  }
  else if(suggested_velocity < ref_vel){
    ref_vel -= 0.224;
  }
  ```

#### 4. Car does not have collisions.

  Collisions are avoided by carefully analyzing the `sensor_fusion` information. A lane change is only allowed if there are no vehicles within certain range in the intended lane. Similarly, when keeping a lane our velocity is limited by the velocity of the vehicle directly ahead of us.

#### 5. The car stays in its lane, except for the time between changing lanes.

  I based a lot of my code on the suggestions presented on the project walkthrough where the trajectory of our vehicle is computed using the `spline.h` library. Lines 323-325 in _main.cpp_ add three 30m spaced points ahead of our location using Frenet space. The distance _d_ of each one of these points is computed considering the _lane_ intended by our path planner. By doing this, we can just use our current spline to interpolate our trajectory points in lanes 365-384 (_mapin.cpp_).

  Whenever our planner outputs a lane change, the provided code will take care of generating a trajectory which will take us to that lane.

#### 6. The car is able to change lanes.

  As described above, the car will be able to change lanes whenever our path planner so desires. The path planner will output the intended lane (lanes 266-267 _main.cpp_) and the trajectory generation will take care of taking us there. The logic behind the path planner is described below.

#### 7. How to generate paths.

  The path planner logic is defined in the function `choose_next_state()` of our _Vehicle_ class and can be briefly summarized as follows:

  - We obtain the possible successor states given our state machine logic by calling the function `successor_states()` (lines 111-134 in _vehicle.cpp_)

  - For each of these possible states, we obtain a set of parameters (intended lane and expected velocity) by calling the function `generate_trajectory()` (136-149 in _vehicle.cpp_)

  - `generate_trajectory()` will take care of verifying the feasibility and safety of each state and will only allow a lane change if it is safe to do it.

  - We will compute a cost value for each one of the possible states (line 58 in _vehicle.cpp_) by calling the function `calculate_cost()` (85-108 in _vehicle.cpp_).

  - `calculate_cost()` will assign a lower cost to those states with result in an overall higher velocity by analyzing both the expected velocity in our current lane and the expected velocity in the intended lane. If a lane change is required, the cost function will assign a lower cost to that lane which will give us more empty room to move around (line 98 in _vehicle.cpp_).

  - The parameters (intended lane and expected velocity) of the best state (minimum cost) are then returned to _main.cpp_ where the rest of our code will take care of adjusting a proper trajectory for the simulator to execute.

---

## Simulator.
You can download the simulator which contains the Path Planning Project from [here](https://github.com/udacity/self-driving-car-sim/releases).

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

 - ["x"] The car's x position in map coordinates
 - ["y"] The car's y position in map coordinates
 - ["s"] The car's s position in Frenet coordinates
 - ["d"] The car's d position in Frenet coordinates
 - ["yaw"] The car's yaw angle in the map
 - ["speed"] The car's speed in MPH

#### Previous path data given to the Planner

Returns the previous list but with processed points removed

  - ["previous_path_x"] The previous list of x points previously given to the simulator
  - ["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

  - ["end_path_s"] The previous list's last point's frenet s value
  - ["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

  - ["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.
