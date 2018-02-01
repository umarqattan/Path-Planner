# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

# Write Up

## GOAL

The goal of this project is to design a path planner for a Self Driving Car so that it can maneuver a 3 lane highway in mild traffic conditions. Given the constraints of the highway such as the speed limit, the number of lanes, and the speeds and positions of neighboring vehicles, the path planner enables the Self Driving Car to move without experiencing an accident.

## Project Breakdown

The Path Planner is designed with three major components. First is vehicle prediction, second is vehicle planning, and third is trajectory generation.

### Prediction

First, the Path Planner takes in sensor fusion data, which contains positions of other vehicles on the road. Given the other vehicles' positions, the Path Planner may check how heavy the surrounding traffic is (e.g. if there's a vehicle that's in the left lane, right lane, or in front of the current vehicle). Not only does the Path Planner check for traffic, but also it determines whether it is safe to change lanes, slow down, or speed up depending on the Self Driving Car's current speed and the length of the previous path. To make sure accidents do not occur due to sharp increases/decreases in speed or premature lane changes, a buffer of space between different cars on the road is defined. Below is some code from `main.cpp` that illustrates this. (main.cpp lines 283-297).

```
 for(int i = 0; i < sensor_fusion.size(); i++) {

                        
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double neighbor_car_s = sensor_fusion[i][5];
    float d = sensor_fusion[i][6];
    double neighbor_car_speed = sqrt(vx*vx*vy*vy);
    neighbor_car_s += ((double)previous_path_size*.02*neighbor_car_speed);

    bool is_neighbor_in_front = neighbor_car_s > car_s;
    bool is_there_front_buffer = neighbor_car_s - car_s < front_thresh;
    bool is_there_back_buffer = neighbor_car_s - car_s < -back_thresh;
    bool is_there_lane_buffer = (is_neighbor_in_front && is_there_front_buffer) || is_there_back_buffer;

    int car_lane = floor(d/4.0);

            
```

### Planning

The second major component to the Path Planner is how the Planner decides what maneuver to make (e.g. left lane change, right lane change, speed up, slow down, remain the same). To determine what maneuver to make, the Path Planner checks if there is a buffer between the Self Driving Car and other cars in its neighborhood. The buffer is set to be 30 meters in front and 10 meters behind. What the Planner does in the code to follow is check if there is a buffer between itself and its neighbors; if so, it then checks if a neighboring car is in the same lane, in the left lane, or in the right lane. Depending on where the neighboring car is, that's the lane not to change into. That is, the variable that checks whether or not to change into a particular lane is set to false to indicate that it's unsafe to change into that lane. The same holds for the other lane variables. (main.cpp lines 301-327).

```
  if (is_there_lane_buffer) {

        bool is_neighbor_in_same_lane = (car_lane == lane && is_neighbor_in_front);
        bool is_neighbor_in_left_lane = (car_lane == lane - 1);
        bool is_neighbor_in_right_lane = (car_lane == lane + 1);
        
        if (is_neighbor_in_same_lane) {

            near_front_vehicle = true;
            cout << "The Vehicle is getting closer to the front Vehicle in lane " << lane << endl;
        }

        
        else if (is_neighbor_in_left_lane) {

            left_lane_change = false;
            cout << "The Vehicle will not change to left lane " << car_lane << endl;

        }

        else if (is_neighbor_in_right_lane) {

            right_lane_change = false;
            cout << "The Vehicle will not change to right lane " << car_lane << endl;
        }
    }
  }

```

After setting the flags for the safety of a potential lane change, another check is made to either decrease the Self Driving Car's velocity in the event that it's closing in on a neighboring vehicle (left or right lanes) or increase the velocity if the car has a large enough buffer and is not yet at the highway's maximum speed limit. (main.cpp lines 329-353)

```
if (near_front_vehicle) {

    reference_velocity -= 0.22;

    if (left_lane_change) {

        lane -= 1;
        cout << "Changing to left lane" << endl;

    }

    else if (right_lane_change) {

        lane += 1;
        cout << "Changing to right lane" << endl;

    }

}
else if (reference_velocity < 50) 
{

    reference_velocity += 0.22;

}

``` 

### Trajectory Generation

The last major component of the Path Planner is generating a trajectory for the Self Driving Car based on the previous two steps (Prediction and Planning). What the Path Planner does in this stage is to create three points through which to generate a spline. These three points are determined by the transformed positions of the car's s and d values and its map waypoints. The spacing between the waypoints range from 0 to 90 meters.

### Results, Images, and Conclusion

There were cases where the car would experience a collision based on the thresholds
![result_1]()

![result_2]()


Changing the thresholds from values between 10 and 50 helped the car better gauge its environment to avoid collisions. The back_threshold was set to 10 and the front_threshold was set to 50. To improve upon this project, it is necessary to add more checks to how the car plans to make a maneuver based on the neighboring cars (e.g. consider the future speed of the neighboring cars, whether the car is currently changing into a lane where a neighboring car may be changing into, or blind spot checks by varying the front and back thresholds dynamically).





