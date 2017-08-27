# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Project Reflection
I created a planner class that decides how to plan the course of the vehicle we are driving. It consists of two parts, the General Path Creation part and the Collision Avoidance part. Although it is the latter that feeds the former, I am going to go review them in reverse for better understanding. This is then followed by a link to the video.

### General Path Creation
1. If there are previous points in the list, add them all back to the current list of points we have to traverse.
2. For the remaining points (We can have up to 50)
  - Generate 3 spline points based on the reference point of the vehicle. If there are no points, use the current position of the vehicle. If there are previous points, use the last position amongst them
  - These spline points are generated at distances of 30, 60 and 90 m away from the reference point of the vehicle.
  - Convert the spline points to xy points in the space of the car and sort them in ascending order of X
  - Generate x & y points based on the target_x (30 m) and target_y (from spline) distances for time intervals of TIME_INTERVAL(0.02 s) at the targetSpeed (varies)
  - Convert these points back to world space and add them to the list of points

### Collision Avoidance
1. Make two lists
  - One that tells us whether lanes are safe to enter (laneSafety) - array<bool, 3>
  - Another that tells us how far the next car is in that lane (laneDistance) - array<double, 3>
2. For Each Vehicle in the Sensor Fusion Data:
  - Calculate it's projected distance and current distance from the controlled vehicle.
  - If this vehicle is in the current lane :
    - If it falls within a certain threshold between 0m and MIN_DIST(30 m) then mark a flag slowDown. 
  - Else
    - If it is less than MIN_BACK_DIST (10 m) behind our main vehicle and less than MIN_FORWARD_DIST (50m) ahead of both the projected and current distance then mark this lane as unsafe
    - Set the distance for the lane
    - In some cases s and d might fail, so fallback to x-y distance in order to avoid such pit falls.
3. If we are slowing down and not changing lanes yet :
  - Check the left lane and right lanes for safety and choose the one where the next car is the farthest out.
  - If we are NO_CHANGE_DISTANCE (10 m) behind the vehicle in front of us in the lane then brake hard (0.3 mph reduction in our target speed).
  - Else if we are SLOW_DOWN_DIST (30 m) behind the vehicle in front of us in the lane then brake gently (0.1 mph reduction in our current speed).
  - If we have an order to change lanes then set the change lanes flag and calculate the delta shift (changeD).
4. Else if we are slowing down and changing lanes :
  - If we are SLOW_DOWN_DIST (5m) away from a vehicle in our current lane then hit the brakes with HEAVY_BRAKE_AMOUNT (0.2 mph).
5. Else if target speed is less than max velocity, increase velocity with HEAVY_ACCEL_AMOUNT(0.2 mph).
6. Set the targetD and targetSpeed values based on the decisions taken above and send them to the path creation module.

### Runtime Video
Please find the video at [https://youtu.be/4IRv2u6lKxI](https://youtu.be/4IRv2u6lKxI).


### Simulator. 
You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

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
