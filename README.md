# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
Download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Using the car's localization and should cruise at 50 MPH speed limit in all ways possible, ex: by passing slower traffic. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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

## Implementation Details
There are several key concepts that need to be addressed before diving into the implementation of this project.
1. Frenet Coordinates 
![Cartesian Image](/images/CartesianExample.png)  
Example of Cartesian
![Frenet Image](/images/FrenetExample.png)  
Example of Frenet  
* Because of the complexity involved in defining a path created by vehicles (eg: vehicle going in an 'infiniy' loop), the frenet coordinate system is prefered. A frenet path can be defined by the the longitudinal distance traveled, as well as the lateral distance traveled.
2. Basic Kinematic Equations

Below are the high level implementation on how the path planning algorithm works.
1. Define target objects around the vehicle: (Front Left, Front, Front Right, Left, Right, Rear Left, Rear Right)
   * Front Left, Front, Front Right Objects defined if ahead of ego vehicle and within 30 meters ahead.
   * Rear Left, Rear Right Objects defined if behind of ego vehicle and within 15 meters behind.
   * Left, Right Objects defined if within +-5 meters ahead of the ego vehicle.
2. To keep distance between target object ahead (Automatic Cruise Control), the algorithm simply subtracts 0.224 m/s every 0.02 seconds until the distance ahead is greater than 30 meters.
3. The ego vehicle will perform a lane change if there is front object and when
   * Front Left, Left, Rear Left conditions are met (no objects).
   * Front Right, Right, Rear Right conditions are met (no objects).
4. To make the route created by the path planner smooth, used spline to smooth out the trajectory and interpolate if there are big gaps.
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
