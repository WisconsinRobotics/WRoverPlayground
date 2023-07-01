# WRoverPlayground Planning Doc

This is the planning document for the WRoverPlayground exercise repo. This document is a work in progress and anyone can contribute. 

## Topics
WRoverPlayground needs to encompass the following topics: 

### Ros setup
TBD

### Ros topics
Subscriber:
Mock navigation data which would be formatting like a lidar, or some coordinate system (would need a lot more math under the hood this way)

Robot heading

Robot coordinate(?)

Publisher: 

Left and right motor powesr

### Ros services
Signaling done by mocking status lights

Waiting for some amount of time before going to next beacon

### Ros debugging/tooling
Using rviz to visualize robot and beacon position

Using rqt_plot to monitor topic data

## Steps and instructions

### Step 1: Development environment and ROS setup
Input: Installing a VM software, setting up Linux, and installing ROS and other dependencies OR using docker environment provided by the team and running the code from there

Ouput: Students should be able to pull the student version of the playground code, compile, and run the program. Students should also be able to see the robot and various beacons on Rviz. 

### Step 2: 