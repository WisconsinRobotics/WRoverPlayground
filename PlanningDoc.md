# WRoverPlayground Planning Doc

This is the planning document for the WRoverPlayground exercise repo. This document is a work in progress and anyone can contribute. 

## Topics
WRoverPlayground needs to encompass the following topics: 

### ROS setup
TBD

### ROS topics
Subscriber:
* Mock navigation data which would be formatting like a lidar, or some coordinate system (would need a lot more math under the hood this way)
* Robot heading
* Robot coordinate(?)

Publisher: 
* Left and right motor powesr

### Ros services
* Signaling done by mocking status lights
* Waiting for some amount of time before going to next beacon

### Ros debugging/tooling
* Using rviz to visualize robot and beacon position
* Using rqt_plot to monitor topic data

## Steps and instructions

Ideally each step should take one meeting session, so studenets should be able to complete the playground tutorial by the end of the second week. 

### Step 1: Development environment and ROS setup
Input: Installing a VM software, setting up Linux, and installing ROS and other dependencies OR using docker environment provided by the team and running the code from there.

Ouput: Students should be able to pull the student version of the playground code, compile, and run the program. Students should also be able to see the robot and various beacons on Rviz. Students should also be familiar like concepts including ROS launching and ROS parameters

### Step 2: Moving the robot around
Input: Students will learn about ROS topics, how to subscribe to topics to receive navigation data and publish to topics to set drive data. 

Ouput: Robot should now be able to navigate to and between beacons. Students should understand how ROS topic works, and is comfortable to writing publishers and subscribers in the future. 

Using rqt_plot to visualize topic data should be included of this part as well.

### Step 3: Adding status light and wait
Input: Students will learn about ROS services, setting up input and return values, and calling services from code. To change the status light on rviz, students will also need to learn about whatever object we'll use to represent the robot. 

Ouput: Robot should now be able to change its status light when reaching a beacon, wait a few seconds, then move to the next beacon. Students should be comfortable to write their own custom services in the future. 