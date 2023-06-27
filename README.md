# WRoverPlayground
Welcome to WRoverPlayground! These learning modules will help you become somewhat proficient in using the Robot Operating System (ROS). Each module builds on the previous ones. When you are finished, you will have built a robot named WRunt. This robot will be able to autonomously navigate through a course of beacons from a starting location. *Add a picture of the course for illustrative purposes. A GIF of WRunt completing a course would be even better*

Your software will allow WRunt to detect beacons and navigate between them. The only instructions that can be given to WRunt during operation are to tell it to go to the next beacon in the course. Once a beacon is reached WRunt will let us know that it has arrived by lighting up its status light. Below are more detailed descriptions of how WRunt will be controlled using ROS. Don't worry if this doesn't make sense now! The modules will walk you through this step by step.

## WRunt specifications
#### Drive system
WRunt will have two drive wheels. There will be a ROS topic associated with the drive system. You will publish the desired motor powers to this topic to make the robot move.

#### Target finding sensor
The target finding sensor will tell you where the target is through a ROS topic. You will need to subscribe to this topic to figure out where to drive WRunt.

#### Status light
The status light lights up when WRunt has reached the target. This is a ROS service.

#### Continue to next beacon signal
