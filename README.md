# WRoverPlayground
Welcome to WRoverPlayground! These learning modules will help you become somewhat proficient in using the Robot Operating System (ROS). Each module builds on the previous ones. When you are finished, you will have built software for a robot named WRunt. This robot will be able to autonomously navigate through a course of infrared (IR) beacons from a starting location. *Add a picture of the course for illustrative purposes. A GIF of WRunt completing a course would be even better*

Your software will allow WRunt to detect beacons and navigate between them. The only instructions that can be given to WRunt during operation are to tell it to go to the next beacon in the course. Once a beacon is reached WRunt will let us know that it has arrived by lighting up its status light. Below are more detailed descriptions of how WRunt will be controlled using ROS. Don't worry if this doesn't make sense now! The modules will walk you through this step by step.

## WRunt specifications
#### Drive system
WRunt will have two drive wheels. The drive system used is called a tank drive. This means that each side of the robot is controlled independently. When both sides are commanded to go forward, the robot will move forward. When one side is commanded forward and the other backward, the robot spins. You can get creative with this and put one side at a slightly lower power to move the robot on a curve.

#### Infrared (IR) sensor
The IR sensor detect IR radiation from the closest beacon and gives you the heading of the closest beacon relative to the robot. You'll need to use this information to figure out where to drive WRunt.

#### Status light
The status light lights up when WRunt has reached the target. This allows observers to know when WRunt knows it has reached its target. During a competition setting, this lets the judges know that the robot actually completed the task and did not just accidentally get to the right spot. 

#### Continue to next beacon signal
Once WRunt has reached the first beacon, you will need to give it a manual signal that it can move on to the next beacon. In real life, this signal would be transmitted over a network connection. For our pursposes, it will be a button on a Graphical User Interface (GUI).

## WRunt behavior flowchart
1. WRunt determines the closest beacon that hasn't been visited yet
2. WRunt move to and reaches the target beacon 
3. WRunt marks the beacon as visited, changes its own status light, and awaits a certain number of seconds
4. Repeat steps 1-3 until all beacons have been reached

## How To Open The Project

This project uses Docker to efficiently containerize the workspace and make cross-platform development easier.  Find your host platform below to see how to start development:

### Linux

### Windows

### Mac
TODO (@Tzanccc): how to do this on mac