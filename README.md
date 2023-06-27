# WRoverPlayground
A ROS simulation of a simple drivetrain

## WRunt specifications
- WRunt will have two drive wheels. Each wheel's motor will have a ROS topic associated with it. You will publish the desired motor power to this topic to make the robot move.
- The target finding sensor will tell you where the target is through a ROS topic. You will need to subscribe to this topic to figure out how to drive the robot.
- The status light lights up when WRunt has reached the target. This is a ROS service.
