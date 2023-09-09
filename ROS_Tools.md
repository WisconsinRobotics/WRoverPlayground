# WRoverPlayground - ROS Tools

## ROS Concepts Crash Course

ROS nodes are processes that use ROS to communicate with other nodes.
ROS nodes communicate with each other using streaming topics, RPC services, and the Parameter Server.

ROS topics are named interfaces with a publisher and subscriber model.
Publisher nodes generate data and publish that data to a topic.
Subscriber nodes subscribe to a topic to use data that is published to a topic.
A topic can have multiple publisher and subscriber nodes.
Topics are intended for unidirectional communication.
If a response is expected from a request, a service should be used instead.

ROS services allow nodes to return data to other nodes.
Server nodes define the service request and response.
Client nodes call the service by sending a request and await the response.

Both topics and services are defined using messages.
A message is a data structure containing typed fields.
For a service, a request and a response message must be defined.

ROS topics and services will be covered in more detail in the next training modules.

The final method of communication between ROS nodes is the Parameter Service.
This is essentially a shared dictionary that can be used to store and retrieve variables at runtime.
It is usually used for configuration since it is not designed for high performance.

## ROS Command Line

### Starting nodes

Open a terminal window and run `roscore`.
This starts the ROS master node.

Open a new terminal and run `rosnode list`.
You should see a single node: `/rosout`.

Run `rosnode info /rosout`.
You should see the following output: 
```
Node [/rosout]
Publications: 
 * /rosout_agg [rosgraph_msgs/Log]

Subscriptions: 
 * /rosout [unknown type]

Services: 
 * /rosout/get_loggers
 * /rosout/set_logger_level
```
This shows that the `/rosout` node publishes to the `/rosout_agg` topic and subscribes sto the `/rosout` topic.
Also, the `/rosout` node provides the `/rosout/get_loggers` and `/rosout/set_logger_level` services.

Now, let's start another node.
Run `rosrun turtlesim turtlesim_node`.
You should see a GUI window with a turtle in the middle.

![TurtleSim GUI](/images/turtlesim_1.png)

In a new terminal, run `rosnode list`.
You should see the following output: 
```
/rosout
/turtlesim
```

Let's find out more about the `/turtlesim` node.
Run `rosnode info /turtlesim`.
You should see the following output:
```
Node [/turtlesim]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /turtle1/color_sensor [turtlesim/Color]
 * /turtle1/pose [turtlesim/Pose]

Subscriptions: 
 * /turtle1/cmd_vel [unknown type]

Services: 
 * /clear
 * /kill
 * /reset
 * /spawn
 * /turtle1/set_pen
 * /turtle1/teleport_absolute
 * /turtle1/teleport_relative
 * /turtlesim/get_loggers
 * /turtlesim/set_logger_level
```

### Interacting with Topics

Let's see what messages are being published to the `/turtle1/pose` topic.
We can do this by running `rostopic echo /turtle1/pose`.
The terminal should be repeatedly outputting the following:
```
x: 5.544444561004639
y: 5.544444561004639
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
---
```

Let's find out more about the message that is being published.
From the `rosnode info /turtlesim` output, we know that the message type of the `/turtle1/pose` topic is `turtlesim/Pose`.

Run `rosmsg show turtlesim/Pose`.
You should see the following output:
```
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
```

Now, let's publish to a topic.
Run the command `rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'`.
The turtle in the GUI window should move to the right.

![TurtleSim GUI](/images/turtlesim_2.png)

Let's find out more about the `geometry_msgs/Twist` message we just published.
Run `rosmsg show geometry_msgs/Twist`.
You should see the following output:
```
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

We can also publish to topics at a specified rate.
Run `rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'`.
The turtle should be moving in a circle.

![TurtleSim GUI](/images/turtlesim_3.png)

### Interacting with services

Let's call the `/turtle1/teleport_absolute` service.

First, we should find out the service's request and response message types.
Run `rosservice type /turtle1/teleport_absolute`.
This should output `turtlesim/TeleportAbsolute`.
Now, run `rossrv show turtlesim/TeleportAbsolute`.
This should output the following:
```
float32 x
float32 y
float32 theta
---
```
The `---` separates the request message and the response message.
This means the service takes 3 float32 values as the request message and returns an empty response message.

Run `rosservice call /turtle1/teleport_absolute 0.0 0.0 0.0`.
The turtle should be moved to the bottom left of the GUI window.

![TurtleSim GUI](/images/turtlesim_4.png)

### ROS Parameters

We can see what parameters are currently on the Parameter Server by running `rosparam list`.
This should output the following:
```
/rosdistro
/roslaunch/uris/host_ros_desktop__33381
/rosversion
/run_id
/turtlesim/background_b
/turtlesim/background_g
/turtlesim/background_r
```

Let's get the value of `/turtlesim/background_b` parameter.
Run `rosparam get /turtlesim/background_b`.
This should output `255`.

Now, let's set the value of the `/turtlesim/background_b` parameter to 0.
Run `rosparam set /turtlesim/background_b 0`.
Then, we call the `/clear` service to tell `turtlesim_node` to redraw the background.
Run `rosservice call /clear`.
The background should now be a shade of green.

![TurtleSim GUI](/images/turtlesim_5.png)

## RQT Tools

## ROS Launching

## WRoverPlayground Outline

## References

http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes

http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams

http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch

http://wiki.ros.org/Nodes

http://wiki.ros.org/Topics

http://wiki.ros.org/Services

http://wiki.ros.org/Parameter%20Server
