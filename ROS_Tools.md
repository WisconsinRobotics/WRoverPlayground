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
Client nodes call the service by sending a request and await the reply.

Both topics and services are defined using messages.
A message is a data structure containing typed fields.
For a service, a request and a response message must be defined.

ROS topics and services will be covered in more detail in the next training modules.

ROS graph model

## ROS Command Line

Open a terminal window and run `roscore`.

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
You should see a window with the image of a turtle in the middle.

In a new terminal, run `rosnode list`.
You should see the following output: 
```
/rosout
/turtlesim
```

Let's find out more about the `/turtlesim` node.
Run `rosnode info /turtlesim`

## ROS Launching

## ROS Parameters

## RQT Tools

## WRoverPlayground Outline

## References

http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes

http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch

http://wiki.ros.org/Nodes

http://wiki.ros.org/Topics

http://wiki.ros.org/Services
