# WRoverPlayground - Topic Training
Now that you have completed the Tools Training, you are ready to learn about ROS topics. From the Setup Training, you should be familiar with:
- the basics of ROS Nodes
- `rosnode`
- `roscore`
- `rqt_graph`
- `rqt_plot`.

Once you complete this training, you should be familiar with:
- the purpose of topics
- the relationship between Publishers and Subscribers
- setting up Publishers and Subscribers
- running Publishers and Subscribers
- custom topic types
- and `rostopic type {topic}`.

## The Purpose of Topics
Though possible, it is not feasible to have one node contain all the information a rover would need. We need to spread out the information into different files. This is where ROS topics come in. The word "topic" sounds very general, but it is actually a specific concept that allows ROS nodes to communicate with each other. They are a way to stream data in and out of a program.

When discussing topics, nodes are divided into those that send data on a topic and those that receive data on a topic. This allows us to specialize nodes, so they can execute their own code/tasks, while also sending and receiving information. For example, we separate the motor driver programs from how we drive the motors from how we gather user input. Despite these all being related to the motors, they are separated for easier maintenance, division of tasks, and readability.

This division of nodes means that our code now becomes 'hot-swappable.' This means that the software can be changed without the entire system being shut down Since the nodes are separated, they can be added, replaced, or removed without affecting other parts of the system.

Also, topics do not care which node is sending or receiving data, so we can easily create a node that serves data from a physical sensor as well as a mock node to test other nodes without needing to change those nodes. The reason is because the interfaces provided through the published data versus an actual sensor are equivalent.

## Publisher/Subscriber Relationship
The Publisher and Subscriber are both nodes that are connected by a topic, allowing them to send messages between each other. The Publisher node publishes messages on the topic, while the Subscriber node subscribes to the same topic to receive the Publisher's messages. 

Topics do not need to have a one-to-one relationship. For example, it is possible for a topic to have multiple Subscribers, which allows a single node to communicate with other nodes.

Also, Publishers and Subscribers do not need to be in separate files. In fact, if the Publisher and Subscriber rely on each other's data, it may be better to keep them in the same file. However, a node should not publish to itself as this would be incredibly inefficient. A way to do this properly would be to have the Subscriber receive data from one topic, the data to be manipulated in some way, and the Publisher to send the result on another topic. 

An example of a Publisher and Subscriber residing in the same file will be provided in the next section.

## Setting up a Publisher and Subscriber
Within the directory `/workspaces/WRoverPlayground/src/topic_training/src`, you will find two files called Demo(.cpp or .py). These files will show a basic class containing both a Publisher and Subscriber. There will be comments throughout the files that explain the purpose of specific sections of the code.

Note that there are a few sections of code that are marked as "Optional." You do not need to include everything in the examples when creating your own code. They are simply showing one way of setting up a Publisher and Subscriber in the same file.

One file will be written in C++, and the other will be in Python. It is recommended that you choose the language you are most comfortable with to complete the training. After you are done with all the trainings in `WRoverPlayground`, you should discuss with one of the Software Leads to determine which part of the rover you would like to be assigned to. You will be given small tasks first, working your way up to more difficult projects. If these tasks/projects require a different language, then you can look through this training again or take what you know from the first time and learn as you go.

Conintue reading here and go into one of the Demo files based on the language you chose. Make sure you understand what is being said. The general idea of the Demo (.cpp or .py) file is that it takes in data from a separate Publisher file (PlotDemo.cpp or .py) as a float on the topic `plot` and sends that data in the form of a String on the topic `chatter`.

### C++

A basic setup for ROS topics in C++ would look something like this:

``` C++
ros::init(argc, argv, "Demo");
ros::NodeHandle nhandle;
ros::Subscriber sub = nhandle.subscribe("plot", QUEUE_SIZE, chatterCallback);
ros::Publisher pub = nhandle.advertise<std_msgs::String>("chatter", QUEUE_SIZE);
```

To break this down,

``` C++
ros::init(argc, argv, "Demo");
```
Initialize ROS and specify the name of the node (which is the name of the file in this case). This requires the parameters from the main function header (`argc` and `argv`).

``` C++
ros::NodeHandle nhandle;
```
Create a handle for the node. This will allow us to create Subscriber and Publisher objects, which you will see in the next couple lines.

``` C++
ros::Subscriber sub = nhandle.subscribe("plot", QUEUE_SIZE, chatterCallback);
```
Initialize the Subscriber. The `subscribe()` function is how we tell ROS we want to receive messages on a specific topic. State the name of the topic (`plot`), the queue size (1), and the function that will be called when a message is received (`chatterCallBack(msg)`). The queue size is the maximum number of messages kept as a buffer before throwing away old ones if we are publishing too quickly.

``` C++
ros::Publisher pub = nhandle.advertise<std_msgs::String>("chatter", QUEUE_SIZE);
```
Initialize the Publisher. The `advertise()` function creates a Publisher object that we can use to publish messages to a topic later on. State the message type we will be publishing (`std_msgs::String`), the name of the topic (`chatter`), and the queue size (1).

The callback function for a Subscriber looks something like this:

``` C++
void chatterCallback(const std_msgs::Float64::ConstPtr& msg)
{
   pi = msg->data;
}
```
This function will get called when a new message has arrived on the topic `plot`. The message received by the Subscriber, sent by the Publisher, is denoted as `msg`.

Publishing a message on a topic would look something like this:

``` C++
std_msgs::String msg;
msg.data = sstream.str();
pub.publish(msg);
ros::spinOnce();
```

To break this down,

``` C++
std_msgs::String msg;
```
Initialize `msg` as the desired type, `std_msgs::String`.

``` C++
msg.data = sstream.str();
```
Set the data of `msg` as what you want the message to send. In this case, `sstream.str()` is a String concatenated with a float value.

``` C++
pub.publish(msg);
```
Publish the message using the previously initialized Publisher `pub`.

``` C++
ros::spinOnce();
```
There is also the function `ros::spin()`. The main difference is that `ros::spinOnce()` is used when other lines of code, and processing arriving messages, need to be executed along side each other, such as in a while loop. Whereas `ros::spin()` should be called at the end of a file.

### Python

A basic setup for ROS topics in Python would look something like this:

``` Python
rospy.init_node('Demo', anonymous=True)
rospy.Subscriber('plot', std_msgs.Float64, callback)
pub = rospy.Publisher('chatter', std_msgs.String, queue_size=QUEUE_SIZE)
```

To break this down,

``` Python
rospy.init_node('Demo', anonymous=True)
```
Initialize ROS and specify the name of the node (which is the name of the file in this case).

``` Python
rospy.Subscriber('plot', std_msgs.Float64, callback)
```
Initialize the Subscriber. State the name of the topic (`plot`), the message type we will be publishing (`std_msgs.Float64`), and the function that will be called when a message is received (`callBack(data)`).

``` Python
pub = rospy.Publisher('chatter', std_msgs.String, queue_size=QUEUE_SIZE)
```
Initialize the Publisher. State the name of the topic (`chatter`), the message type we will be publishing (`std_msgs.String`), and the queue size (1). The queue size is the maximum number of messages kept as a buffer before throwing away old ones if we are publishing too quickly.

The callback function for a Subscriber looks something like this:

``` Python
def callback(data: std_msgs.Float64) -> None:
    global pi 
    pi = data.data
```
This function will get called when a new message has arrived on the topic `plot`. The message received by the Subscriber, sent by the Publisher, is denoted as `msg`.

Publishing a message on a topic would look something like this:

``` Python
pi_msg = "currently at %s" % pi
pub.publish(pi_msg)
```

To break this down,

``` Python
pi_msg = "currently at %s" % pi
```
Create the message.

``` Python
pub.publish(pi_msg)
```
Publish the message using the previously initialized Publisher `pub`.

If you have glanced at the C++ example code, you may have noticed the function `ros::spinOnce()` and its differences with `ros::spin()`. Python only has `rospy.spin()`, which can be completely replaced by a for loop, which is why you do not see it in the Python example code.

## Running a Publisher and Subscriber
Before you are able to run the example code, find the CMakeLists.txt file in the directory `/workspaces/WRoverPlayground/src/topic_training`. This file will include the the configurations, builds, and installs required to run your program as well as the files you will be running. Do not change anything in the file as it should already be set up for the demo files to work. This file will be covered in the Setup Training. You can also check out the package.xml file in the same directory, which should already be set up as well.

Now, we will actually run the program.

1. In the terminal, run the command `roscore`. 

2. In a new terminal window, change your current directory to be the catkin workspace. 
In this case, use `cd /workspaces/WRoverPlayground`. 

3. Use the commands `catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` and `source devel/setup.bash` in this order. 
These commands adjust the catkin workspace based on the CMakeLists.txt and package.xml files. You must run these commands everytime you change these two files. 

*Note: This year we decided to start using a Docker container, which should take care of step #3 automatically. It will not break anything if you run these commands, but they are redundant. The training still includes this step so that you know what is happening behind the scenes.*

4. Run the Publisher file using `rosrun topic_training PlotDemo` to execute the C++ version, or `rosrun topic_training PlotDemo.py` to execute the Python version. 
This should not print anything to the terminal.

5. In a new terminal window, repeat the commands `cd /workspaces/WRoverPlayground`, `catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`, and `source devel/setup.bash` in this order. 

6. With everything hopefully set up properly, you may now run `rosrun topic_training Demo` to execute the C++ version of the demo, or `rosrun topic_training Demo.py` to execute the Python version of the demo. 
In the terminal, you should see the program continually print "currently at {next sine value}" until you exit the program by inputting `Ctrl+C` in the terminal or the program crashes.

### rqt_graph and rqt_plot
1. In another terminal window, run `rosrun rqt_graph rqt_graph`. 
You should already be familiar with this command from the ROS Debug/Tools Training. 

2. Make sure the graph representation is what is expected. 
It should show "/PlotDemo" enclosed in a circle pointing to "/Demo" with an arrow named "/plot".

You should have also learned about rqt_plot. 

1. Type Ctrl-C into the terminal to stop running Demo(.cpp or .py) so only PlotDemo (.cpp or .py) is running. 

2. Run `rosrun rqt_plot rqt_plot`. 
You should see a sine graph being plotted onto the screen. Recall that rqt_plot can only be used with numerical values. Using it with the file Demo(.cpp or .py) would not have worked because that is dealing with Strings. Feel free to check this yourself.

3. You may also look through the file PlotDemo(.cpp or .py) to see how the math works and to see another example of a Publisher but with a different message type. 
For simplicity's sake, the comments in these files are only explaining the math since you should already understand the basic ROS functions from the previous section of this training.

While developing your code for the challenge, use rqt_graph and rqt_plot to make sure your nodes are working as expected.

## Defining a Custom Topic Type
A topic's type is determined by the type of the message being published. If you run the command `rostopic type chatter`, you should see the terminal print out "std_msgs/String" while roscore and rosrun are executing Demo(.cpp or .py). The command will print out "std_msgs/Float64" if you are executing PlotDemo(.cpp or .py).

A topic can have messages of type int8, int16, int32, int64, uint*, float32, float64, String, time, duration, other msg files, and array[] of variable length or fixed length. To message a different type, just change where ever there would be a String in Demo(.cpp or .py) to your desired type and update the includes (C++) or imports (Python) at the top of the file. Remember though, use message types instead of variable types, that is, use std_msgs/String (for C++) or std_msgs.msg.String (for Python) instead of just String.

Along with the types listed above, you can also create custom topic types. Custom topic types are defined in .msg files. 

1. In `/workspaces/WRoverPlayground/src/topic-training/msg/`, find the CustomType.msg file. 
This file shows an example of a custom message type. The format is `{variable type} {variable name}`. As you can see, creating custom types allows us to have data that contains multiple fields/attributes. If you have experience with Object-oriented programming, this concept should be similar to Objects.

Please know that this custom type is not used in any example code in this training. It is only here to show you how to use it and set it up for future projects. Do not actually change anything in the next few steps.

2. To use .msg files, you need to add the directive `add_message_files` in the CMakeLists.txt file and list the custom types needed for your program. 
It is already added for you and, currently, just lists the CustomType.msg file. 

3. In your actual code, you need to add `#include "{ROS package}/{msg file}.h"` for C++ or `from {ROS package}.msg import {msg file}` for Python. 
Using the provided example file, this would be `#include "topic_training/CustomType.h"` or `from topic_training.msg import CustomType`. 

4. To initialize your message, write `{ROS package}::{file name} msg;` for C++ or `msg = {file name}();` for Python. 
In this example, it would be `topic-training::CustomType msg;` or `msg = CustomType();`. 

5. Both languages use the syntax `msg.{variable name}`, such as `msg.player_name`, to access the different fields/attributes of the message type.

## Challenge
Once you have completed the training from WRoverPlayground, you should have some working code that will be put into a simulation. The simulation will include a rover that your code is meant to control using mock navigation data to reach some beacon. There will be multiple rounds of this, each round containing one beacon.

You will take in sensor data as a 180-element array of floats that represents a 180 degree view of the robot. The array is supposed to mock the input from a LiDAR, which is what we use for our rover. If you want to learn more about what a LiDAR is, here are two useful links from the [National Oceanic and Atmospheric Administration](https://oceanservice.noaa.gov/facts/lidar.html)(NOAA) and the [National Ecological Observatory Network](https://www.neonscience.org/resources/learning-hub/tutorials/lidar-basics). 

The array is a series of readings of distances detected by the LiDAR. Each element is a degree-sized slice of the robot's view. One element will contain a float, while the other elements will contain `Inf`. The float in this array represents the distance from the head of the robot to the head of the beacon. If the element contains `Inf`, then there are no obstacles detected in that direction.

The location of an element in the array represents its position relative to the robot. The readings are clockwise, so the 0th index maps to 9 o'clock, the 89th index maps to 12 o'clock, and the 179th index maps to 3 o'clock. This means that if the float is in the 89th index, the beacon is straight ahead. If the float is before the 89th index, the beacon is to the robot's left, and vice versa. A new array is given at the beginning of every round.

Using this information, you will output speeds, on the interval [-1, 1], for the robot until it reaches its beacon. There will be two separate values controlling the speed of the left side and the speed of right side. To learn more about how the controllers are divided between the left and right side, refer back to the general description of the training.

The general idea should be very similar to the PlotDemo and Demo examples. All within the same file, 
1. the Subscriber should receive the sensor data from the LiDAR
2. you will interpret the array and determine if the robot should go left, right, or straight ahead
3. and the Publisher will send the speed values for the left and right motors to the rover.

Go into the MotorChallenge(.cpp or .py) file in the directory /workspaces/WRoverPlayground/src/topic_training/src. This is where you will be writing your code. Feel free to use the examples provided in this training and any other resources that will be useful. The CMakeLists.txt file should already be set up to run your code. 
