# WRoverPlayground - Topic Training
Now that you have completed the Tools Training, you are ready to learn about ROS topics. From the Setup Training, you should be familiar with:
- the basics of ROS Nodes
- `rosnode`
- and `roscore`.

Once you complete this training, you should be familiar with:
- the purpose of topics
- the relationship between Publishers and Subscribers
- setting up Publishers and Subscribers
- running Publishers and Subscribers
- `rqt_graph`
- `rqt_plot`
- custom topic types
- and `rostopic type {topic}`.

## The Purpose of Topics
The word "topic" sounds very general, but it is actually a specific concept that allows ROS nodes to communicate with each other. This is extremely important because one node cannot possibly contain all the information a rover would need.

Topics allow us to specialize nodes, so they can execute their own code/tasks, while also sending and receiving information. For example, a node could receive data and send it to another one, a node could receive and analyze data and have another node do some action with it, a node could receive data from multiple nodes, etc.

## Publisher/Subscriber Relationship
The Publisher and Subscriber are both nodes that are connected by a topic, allowing them to send messages between each other. The Publisher node publishes messages on the topic, while the Subscriber node subscribes to the same topic to receive the Publisher's messages. 

Publishers and Subscribers do not need to be in separate files. In fact, if the Publisher and Subscriber rely on each other's data, it may be better to keep them in the same file. An example of the Publisher and Subscriber residing in the same file will be provided in the next section.

## Setting up a Publisher and Subscriber
Within this directory, you will find two files called Demo(.cpp or .py). These files will show a basic class containing both a Publisher and Subscriber. There will be comments throughout the files that explain the purpose of specific sections of the code.

Note that there are quite a few sections of code that are marked as "Optional." You do not need to include everything in the examples when creating your own code. They are simply showing one way of setting up a Publisher and Subscriber in the same file.

One file will be written in C++, and the other will be in Python. It is recommended that you choose the language you are most comfortable with to complete the training. Afterwards, you should discuss with one of the Software Leads to determine which part of the rover you would like to be assigned to. You will be given small tasks first, working your way up to more difficult projects. If these tasks/projects require a different language, then you can look through this training again or take what you know from the first time and learn as you go.

Go through one of the files, making sure you understand all the code and comments. Come back to this file once you feel ready to move on.

## Running a Publisher and Subscriber
Before you are able to run the example code, find the CMakeLists.txt file that is outside of the src directory but still inside of the topic_training directory. This file will include the the configurations, builds, and installs required to run your program as well as the files you will be running. Do not change anything in the file as it should already be set up for the demo files to work. 

The `find_package`, `catkin_package`, and `include_directories` specifications are required for both the C++ and Python files to work. These provide the packages and directories needed for running ROS and ROS topics. 

To run the C++ files, you will need the `add_executable`, `add_dependencies`, and `target_link_libraries` specifications. There will be three of each, representing the three C++ files in this section. `add_executable` determines which file you will use with the command `rosrun`, `add_dependencies` shows what the executable file depends on, and `target_link_libraries` links libraries or executable targets that may be used in the program. In larger projects, these specifications may contain different names, but since the code examples in this training are simple, they are all the same.

For the Python files, you will only need the `catkin_install_python` specification. This will list out the locations of the Python files being used.

In future projects, you might need to use other configurations, builds, and installs, but for the purposes of this training, you only need these for the example code to run.

You can also check out the package.xml file, which you also may need to tailor for future projects. However, everything should already be set up correctly, so do not change anything. At the top of the file, it should name the creator of the package. Then, it will include the license regarding open source policies. Right now, it should not specify a license because the license for the package for the WRoverPlayground training trickles down to this package. At the bottom, there should be various dependencies that allow us to use ROS and ROS topics, similar to the CMakeLists.txt `find_package`.

In the terminal, run the command `roscore`. In a new terminal window, cd into the catkin workspace. In this case, use `cd WRoverPlayground`. Then, use the commands `catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` and `source devel/setup.bash`. These commands adjust the catkin workspace based on the CMakeLists.txt and package.xml files. You must run these commands everytime you change these two files. 

With everything hopefully set up properly, you may now run `rosrun topic_training Demo` to execute the C++ version of the demo, or `rosrun topic_training Demo.py` to execute the Python version of the demo. In the terminal, you should see the program continually print "hello world {count} I heard: [hello world {count}]" until you exit the program by inputting `Ctrl+C` in the terminal or the program crashes.

### rqt_graph and rqt_plot
In another terminal window, run `rosrun rqt_graph rqt_graph`. This will show a graph representation of the current topic you are running. The name of the file (+ the ID number if you are using Python) should be encircled with an arrow named after the topic (in this case "chatter") pointing to itself since the file contains both the Publisher and Subscriber.

There is also another command that helps you visualize the nodes and topics. Type Ctrl-C to stop running Demo(.cpp or .py) and, instead, run `rosrun topic_training PlotDemo` or `rosrun topic_training PlotDemo.py`. You should not see anything print. The file PlotDemo(.cpp or .py) is a Publisher (no Subscriber) that sends sine values to the same topic ("chatter"). If you run `rosrun rqt_plot rqt_plot`, you should see a sine graph being plotted onto the screen. The command rqt_plot is only able to be used on topics with a numerical type. Using it with the file Demo(.cpp or .py) would not have worked. 

If you run rqt_graph on the PlotDemo(.cpp or .py) file, it should show just the name of the file (+ the ID number if you are using Python) by itself with no arrow. If you run rqt_graph when both files are running, there might be an error in the terminal, but the graph application should show both files. PlotDemo(.cpp or .py) will be by itself with no arrow, and Demo(.cpp or .py) will be by itself with or without an arrow depending on the order you executed the two files.

Feel free to look through the file PlotDemo(.cpp or .py) to see how the math works and to see another example of a Publisher but with a different message type. For simplicity's sake, the comments in these files are only explaining the math since you should already understand the basic ROS functions from the previous section of this training.

### Defining a Custom Topic Type
A topic's type is determined by the type of the message being published. If you run the command `rostopic type chatter`, you should see the terminal print out "std_msgs/String" while roscore and rosrun are executing Demo(.cpp or .py). The command will print out "std_msgs/Float64" if you are executing PlotDemo(.cpp or .py).

A topic can have messages of type int8, int16, int32, int64, uint*, float32, float64, String, time, duration, other msg files, and array[] of variable length or fixed length. To message a different type, just change where ever there would be a String in Demo(.cpp or .py) to your desired type and update the includes (C++) or imports (Python) at the top of the file.

## Challenge
Once you have completed the training from WRoverPlayground, you should have some working code that will be put into a simulation. The simulation will have a rover that your code is meant to control its drive through mock navigation data to reach some target/beacon. 

You will take in sensor data as a 180-element array of floats that represents a 180 degree view of the robot. Each element is a degree-sized slice of the robot's view. One element will contain a float, while the other elements will be undefined. The float in this array represents the distance from the head of the robot to the head of the target. The location of the element in the array represents its position relative to the robot. If the float is in the 90th element, the target is straight ahead. If the float is before the 90th element, the target is to the robot's left, and vice versa. 

Using this information, you will output speeds, on the interval [-1, 1], for the robot until it reaches its target. There will be two separate values controlling the speed of the left side and the speed of right side. To learn more about how the controllers are divided between the left and right side, refer back to the general description of the training.

Go into the MotorChallenge(.cpp or .py) file. This is where you will be writing your code. Feel free to use the examples provided in this training and any other resources that will be useful. The CMakeLists.txt file should already be set up to run your code if you uncommented correctly.