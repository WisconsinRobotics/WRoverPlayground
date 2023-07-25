# WRoverPlayground - Topic Training
Now that you have completed the Setup Training, you are ready to learn about ROS topics. From the Setup Training, you should be familiar with:
(*list out what newcomers should already know*)
- ROS Nodes
- etc.
- these ROS commands:
    - .
Once you complete this training, you should be familiar with:
(*list out what newcomers should learn*)
- topics
- pub/sub relationship
- these ROS commands:
    - .

## The Purpose of Topics
The word "topic" sounds very general, but it is actually a specific concept that allows ROS nodes to communicate with each other. 

## Publisher/Subscriber Relationship
The Publisher and Subscriber are both nodes that are connected by a topic, allowing them to send messages between themselves. The Publisher node publishes messages on the topic, while the Subscriber node subscribes to the same topic to receive the Publisher's messages. 

Publishers and Subscribers do not need to be in separate files. In fact, if the Publisher and Subscriber rely on each other's data, it may be better to keep them in the same file. An example of the Publisher and Subscriber residing in the same file will be provided in the next section.

## Setting up a Publisher and Subscriber
Within this directory, you will find two files called Demo(.cpp or .py). These files will show a basic class containing both a Publisher and Subscriber. There will be comments throughout the files that explain the purpose of specific sections of the code.

Note that there are quite a few sections of code that are marked as "Optional." You do not need to include everything in the examples when creating your own code. They are simply showing one way of setting up a Publisher and Subscriber in the same file.

One file will be written in C++, and the other will be in Python. I recommend discussing with one of the Software Leads to determine which part of the rover you would like to be assigned to. This will help you decide which language to focus on for this training. However, being familiar with both languages will not hurt.

Go through one of the files, making sure you understand all the code and comments. Come back to this file once you feel ready to move on.

### Running a Publisher and Subscriber
Before you are able to run the example code, find the CMakeLists.txt file that is outside of the src directory but still inside of the topic_training directory. 

This file is mostly commented out. Make sure these are uncommented:

- find_package(catkin REQUIRED
    roscpp
    rospy
    std_msgs
    message_generation
  )

- catkin_package(
    CATKIN_DEPENDS message_runtime
  )

- include_directories(
    ${catkin_INCLUDE_DIRS}
  )

The C++ and the Python file both require these to work.

If you chose C++, also uncomment these:

- add_executable(Demo src/Demo.cpp)
  add_executable(PlotDemo src/PlotDemo.cpp)

- add_dependencies(Demo 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
  )
  add_dependencies(PlotDemo 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
  )

- target_link_libraries(Demo 
    ${catkin_LIBRARIES}
  )
  target_link_libraries(PlotDemo
    ${catkin_LIBRARIES}
  )

If you chose Python, also uncomment this:

- catkin_install_python(PROGRAMS src/Demo.py src/PlotDemo.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

In future projects, you might need to use some of the other configurations, builds, and installs, but for the purposes of this training, you only need these for the example code to run.

You can also check out the package.xml file, which you also may need to tailor for future projects. However, everything should already be set up correctly, so do not change anything.

In the terminal, run the command *roscore*. In a new terminal window, cd into the catkin workspace. In this case, use *cd WRoverPlayground*. Then, use the commands *catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON* and *source devel/setup.bash*. These commands adjust the catkin workspace based on the CMakeLists.txt and package.xml files. You must run these commands everytime you change these two files. 

With everything hopefully set up properly, you may now run *rosrun topic_training Demo* or *rosrun topic_training Demo.py*. In the terminal, you should see the program continually print "hello world {count} I heard: [hello world {count}]" until the terminal receives Ctrl-C or the program crashes.

### rqt_graph and rqt_plot
In another terminal window, run *rosrun rqt_graph rqt_graph*. This will show a graph representation of the current topic you are running. The name of the file (+ the ID number if you are using Python) should be encircled with an arrow named after the topic (in this case "chatter") pointing to itself since the file contains both the Publisher and Subscriber.

There is also another command that helps you visualize the nodes and topics. Type Ctrl-C to stop running Demo(.cpp or .py) and, instead, run *rosrun topic_training PlotDemo* or *rosrun topic_training PlotDemo.py*. You should not see anything print. The file PlotDemo(.cpp or .py) is a Publisher (no Subscriber) that sends sine values to the same topic ("chatter"). If you run *rosrun rqt_plot rqt_plot*, you should see a sine graph being plotted onto the screen. The command rqt_plot is only able to be used on topics with a numerical type. Using it with the file Demo(.cpp or .py) would not have worked. 

If you run rqt_graph on the PlotDemo(.cpp or .py) file, it should show just the name of the file (+ the ID number if you are using Python) by itself with no arrow. If you run rqt_graph when both files are running, there might be an error in the terminal, but graph application should show both files. PlotDemo(.cpp or .py) will be by itself with no arrow, and Demo(.cpp or .py) will be by itself with or without an arrow depending on the order you executed the two files.

Feel free to look through the file PlotDemo(.cpp or .py) to see how the math works and to see another example of a Publisher but with a different message type. For simplicity's sake, the comments in these files are only explaining the math since you should already understand the basic ROS functions from the previous section of this training.

### Defining a Custom Topic Type
A topic's type is determined by the type of the message being published. If you run the command *rostopic type chatter*, you should see the terminal print out "std_msgs/String" while roscore and rosrun are executing Demo(.cpp or .py). The command will print out "std_msgs/Float64" if you are executing PlotDemo(.cpp or .py).

A topic can have messages of type int8, int16, int32, int64, uint*, float32, float64, String, time, duration, other msg files, and array[] of variable length or fixed length. The message in the example code is of type String. To message a different type, just change the Strings to your desired type and update the includes (C++) or imports (Python).

## Challenge
