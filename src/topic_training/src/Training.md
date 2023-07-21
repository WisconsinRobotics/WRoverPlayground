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
- add_dependencies(Demo 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
  )
- target_link_libraries(Demo 
    ${catkin_LIBRARIES}
  )

If you chose Python, also uncomment this:
- catkin_install_python(PROGRAMS src/Demo.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

In future projects, you might need to use some of the other configurations, builds, and installs, but for the purposes of this training, you only need these for the example code to run.

You can also check out the package.xml file, which you also may need to tailor for future projects. However, everything should already be set up correctly, so do not change anything.

In the terminal, run the command *roscore*. In a new terminal window, cd into the catkin workspace. In this case, use *cd WRoverPlayground*. Then, use the commands *catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON* and *source devel/setup.bash*. These commands adjust the catkin workspace based on the CMakeLists.txt and package.xml files. You must run these commands everytime you change these two files. 

With everything hopefully set up properly, you may now run *rosrun topic_training Demo* or *rosrun topic_training Demo.py*. In the terminal, you should see the program continually print "hello world {count} I heard: [hello world {count}]" until the terminal receives Ctrl-C or the program crashes.

In another terminal window, run *rosrun rqt_graph rqt_graph*. This will show a graph representation of the current topic you are running. The name of the file (+ the ID number if you are using Python) should be encircled with an arrow named after the topic (in this case "chatter") pointing to itself since the file contains both the Publisher and Subscriber.

(*rosrun rqt_plot rqt_plot*)

### Defining a Custom Topic Type
rostopic type chatter

A topic can have messages of type int8, int16, int32, int64, uint*, float32, float64, String, time, duration, other msg files, and array[] of variable length or fixed length. The message in the example code is of type String. To message a different type, just change the Strings to your desired type and update the includes (C++) or imports (Python).

## Challenge
