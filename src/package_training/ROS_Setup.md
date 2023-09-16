# ROS Setup

This covers the ROS package structure and how to make your own ROS packages.
ROS packages are a way to organize code within a ROS project.

## Creating your own ROS package

In the `/workspaces/WRoverPlayground/src` folder, run `catkin_create_pkg wrover_test`.

This creates a `wrover_test` directory with a `CMakeLists.txt` and `package.xml`.
You may have noticed that there are some other directories and files in the packages that you have worked in.

## Navigating ROS packages

ROS provides commands that make navigating ROS packages convenient.

To find the location of the new `wrover_test` package, run `rospack find wrover_test`.
The output should be `/workspaces/WRoverPlayground/src/wrover_test`.

To list the contents of the `wrover_test` package, run `rosls wrover_test`.
Since it is a new package, there should be two files: `CMakeLists.txt` and `package.xml`.

To change the working directory to the `wrover_test` package, run `roscd wrover_test`.

## ROS package structure

Some common directories and files in a ROS package are:

* `CMakeLists.txt` CMake build file
* `package.xml` Defines package information and dependencies

The following are optional and only needed if the package uses these:
* `src/` contains code
* `msg/` contains message files
* `srv/` contains service files
* `launch/` contains launch files that define ways to start nodes
* `node/` helpful for separating executable files from backing libraries, mainly for Python 

## Editing `CMakeLists.txt` and `package.xml`

Copy the `msg/`, `src/`, and `srv/` folders from this folder to the `wrover_test` package.

In `package.xml`, find and uncommment the following lines to use messages and services in the package:
```XML
<!--   <build_depend>message_generation</build_depend> -->
```
```XML
<!--   <exec_depend>message_runtime</exec_depend> -->
```

Use `CMakeLists.txt` to specify message and service files that are required by the package.

Change the line `find_package(catkin REQUIRED)` to the following:
```CMake
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```
If you need messages, services, or libraries from other packages, they should also go in this list.

To specify messages from the `msg/` folder of a package, find and uncomment the following lines:
```CMake
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
```
Replace `Message1.msg` and `Message2.msg` with the names of the message files in `msg/`.

To specify services from the `srv/` folder of a package, find and uncomment the following lines:
```CMake
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
```
Replace `Service1.srv` and `Service2.srv` with the names of the service files in `srv/`.

Finally, uncomment the following lines:
```CMake
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )
```

You should now be able to run `catkin_make` to generate message and service header files.

To test that this works, start `roscore & rosrun wrover_test main.py`.
Then, open a new terminal and run `rostopic echo /hello_world`.
You should see output similar to:
```
hello: "Hello World!"
iteration: 3
---
hello: "Hello World!"
iteration: 4
---
```

## References

http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem

http://wiki.ros.org/ROS/Tutorials/CreatingPackage

http://wiki.ros.org/Packages

http://wiki.ros.org/ROS/Concepts

http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
