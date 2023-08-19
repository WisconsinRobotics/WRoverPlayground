# ROS Setup

This covers the ROS package structure and how to make your own ROS packages.
ROS packages are a way to organize code within a ROS project.

## Creating your own ROS package

In the `src` folder, run `catkin_create_pkg wrover_test`.

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
* `src/` contains code
* `msg/` contains message files
* `srv/` contains service files
* `launch/` contains launch files that define ways to start nodes

## References

http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem

http://wiki.ros.org/ROS/Tutorials/CreatingPackage

http://wiki.ros.org/Packages

http://wiki.ros.org/ROS/Concepts
