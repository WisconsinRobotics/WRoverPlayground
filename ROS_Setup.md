# ROS Setup

This covers the ROS package structure and how to make your own ROS packages. ROS packages are a way to organize code within a ROS project.

## Creating your own package

In the `src` folder, run `catkin_create_pkg wrover_test`.

This should create `wrover_test` directory with a `CMakeLists.txt` and `package.xml`. You may have noticed that there are some other directories and files in the packages that you have worked in.

## ROS package structure

Some common directories and files in a ROS package are:

* `launch/` contains launch files that define ways to start nodes in your package
* `msg/` contains Message types
* `src/` contains code
* `srv/` contains Service types
* `CMakeLists.txt` CMake build file
* `package.xml` Defines package information and dependencies

## References

http://wiki.ros.org/Packages

http://wiki.ros.org/ROS/Concepts