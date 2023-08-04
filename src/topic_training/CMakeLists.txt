cmake_minimum_required(VERSION 3.0.2)
project(topic_training)

## Compile as C++20, supported in ROS Kinetic and newer
# add_compile_options(-std=c++20)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
   roscpp
   rospy
   std_msgs
   message_generation
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES topic_training
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable 
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(Demo src/Demo.cpp)
add_executable(PlotDemo src/PlotDemo.cpp)
add_executable(MotorChallenge src/MotorChallenge.cpp)

## Add cmake target dependencies of the executable 
## same as for the library above
add_dependencies(Demo 
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
)
add_dependencies(PlotDemo 
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
)
add_dependencies(MotorChallenge 
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against 
target_link_libraries(Demo
   ${catkin_LIBRARIES}
)
target_link_libraries(PlotDemo
   ${catkin_LIBRARIES}
)
target_link_libraries(MotorChallenge
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation 
# in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS src/Demo.py src/PlotDemo.py src/MotorChallenge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
