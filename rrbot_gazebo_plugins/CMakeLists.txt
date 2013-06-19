# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(rrbot_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS 
  ros_control_gazebo 
  ros_control_gazebo_plugin 
  effort_controllers 
  gazebo_ros
)

# Find gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
)

# Build RobotSim Interface
add_library(rrbot_gazebo src/robot_sim_rrbot.cpp)

## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS 
      ros_control_gazebo 
      ros_control_gazebo_plugin 
      effort_controllers 
      gazebo_ros
    CATKIN_DEPENDS
    INCLUDE_DIRS 
    LIBRARIES 
)
