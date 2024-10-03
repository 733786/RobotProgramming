cmake_minimum_required(VERSION 3.16)
project(mrsim)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    # Add required ROS packages to use in this project (ie. sensor_msgs, nav_msgs, etc)
    roscpp
	std_msgs
    geometry_msgs
    message_generation
)
catkin_package(
    INCLUDE_DIRS "include/"
    CATKIN_DEPENDS
    roscpp
    # Add required ROS dependencies
)
include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
)
# Package: mrsim
add_library(mrsim SHARED  # Library
    src/robot.cpp
    src/world.cpp
    src/lidar.cpp
    src/simple_geometry.cpp
)
target_include_directories(mrsim PUBLIC  # Include dirs
    include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(mrsim PUBLIC # Link OpenCV/Eigen3 libs
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
)
# Node: mrsim_node
add_executable(mrsim_node # Node
    bin/mrsim_node.cpp
)
target_include_directories(mrsim_node PUBLIC # Include dirs
    include 
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(mrsim_node PUBLIC# Link mrsim libs
    mrsim
    ${catkin_LIBRARIES}
)
# If you want to use jsoncpp library, remember to link it!
# target_link_libraries(<your_module> PUBLIC/PRIVATE jsoncpp_lib)