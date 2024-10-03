cmake_minimum_required(VERSION 3.16)
project(multi)

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
# Package: multi
add_library(multi SHARED  # Library
    src/robot.cpp
    src/world.cpp
    src/lidar.cpp
    src/simple_geometry.cpp
)
target_include_directories(multi PUBLIC  # Include dirs
    include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(multi PUBLIC # Link OpenCV/Eigen3 libs
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
)
# Node: multi_node
add_executable(multi_node # Node
    bin/multi_node.cpp
)
target_include_directories(multi_node PUBLIC # Include dirs
    include 
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(multi_node PUBLIC# Link multi libs
    multi
    ${catkin_LIBRARIES}
)
# If you want to use jsoncpp library, remember to link it!
# target_link_libraries(<your_module> PUBLIC/PRIVATE jsoncpp_lib)