cmake_minimum_required(VERSION 3.16)
project(multi)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

# Find Eigen3 and OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(jsoncpp REQUIRED)

# Include directories for OpenCV and Eigen3
INCLUDE_DIRECTORIES(. /usr/include/eigen3   # Eigen3
                    ${OpenCV_INCLUDE_DIRS}  # OpenCV
)

# ----- SIMPLE CPP ----- #
# Just in case we want to run ./main
# add_executable(main
#   src/simple_geometry.cpp
#   src/world.cpp
#   src/robot.cpp
#   src/lidar.cpp
#   src/main.cpp
# )

# target_include_directories(main PUBLIC include "include")

# target_link_libraries(
#   main
#   ${OpenCV_LIBS}
# )
# ---------------------- #

# ----- ROS Integration ----- #
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
    src/misc.cpp
)

target_include_directories(multi PUBLIC  # Include dirs
    include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(multi PUBLIC # Link OpenCV/Eigen3 libs
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
    jsoncpp_lib
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
    jsoncpp_lib
)

# If you want to use jsoncpp library, remember to link it!
# target_link_libraries(<your_module> PUBLIC/PRIVATE jsoncpp_lib)
# --------------------------- #