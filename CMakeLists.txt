cmake_minimum_required(VERSION 3.5)
project(iris_lama_ros)

# Default to release mode.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(iris_lama REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    rosbag
    nav_msgs
    sensor_msgs
    tf_conversions
    geometry_msgs
    message_filters
    visualization_msgs
    rosbag_storage
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})
add_subdirectory(src)

#############
## Install ##
#############

## Mark cpp header files for installation
#install(DIRECTORY include/rtk/ros/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.h"
#    PATTERN ".svn" EXCLUDE
#)


