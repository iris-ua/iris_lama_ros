cmake_minimum_required(VERSION 3.5)
project(iris_lama_ros)

# Default to release mode.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(iris_lama REQUIRED)
find_package(yaml-cpp REQUIRED)

set(CMAKE_CXX_STANDARD 14)
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
    std_srvs
)

catkin_package()

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})
add_subdirectory(src)


# Install our lannch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE
)


