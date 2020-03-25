cmake_minimum_required(VERSION 3.5)
project(iris_lama_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)

find_package(tf2_ros REQUIRED)  #tf_ros
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)   #roscpp
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(tf_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosbag2 REQUIRED) #rosbag_storage

find_package(iris_lama REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

include_directories(include /usr/include/eigen3)  #TODO use some variable for eigen3
add_subdirectory(src)
