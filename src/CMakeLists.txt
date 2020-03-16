
add_executable(loc2d_ros loc2d_ros.cpp)
ament_target_dependencies(loc2d_ros Eigen3 tf2_ros rclcpp nav_msgs sensor_msgs geometry_msgs message_filters visualization_msgs rosbag2 iris_lama)
#target_link_libraries(loc2d_ros ${catkin_LIBRARIES})

add_executable(slam2d_ros slam2d_ros.cpp offline_replay.cpp)
ament_target_dependencies(slam2d_ros Eigen3 tf2_ros rclcpp nav_msgs sensor_msgs geometry_msgs message_filters visualization_msgs rosbag2 iris_lama)
#target_link_libraries(slam2d_ros ${catkin_LIBRARIES})

add_executable(pf_slam2d_ros pf_slam2d_ros.cpp offline_replay.cpp)
ament_target_dependencies(pf_slam2d_ros Eigen3 tf2_ros rclcpp nav_msgs sensor_msgs geometry_msgs message_filters visualization_msgs rosbag2 iris_lama)
#target_link_libraries(pf_slam2d_ros ${catkin_LIBRARIES})

install(TARGETS loc2d_ros slam2d_ros pf_slam2d_ros
    ARCHIVE DESTINATION ./ilr2_build/
    LIBRARY DESTINATION ./ilr2_build/
    RUNTIME DESTINATION ./ilr2_build/
)
