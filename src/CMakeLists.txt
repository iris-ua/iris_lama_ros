add_library(loc2d_ros_component loc2d_ros.cpp)
ament_target_dependencies(loc2d_ros_component
    "tf2" "tf2_sensor_msgs" "tf2_msgs" "tf2_ros" "tf2_geometry_msgs" "rclcpp" "nav_msgs" "sensor_msgs"
    "geometry_msgs" "message_filters" "visualization_msgs" "rosbag2" "rclcpp_components")
target_link_libraries(loc2d_ros_component iris_lama::iris_lama)

add_library(slam2d_ros_component SHARED slam2d_ros.cpp)
ament_target_dependencies(slam2d_ros_component
        "Eigen3" "tf2" "tf2_sensor_msgs" "tf2_msgs" "tf2_ros" "tf2_geometry_msgs" "rclcpp" "nav_msgs" "sensor_msgs"
        "geometry_msgs" "message_filters" "visualization_msgs" "rosbag2" "rclcpp_components")
target_link_libraries(slam2d_ros_component iris_lama::iris_lama)

add_library(pf_slam2d_ros_component pf_slam2d_ros.cpp)
ament_target_dependencies(pf_slam2d_ros_component
        "Eigen3" "tf2" "tf2_sensor_msgs" "tf2_msgs" "tf2_ros" "tf2_geometry_msgs" "rclcpp" "nav_msgs" "sensor_msgs"
        "geometry_msgs" "message_filters" "visualization_msgs" "rosbag2" "rclcpp_components")
target_link_libraries(pf_slam2d_ros_component iris_lama::iris_lama)

add_library(graph_slam2d_ros_component graph_slam2d_ros.cpp)
ament_target_dependencies(graph_slam2d_ros_component
        "Eigen3" "tf2" "tf2_sensor_msgs" "tf2_msgs" "tf2_ros" "tf2_geometry_msgs" "rclcpp" "nav_msgs" "sensor_msgs"
        "geometry_msgs" "message_filters" "visualization_msgs" "rosbag2" "rclcpp_components")
target_link_libraries(graph_slam2d_ros_component iris_lama::iris_lama)

# Install libraries(composable nodes) and executables
ament_export_targets(export_loc2d_ros_component)
install(TARGETS loc2d_ros_component
        EXPORT export_loc2d_ros_component
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_export_targets(export_slam2d_ros_component)
install(TARGETS slam2d_ros_component
        EXPORT export_slam2d_ros_component
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_export_targets(export_pf_slam2d_ros_component)
install(TARGETS pf_slam2d_ros_component
        EXPORT export_pf_slam2d_ros_component
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_export_targets(export_graph_slam2d_ros_component)
install(TARGETS graph_slam2d_ros_component
        EXPORT export_graph_slam2d_ros_component
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Install launch files
install(DIRECTORY ../launch
        DESTINATION share/${PROJECT_NAME}/)

# Install config files
install(DIRECTORY ../config
        DESTINATION share/${PROJECT_NAME}/)
