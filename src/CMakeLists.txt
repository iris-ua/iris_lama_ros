
add_executable(loc2d_ros loc2d_ros.cpp)
target_link_libraries(loc2d_ros ${catkin_LIBRARIES})

add_executable(slam2d_ros slam2d_ros.cpp offline_replay.cpp)
target_link_libraries(slam2d_ros ${catkin_LIBRARIES})

add_executable(pf_slam2d_ros pf_slam2d_ros.cpp offline_replay.cpp)
target_link_libraries(pf_slam2d_ros ${catkin_LIBRARIES})

add_executable(ros2sdm ros2sdm.cpp)
target_link_libraries(ros2sdm ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

install(TARGETS loc2d_ros slam2d_ros pf_slam2d_ros ros2sdm
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

