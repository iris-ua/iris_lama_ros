# after running docker for the first time
./ros_entrypoint.sh
cd ~/turtlebot3_ws && colcon build --symlink-install
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
source ~/.bashrc

# to start docker with X server
xhost +local:docker
sudo docker ps -a
sudo docker start gallant_matsumoto
sudo docker attach gallant_matsumoto

# To move TB3 randomly
tmux
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 run turtlebot3_gazebo turtlebot3_drive
ros2 launch turtlebot3_bringup rviz2.launch.py

# To use IRIS LaMa:
	# TurtleBot3 (use waffle, burger, or waffle_pi)
	export TURTLEBOT3_MODEL=waffle
	ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
	ros2 run turtlebot3_teleop teleop_keyboard

	# Map making
	ros2 launch iris_lama_ros2 pf_slam2d_live_launch.py
	ros2 run nav2_map_server map_saver -f /mnt/myfolder/map

	# Navigating
	# TB3 doesnt have a planar sensor, so you need to uncomment line 336 of loc2d_ros.cpp and rebuild
	ros2 launch iris_lama_ros2 loc2d_launch.py

	# show map
	ros2 run nav2_map_server map_server __params:=map_server_params.yaml
	ros2 run nav2_util lifecycle_bringup map_server

	# show rviz2
	ros2 launch turtlebot3_bringup rviz2.launch.py
	# will show some laser_scan errors: https://github.com/ROBOTIS-GIT/turtlebot3/issues/508
	# originally for slam and rviz: ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
