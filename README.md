LaMa ROS - Alternative Localization and Mapping for ROS.
========================================================
https://github.com/iris-ua/iris_lama_ros

![Build Melodic](https://github.com/iris-ua/iris_lama_ros/workflows/Build%20Melodic/badge.svg)

Developed and maintained by Eurico Pedrosa, University of Aveiro (C) 2019.

Overview
--------

ROS integration of [LaMa]( https://github.com/iris-ua/iris_lama), a Localization and Mapping package from the **Intelligent Robotics and Systems** (IRIS) Laboratory, University of Aveiro. It provides 2D Localization and SLAM. It works great on a [TurtleBot2](https://www.turtlebot.com/turtlebot2/) with a [Raspberry Pi 3 Model B+](https://www.raspberrypi.org/products/raspberry-pi-3-model-b-plus/) and an Hokuyo (Rapid URG).

#### Build

To build LaMa ROS, clone it from GitHub and use `catkin` to build.
```
mkdir src
cd src
git clone https://github.com/iris-ua/iris_lama
git clone https://github.com/iris-ua/iris_lama_ros
cd ..
catkin config --extend /opt/ros/melodic
catkin build
```
The build was tested in **Ubuntu 18.04** with ROS **melodic**. It will not build with `catkin_make` or `catkin_make_isolated`.

## SLAM nodes

To create a map using *Online SLAM* execute
```
rosrun iris_lama_ros slam2d_ros scan_topic:=base_scan
```
and to create a map using *Particle Filter SLAM* execute
```
rosrun iris_lama_ros pf_slam2d_ros scan_topic:=base_scan
```

Both nodes will publish to expected topics such as `/map` and `/tf`.

### Offline Mapping (rosbag)

If you want to obtain a map from a rosbag and you want to save time (a lot),
you can let iris_lama_ros "play" the rosbag for you.

```
roslaunch iris_lama_ros slam2d_offine.launch scan_topic:=base_scan rosbag:=/path/your/rosbag.bag
```
or
```
roslaunch iris_lama_ros pf_slam2d_offine.launch scan_topic:=base_scan rosbag:=/path/your/rosbag.bag
```

### Parameters

* `~global_frame_id`: The frame attached to the map (default: "map").
* `~odom_frame_id`: The frame attached to the odometry system (default: "odometry").
* `~base_frame_id`: The frame attached to the mobile base (default: "base_link").
* `~scan_topic`: Laser scan topic to subscribe (default: "/scan").
* `~initial_pos_x`: Initial x position (default: 0 meters).
* `~initial_pos_y`: Initial y position (default: 0 meters).
* `~initial_pos_a`: Initial rotation (or angle) (default: 0 rad).
* `~d_thresh`: Traveled distance to accumulate before updating (default: 0.01 meters).
* `~a_thresh`: Angular motion to accumulate before updating (default: 0.25 rads).
* `~l2_max`: Maximum distance to use in the dynamic Euclidean distance map (default: 0.5 meters).
* `~resolution`: Resolution of the grid maps (default: 0.05 meters).
* `~patch_size`: Length of a patch (default: 32 cells).
* `~strategy`: Scan matching optimization strategy, GaussNewton ("gm") or Levenberg Marquard ("lm") (default: "gn").
* `~max_iterations`: Maximum number of interations performed by the optimizer (default: 100)
* `~use_compression`: Should the maps be compressed (default: false).
* `~compression_algorithm`: Compression algorithm to use, lz4 or zstd (default: "lz4").
* `~cache_size`: Size of the LRU used during online data compression (default: 100).
* `~mrange`: Maximum laser scan range (default: 16 meters).
* `~map_publish_period`: How long between updates to the map (default: 5 seconds).

Particle Filter SLAM only:
* `~d_thresh`: Traveled distance to accumulate before updating (default: 0.5 meters).
* `~particles`: Number of particles to use (default: 30).
* `~seed`: RNG seed value, use 0 for a random seed from device (default: 0)
* `~threads`: Number of working threads, -1 means disabled and 0 will expand to the available number of cores (default: -1).
* `~sigma`: Measurement variance (default: 0.05).
* `~lgain`: Gain value for smoothing the particles likelihood (default: 3.0).
* `~srr`: Odometry error in rotation as a function of rotation (default: 0.1).
* `~str`: Odometry error in rotation as a function of translation (default: 0.2).
* `~stt`: Odometry error in traslation as a function of translation (default: 0.1).
* `~srt`: Odometry error in translation as a funciton of rotation (default: 0.1).

## Localization node

This node requires the existence of the `/static_map` service to load the map.
To run the localization just execute
```
rosrun iris_lama_ros loc2d_ros scan:=base_scan
```
Please use `rviz` to set the initial pose. Global localization is not yet implemented.

### Parameters

* `~global_frame_id`: The frame attached to the map (default: "map").
* `~odom_frame_id`: The frame attached to the odometry system (default: "odometry").
* `~base_frame_id`: The frame attached to the mobile base (default: "base_link").
* `~scan_topic`: Laser scan topic to subscribe (default: "/scan").
* `~initial_pos_x`: Initial x position (default: 0 meters).
* `~initial_pos_y`: Initial y position (default: 0 meters).
* `~initial_pos_a`: Initial rotation (or angle) (default: 0 rad).
* `~d_thresh`: Traveled distance to accumulate before updating (default: 0.01 meters).
* `~a_thresh`: Angular motion to accumulate before updating (default: 0.2 rads).
* `~l2_max`: Maximum distance to use in the dynamic Euclidean distance map (default: 0.5 meters).
* `~strategy`: Scan matching optimization strategy, GaussNewton ("gm") or Levenberg Marquard ("lm") (default: "gn").
* `~patch_size`: Length of a patch (default: 32 cells).


