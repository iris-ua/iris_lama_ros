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
* `~odom_frame_id`: The frame attached to the odometry system (default: "odom").
* `~base_frame_id`: The frame attached to the mobile base (default: "base_link").
* `~scan_topic`: Laser scan topic to subscribe (default: "/scan").
* `~transform_tolerance`: Defines how long map->odom transform is good for by [future dating tf](https://answers.ros.org/question/218864/why-does-amcl-post-date-tf-transform_tolerance/) (default: 0.1)
* `~initial_pos_x`: Initial x position (default: 0 meters).
* `~initial_pos_y`: Initial y position (default: 0 meters).
* `~initial_pos_a`: Initial rotation (or angle) (default: 0 rad).
* `~d_thresh`: Traveled distance to accumulate before updating (default: 0.01 meters).
* `~a_thresh`: Angular motion to accumulate before updating (default: 0.25 rads).
* `~l2_max`: Maximum distance to use in the dynamic Euclidean distance map (default: 0.5 meters).
* `~resolution`: Resolution of the grid maps (default: 0.05 meters).
* `~patch_size`: Length of a patch (default: 32 cells).
* `~strategy`: Scan matching optimization strategy, GaussNewton ("gm") or Levenberg Marquard ("lm") (default: "gn").
* `~max_iterations`: Maximum number of iterations performed by the optimizer (default: 100)
* `~use_compression`: Should the maps be compressed (default: false).
* `~compression_algorithm`: Compression algorithm to use, lz4 or zstd (default: "lz4").
* `~cache_size`: Size of the LRU used during online data compression (default: 100).
* `~mrange`: Maximum laser scan range (default: 16 meters).
* `~beam_step`: Number of beams to step (or skip) in each scan (default: 1).
* `~truncate`: Truncate the laser scan range from start to "middle" (default: 0.0 meters, 0.0 means no truncation).
* `~truncate_ray`: Truncate the laser scan range (or ray) from "middle" to end (default: 0.0 meters, 0.0 means no truncation).
* `~map_publish_period`: How long between updates to the map (default: 5 seconds).
* `~transient_map`: True to only keep the most recent portion of the map "sensed" by the latest surface (default": `false`)
* `~create_summary`: (default: `false`)

Particle Filter SLAM only:
* `~d_thresh`: Traveled distance to accumulate before updating (default: 0.5 meters).
* `~particles`: Number of particles to use (default: 30).
* `~seed`: RNG seed value, use 0 for a random seed from device (default: 0)
* `~threads`: Number of working threads, -1 means disabled and 0 will expand to the available number of cores (default: -1).
* `~sigma`: Measurement variance (default: 0.05).
* `~lgain`: Gain value for smoothing the particles likelihood (default: 3.0).
* `~srr`: Odometry error in rotation as a function of rotation (default: 0.1).
* `~str`: Odometry error in rotation as a function of translation (default: 0.2).
* `~stt`: Odometry error in translation as a function of translation (default: 0.1).
* `~srt`: Odometry error in translation as a function of rotation (default: 0.1).

### Services

* `/dynamic_map`: 

## Localization node

This node requires the existence of the `/static_map` service to load the map.
To run the localization just execute
```
rosrun iris_lama_ros loc2d_ros scan:=base_scan
```
Please use `rviz` to set the initial pose. Global localization is not yet implemented.

### Services

* `/request_nomotion_update`: Called to trigger an update without moving the robot (no-motion update)
* `/global_localization`: Called to trigger a global localization procedure.

### Parameters

* `~global_frame_id`: The frame attached to the map (default: "map").
* `~odom_frame_id`: The frame attached to the odometry system (default: "odometry").
* `~base_frame_id`: The frame attached to the mobile base (default: "base_link").
* `~scan_topic`: Laser scan topic to subscribe (default: "/scan").
* `~mrange`: Maximum laser scan range. When 0, maximum range defaults to the sensor maximum range. (default: 0 meters).
* `~transform_tolerance`: Defines how long map->odom transform is good for by [future dating tf](https://answers.ros.org/question/218864/why-does-amcl-post-date-tf-transform_tolerance/) (default: 0.1)
* `~temporal_update`: Force an update if the last processed scan is older than this. A value less or equal to zero disables the check (default: 0.0).
* `~publish_tf`: Publish TF correction from `global_frame_id` to `odom_frame_id` (default: true).
* `~beam_step`: Number of beams to step (or skip) in each scan (default: 1).
* `~initial_pos_x`: Initial x position (default: 0 meters).
* `~initial_pos_y`: Initial y position (default: 0 meters).
* `~initial_pos_a`: Initial rotation (or angle) (default: 0 rad).
* `~d_thresh`: Traveled distance to accumulate before updating (default: 0.01 meters).
* `~a_thresh`: Angular motion to accumulate before updating (default: 0.2 rads).
* `~l2_max`: Maximum distance to use in the dynamic Euclidean distance map (default: 0.5 meters).
* `~strategy`: Scan matching optimization strategy, GaussNewton ("gm") or Levenberg Marquard ("lm") (default: "gn").
* `~patch_size`: Length of a patch (default: 32 cells).
* `~covariance_blend`: Blending factor ([0,1]) between optimization covariance and sampling covariance (default: 0.0).
* `~use_map_topic`: True to subscribe to the `/map` topic instead of requesting the map through the "`static_map`" service (default: `false`).
* `~first_map_only`: True to use only the first map ever received (default: `false`).
* `~use_pose_on_new_map`: True to use the current algorithm pose when the map changes (default: `false`).
* `~force_update_on_initial_pose`: True to trigger a no-motion update when an initial pose is received (default: `false`)
* `~gloc_particles`: Number of particles used to find the best global localization (default: 3000)
* `~gloc_thresh`: Value at which a global localization particle is considered viable. (default: 0.15 RMSE)
* `~gloc_iters`: Maximum number of iterations executed by the global localization procedure (default: 20)
* `~do_global_loc`: True to trigger an initial global localization (default: `false`)


