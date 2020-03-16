/*
 * IRIS Localization and Mapping (LaMa) for ROS
 *
 * Copyright (c) 2019-today, Eurico Pedrosa, University of Aveiro - Portugal
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Aveiro nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

// ROS includes
#include "rclcpp/rclcpp.hpp"

// Transform include
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>

#include <message_filters/subscriber.h>

// Pose publishing
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
// Laser message
//#include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/msg/laser_scan.hpp"
// maps
//#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/msg/occupancy_grid.hpp"
//#include <nav_msgs/GetMap.h>
#include "nav_msgs/srv/get_map.hpp"

#include <lama/slam2d.h>
#include <lama/pose3d.h>

namespace lama {

class Slam2DROS {
public:

    Slam2DROS(std::string name);
    ~Slam2DROS();

    void onLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);
    // TODO change signature https://github.com/ros2/examples/blob/master/rclcpp/minimal_service/main.cpp
    bool onGetMap(nav_msgs::srv::GetMap::Request &req, nav_msgs::srv::GetMap::Response &res);
    void publishMaps();

    void printSummary();

    std::shared_ptr<rclcpp::Node> nh;

private:

    bool initLaser(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);

    bool OccupancyMsgFromOccupancyMap(nav_msgs::msg::OccupancyGrid& msg);
    bool DistanceMsgFromOccupancyMap(nav_msgs::msg::OccupancyGrid& msg);

    void publishCallback();  // const rclcpp::TimerEvent &
private:

    // == ROS stuff ==
    //rclcpp::Node nh_;  ///< Root ros node handle.
    rclcpp::Node pnh_; ///< Private ros node handle.

    rclcpp::Clock ros_clock;
    rclcpp::TimerBase::SharedPtr periodic_publish_;
    //rclcpp::Timer periodic_publish_; /// timer user to publish periodically the maps

    tf2_ros::TransformBroadcaster* tfb_; ///< Position transform broadcaster.
    tf2_ros::TransformListener*    tf_;  ///< Gloabal transform listener.

    tf2_ros::Buffer latest_tf_; ///< The most recent transform.

    tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>*           laser_scan_filter_; ///< Transform and LaserScan message Syncronizer.
    message_filters::Subscriber<sensor_msgs::msg::LaserScan>* laser_scan_sub_;    ///< Subscriber to the LaserScan message.

    // publishers   
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;      ///< Publisher of the pose with covariance.
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dist_pub_;

    /*rclcpp::Publisher pose_pub_;   ///< Publisher of the pose with covariance.
    rclcpp::Publisher map_pub_;
    rclcpp::Publisher dist_pub_;*/

    rclcpp::ServiceServer ss_;

    // == Laser stuff ==
    // allow to handle multiple lasers at once
    std::map<std::string, int> frame_to_laser_; ///< Map with the known lasers.
    std::vector<bool>          laser_is_reversed_;  ///< Vector that signals if the laser is reversed
    std::vector<Pose3D>        lasers_origin_;  ///< Laser origin transformation
    double max_range_;

    // maps
    nav_msgs::msg::OccupancyGrid ros_occ_;
    nav_msgs::msg::OccupancyGrid ros_cost_;

    // == configuration variables ==
    std::string global_frame_id_;       ///< Global frame id, usualy the map frame.
    std::string odom_frame_id_;         ///< Odometry frame id.
    std::string base_frame_id_;         ///< Robot base frame.

    std::string scan_topic_;   ///< LaserScan message topic.

    rclcpp::Duration transform_tolerance_;   ///< Defines how long map->odom transform is good for.

    // == Inner state ==
    Slam2D*   slam2d_;
    Pose2D    odom_;
};

} /* lama */

