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
#include <ros/ros.h>

// Transform include
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/tf.h>

#include <message_filters/subscriber.h>

// Pose publishing
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// Laser message
#include <sensor_msgs/LaserScan.h>
// maps
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <lama/slam2d.h>
#include <lama/pose3d.h>

#include <Eigen/StdVector>

namespace lama {

class Slam2DROS {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Slam2DROS();
    ~Slam2DROS();

    void onLaserScan(const sensor_msgs::LaserScanConstPtr& laser_scan);
    bool onGetMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);
    void publishMaps();

    void printSummary();

private:

    bool initLaser(const sensor_msgs::LaserScanConstPtr& laser_scan);

    bool OccupancyMsgFromOccupancyMap(nav_msgs::OccupancyGrid& msg);
    bool DistanceMsgFromOccupancyMap(nav_msgs::OccupancyGrid& msg);

    void publishCallback(const ros::TimerEvent &);
private:

    // == ROS stuff ==
    ros::NodeHandle nh_;  ///< Root ros node handle.
    ros::NodeHandle pnh_; ///< Private ros node handle.

    ros::Timer periodic_publish_; /// timer user to publish periodically the maps
    tf::TransformListener*    tf_;  ///< Gloabal transform listener.
    tf::TransformBroadcaster* tfb_; ///< Position transform broadcaster.

    tf::Transform latest_tf_; ///< The most recent transform.

    tf::MessageFilter<sensor_msgs::LaserScan>*           laser_scan_filter_; ///< Transform and LaserScan message Syncronizer.
    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;    ///< Subscriber to the LaserScan message.

    // publishers
    ros::Publisher pose_pub_;   ///< Publisher of the pose with covariance.
    ros::Publisher map_pub_;
    ros::Publisher dist_pub_;

    ros::ServiceServer ss_;

    // == Laser stuff ==
    // allow to handle multiple lasers at once
    std::map<std::string, int> frame_to_laser_; ///< Map with the known lasers.
    std::vector<Pose3D, Eigen::aligned_allocator<Pose3D>>        lasers_origin_;  ///< Laser origin transformation
    double max_range_;
    int beam_step_;

    // maps
    nav_msgs::OccupancyGrid ros_occ_;
    nav_msgs::OccupancyGrid ros_cost_;

    // == configuration variables ==
    std::string global_frame_id_;       ///< Global frame id, usualy the map frame.
    std::string odom_frame_id_;         ///< Odometry frame id.
    std::string base_frame_id_;         ///< Robot base frame.

    std::string scan_topic_;   ///< LaserScan message topic.

    ros::Duration transform_tolerance_;   ///< Defines how long map->odom transform is good for.

    // == Inner state ==
    Slam2D*   slam2d_;
    Pose2D    odom_;
};

} /* lama */

