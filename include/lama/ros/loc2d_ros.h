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
// Force triggering a no-motion update and global localization.
#include <std_srvs/Empty.h>

#include <lama/pose3d.h>
#include <lama/loc2d.h>

#include <lama/sdm/simple_occupancy_map.h>
#include <lama/sdm/dynamic_distance_map.h>

namespace lama {

class Loc2DROS {
public:

    Loc2DROS();
    ~Loc2DROS();

    void onInitialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& initial_pose);
    void onInitialPose(const Pose2D& prior);
    void onLaserScan(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void onMapReceived(const nav_msgs::OccupancyGridConstPtr& msg);

    //bool onGetMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);

private:

    // keep old API
    inline void InitLoc2DFromOccupancyGridMsg(const nav_msgs::OccupancyGrid& msg)
    {
        InitLoc2DFromOccupancyGridMsg(initial_prior_, msg);
    }

    void InitLoc2DFromOccupancyGridMsg(const Pose2D& prior, const nav_msgs::OccupancyGrid& msg);

    bool initLaser(const sensor_msgs::LaserScanConstPtr& laser_scan);

    // A localization update can be forced by an external trigger.
    bool onTriggerUpdate(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    // Trigger a global localization procedure.
    bool globalLocalizationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

    void publishCurrentPose();

private:

    // == ROS stuff ==
    ros::NodeHandle nh_;  ///< Root ros node handle.
    ros::NodeHandle pnh_; ///< Private ros node handle.

    tf::TransformBroadcaster* tfb_; ///< Position transform broadcaster.
    tf::TransformListener*    tf_;  ///< Gloabal transform listener.

    tf::Transform latest_tf_; ///< The most recent transform.
    ros::Duration transform_tolerance_;   ///< Defines how long map->odom transform is good for.

    tf::MessageFilter<sensor_msgs::LaserScan>*           laser_scan_filter_; ///< Transform and LaserScan message Syncronizer.
    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;    ///< Subscriber to the LaserScan message.

    // Publishers
    ros::Publisher pose_pub_; ///< Publishers of the pose with covariance

    // Subscribers
    ros::Subscriber pose_sub_;   ///< Subscriber of the initial pose (with covariance)
    ros::Subscriber map_sub_; ///< Subscriber of the map; used if \p use_map_topic_ is true.

    // Service providers
    ros::ServiceServer srv_update_; ///< Service to trigger a scan match
    ros::ServiceServer srv_global_loc_; ///< Used to trigger global localization on request.

    // == Laser stuff ==
    // Handle multiple lasers at once
    std::map<std::string, int> frame_to_laser_; ///< Map with the known lasers.
    std::vector<Pose3D>        lasers_origin_;  ///< Laser origin transformation

    // == configuration variables ==
    std::string global_frame_id_;       ///< Global frame id, usualy the map frame.
    std::string odom_frame_id_;         ///< Odometry frame id.
    std::string base_frame_id_;         ///< Robot base frame.

    std::string scan_topic_;            ///< LaserScan message topic.

    double temporal_update_;            ///< Force an update when the last processed scan is older than this.

    bool publish_tf_;                   ///< True to publish the transformations.
    bool use_map_topic_;                ///< True to subscribe to the map topic instead of requesting the map through the "static_map" service
    bool first_map_only_;               ///< True to use only the first map ever received
    bool first_map_received_;           ///< True if the first map has already been received
    bool use_pose_on_new_map_;          ///< True to use the current algorithm pose when the map changes
    bool force_update_;                 ///< True to force an update when a new laser scan is received
    bool force_update_on_initial_pose_; ///< True to trigger a forced updated when an initial pose is received

    // == Inner state ==
    Loc2D   loc2d_;
    Pose2D odom_;
    Loc2D::Options options_;
    Pose2D initial_prior_;
    tf::Quaternion current_orientation_;
    geometry_msgs::PoseWithCovarianceStamped cur_pose_msg_;

    double max_range_ = 20;     ///< Maximum laser scan range
    int beam_step_ = 1;         ///< Number of beams to step (or skip) in each scan.
};

} /* lama */

