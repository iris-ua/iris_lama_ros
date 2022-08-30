/*
 * IRIS Localization and Mapping (LaMa) for ROS
 *
 * Copyright (c) 2022-today, Eurico Pedrosa, University of Aveiro - Portugal
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

#include <tf2/LinearMath/Transform.h>

#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <message_filters/subscriber.h>

// Pose publishing
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// Laser message
#include <sensor_msgs/LaserScan.h>
// maps
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <lama/graph_slam2d.h>
#include <lama/pose3d.h>
#include <lama/sdm/occupancy_map.h>

#include <Eigen/StdVector>

#include "base_ros_node.h"

namespace lama {

class GraphSlam2DROS : public BaseROSNode<sensor_msgs::LaserScan> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GraphSlam2DROS();
    virtual ~GraphSlam2DROS();

    virtual void onData(const sensor_msgs::LaserScan::ConstPtr& laser_scan) final;

    bool onGetMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);
    void publishMaps();

    inline void optimizePoseGraph()
    { slam2d_->optimizePoseGraph(); }

private:

    bool OccupancyMsgFromOccupancyMap(nav_msgs::OccupancyGrid& msg, const lama::OccupancyMap* occ);

    void publishCallback(const ros::TimerEvent &);
private:

    // == ROS stuff ==
    ros::NodeHandle nh_;  ///< Root ros node handle.
    ros::NodeHandle pnh_; ///< Private ros node handle.

    ros::Timer periodic_publish_; /// timer user to publish periodically the maps

    // publishers
    ros::Publisher pose_pub_;   ///< Publisher of the pose with covariance.
    ros::Publisher map_pub_;
    ros::Publisher transient_map_pub_;
    ros::Publisher dist_pub_;
    ros::Publisher graph_pub_;

    ros::ServiceServer ss_;

    // == Laser stuff ==
    double max_range_;
    double min_range_;
    int beam_step_;

    // maps
    nav_msgs::OccupancyGrid ros_occ_;
    nav_msgs::OccupancyGrid ros_cost_;

    // == Inner state ==
    GraphSlam2D* slam2d_;
    Pose2D    odom_;
};

} /* lama */

