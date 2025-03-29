/*
 * IRIS Localization and Mapping (LaMa) for ROS
 *
 * Copyright (c) 2025-today, Eurico Pedrosa, University of Aveiro - Portugal
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
#include "rclcpp_components/register_node_macro.hpp"

// Transform includes
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"

// Map includes
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"

// Laser includes
#include "sensor_msgs/msg/laser_scan.hpp"

// Visualization includes
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "lama/graph_slam2d.h"
#include "lama_utils.h"

namespace lama {

class GraphSlam2DROS : public rclcpp::Node {
public:

    GraphSlam2DROS(const rclcpp::NodeOptions& node_options);
    ~GraphSlam2DROS();

private:
    void slamExecutionCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan);
    void mapPublishCallback();
    void getMapServiceCallback(const std::shared_ptr<nav_msgs::srv::GetMap::Request> request, std::shared_ptr<nav_msgs::srv::GetMap::Response> response);
    Pose2D getOdometry(const rclcpp::Time& stamp);

private:
    std::string global_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    float max_range_;
    float min_range_;
    int beam_step_;
    bool publish_tf_;
    bool publish_graph_;

    rclcpp::Duration transform_tolerance_;

    std::unique_ptr<GraphSlam2D> slam2d_; 

    std::unique_ptr<lama_utils::MarkersManager2D<GraphSlam2D>> markers_manager_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> data_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf2_filter_;

    tf2::Transform latest_tf_odom_to_map_;

    rclcpp::TimerBase::SharedPtr periodic_map_publish_timer_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr transient_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dist_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_pub_;

    rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr ss_;
};

} /* lama */

RCLCPP_COMPONENTS_REGISTER_NODE(lama::GraphSlam2DROS)