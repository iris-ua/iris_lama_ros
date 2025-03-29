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

#include "tf2_ros/create_timer_ros.h"
#include "tf2/utils.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "lama/ros/graph_slam2d_ros.h"

using std::placeholders::_1;
using std::placeholders::_2;

 lama::GraphSlam2DROS::GraphSlam2DROS(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions().use_intra_process_comms(false))
: rclcpp::Node("graph_slam2d_ros", node_options)
, transform_tolerance_(0, 100000000) {
    Pose2D initial_pose;
    double map_publish_period;

    this->declare_parameter("global_frame_id", "map");
    this->declare_parameter("odom_frame_id", "odom");
    this->declare_parameter("base_frame_id", "base_link");
    this->declare_parameter("initial_pos_x", 0.0);
    this->declare_parameter("initial_pos_y", 0.0);
    this->declare_parameter("initial_pos_a", 0.0);
    this->declare_parameter("max_range", 16.0);
    this->declare_parameter("min_range", 0.0);
    this->declare_parameter("beam_step", 1);
    this->declare_parameter("publish_tf", true);
    this->declare_parameter("publish_graph", true);
    this->declare_parameter("transform_tolerance", 0.5);
    this->declare_parameter("map_publish_period", 5.0);

    this->declare_parameter("d_thresh", 0.25);
    this->declare_parameter("a_thresh", 0.25);
    this->declare_parameter("l2_max", 0.5);
    this->declare_parameter("resolution", 0.05);
    this->declare_parameter("strategy", "gn");
    this->declare_parameter("key_pose_distance", 0.5);
    this->declare_parameter("key_pose_angular_distance", 0.5 * M_PI);
    this->declare_parameter("key_pose_head_delay", 3);
    this->declare_parameter("loop_search_max_distance", 15.0);
    this->declare_parameter("loop_search_min_distance", 5.0);
    this->declare_parameter("loop_closure_scan_rmse", 0.075);
    this->declare_parameter("loop_max_candidates", 5);
    this->declare_parameter("ignore_n_chain_poses", 20);
    this->declare_parameter("max_iterations", 100);
    this->declare_parameter("patch_size", 32);

    global_frame_ = this->get_parameter("global_frame_id").as_string();
    odom_frame_ = this->get_parameter("odom_frame_id").as_string();
    base_frame_ = this->get_parameter("base_frame_id").as_string();
    initial_pose = Pose2D(
        this->get_parameter("initial_pos_x").as_double(), 
        this->get_parameter("initial_pos_y").as_double(), 
        this->get_parameter("initial_pos_a").as_double()
    );
    max_range_ = this->get_parameter("max_range").as_double();
    min_range_ = this->get_parameter("min_range").as_double();
    beam_step_ = this->get_parameter("beam_step").as_int();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    publish_graph_ = this->get_parameter("publish_graph").as_bool();
    transform_tolerance_ = rclcpp::Duration::from_seconds(this->get_parameter("transform_tolerance").as_double());

    map_publish_period = this->get_parameter("map_publish_period").as_double();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    GraphSlam2D::Options slam_options;
    slam_options.trans_thresh = this->get_parameter("d_thresh").as_double();
    slam_options.rot_thresh = this->get_parameter("a_thresh").as_double();
    slam_options.l2_max = this->get_parameter("l2_max").as_double();
    slam_options.resolution = this->get_parameter("resolution").as_double();
    slam_options.strategy = this->get_parameter("strategy").as_string();
    slam_options.key_pose_distance = this->get_parameter("key_pose_distance").as_double();
    slam_options.key_pose_angular_distance = this->get_parameter("key_pose_angular_distance").as_double();
    slam_options.key_pose_head_delay = this->get_parameter("key_pose_head_delay").as_int();
    slam_options.loop_search_max_distance = this->get_parameter("loop_search_max_distance").as_double();
    slam_options.loop_search_min_distance = this->get_parameter("loop_search_min_distance").as_double();
    slam_options.loop_closure_scan_rmse = this->get_parameter("loop_closure_scan_rmse").as_double();
    slam_options.loop_max_candidates = this->get_parameter("loop_max_candidates").as_int();
    slam_options.ignore_n_chain_poses = this->get_parameter("ignore_n_chain_poses").as_int();
    slam_options.max_iter = this->get_parameter("max_iterations").as_int();
    slam_options.patch_size = this->get_parameter("patch_size").as_int();

    slam2d_ = std::make_unique<GraphSlam2D>(slam_options);
    slam2d_->Init(initial_pose);

    markers_manager_ = std::make_unique<lama_utils::MarkersManager2D<GraphSlam2D>>(*(slam2d_.get()), global_frame_);

    data_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
        this, "scan", rclcpp::QoS(rclcpp::SystemDefaultsQoS()).keep_last(100).get_rmw_qos_profile());
    tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
        *data_sub_, *tf_buffer_, "odom", 20, this->get_node_logging_interface(),
        this->get_node_clock_interface());
    tf2_filter_->registerCallback(&lama::GraphSlam2DROS::slamExecutionCallback, this);

    if(map_publish_period > 0.0)
    {
        periodic_map_publish_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::milliseconds(static_cast<int>(map_publish_period * 1000.0))),
            std::bind(&lama::GraphSlam2DROS::mapPublishCallback, this));
    }

    rclcpp::QoS map_qos(1);
    map_qos.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    map_qos.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", map_qos);

    transient_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("transient_map", 1);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 2);
    
    dist_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("distance", 1);

    graph_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("graph", 1);

    ss_ = this->create_service<nav_msgs::srv::GetMap>("dynamic_map", std::bind(&lama::GraphSlam2DROS::getMapServiceCallback, this, _1, _2));
}

lama::GraphSlam2DROS::~GraphSlam2DROS() {

}

void lama::GraphSlam2DROS::slamExecutionCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan) {
    try {
        Pose2D odometry = getOdometry(laser_scan->header.stamp);

        // Publish the last transform if nothing has changed.
        if(!(slam2d_->enoughMotion(odometry))) {
            if(publish_tf_) {
                auto transform_map_to_odom = lama_utils::createTransformStamped(
                    latest_tf_odom_to_map_, 
                    rclcpp::Time(laser_scan->header.stamp) + transform_tolerance_, 
                    global_frame_, 
                    odom_frame_);
                tf_broadcaster_->sendTransform(transform_map_to_odom);
            }

            return;
        }

        auto transform_scan_to_base = tf_buffer_->lookupTransform(base_frame_, laser_scan->header.frame_id, laser_scan->header.stamp, rclcpp::Duration::from_nanoseconds(1));
        auto cloud = lama_utils::createPointCloud(*laser_scan, transform_scan_to_base.transform, beam_step_, min_range_, max_range_);
        
        auto start_time = this->get_clock()->now();
        slam2d_->update(cloud, odometry, static_cast<double>(laser_scan->header.stamp.sec) + static_cast<double>(laser_scan->header.stamp.nanosec) * 0.001 * 0.001);
        auto end_time = this->get_clock()->now();
        RCLCPP_DEBUG(this->get_logger(), "Update time: %lf ms", static_cast<double>((end_time - start_time).nanoseconds()) / 1000.0 / 1000.0);
        
        // Update the transform and publish it
        if(publish_tf_) {
            // Compute transform from odom to base
            auto transform_odom_to_base = tf_buffer_->lookupTransform(base_frame_, odom_frame_, laser_scan->header.stamp, rclcpp::Duration::from_nanoseconds(1));
            tf2::Transform tf_odom_to_base;
            tf2::fromMsg(transform_odom_to_base.transform, tf_odom_to_base);

            // Compute transform from base to map
            auto tf_base_to_map = lama_utils::createTransform(slam2d_->getPose());
            
            // Compute transform from odom to map
            latest_tf_odom_to_map_ = tf_odom_to_base * tf_base_to_map;

            auto transform_map_to_odom = lama_utils::createTransformStamped(
                latest_tf_odom_to_map_, 
                rclcpp::Time(laser_scan->header.stamp) + transform_tolerance_, 
                global_frame_, 
                odom_frame_);
            tf_broadcaster_->sendTransform(transform_map_to_odom);
        }

        if(publish_graph_) {
            markers_manager_->update(laser_scan->header.stamp);
            graph_pub_->publish(markers_manager_->getMarkers());
        }

    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN(this->get_logger(), "%s", e.what());
        return;
    }
}

void lama::GraphSlam2DROS::mapPublishCallback() {
    auto stamp = this->get_clock()->now();

    auto map = lama_utils::createOccupancyGrid(*(slam2d_->generateOccupancyMap(true).get()), global_frame_, stamp);
    map_pub_->publish(map);

    auto transient_map = lama_utils::createOccupancyGrid(*(slam2d_->slam->getOccupancyMap()), global_frame_, stamp);
    transient_map_pub_->publish(transient_map);
}

void lama::GraphSlam2DROS::getMapServiceCallback(const std::shared_ptr<nav_msgs::srv::GetMap::Request> request, std::shared_ptr<nav_msgs::srv::GetMap::Response> response) {
    static_cast<void>(request);  // To suppress compiler warning

    // Make sure the graph is optimized
    slam2d_->optimizePoseGraph();

    response->map = lama_utils::createOccupancyGrid(*(slam2d_->generateOccupancyMap(true).get()), global_frame_, this->get_clock()->now());
}

/**
 * @brief Get odometry data at specified time
 * 
 * @param stamp Timestamp we want to look up
 * @return lama::Pose2D 
 */
lama::Pose2D lama::GraphSlam2DROS::getOdometry(const rclcpp::Time& stamp) {
    geometry_msgs::msg::PoseStamped identity;
    geometry_msgs::msg::PoseStamped odom_pose;

    identity.header.frame_id = base_frame_;
    identity.header.stamp = stamp;
    identity.pose = geometry_msgs::msg::Pose();
    tf_buffer_->transform(identity, odom_pose, odom_frame_);

    return Pose2D(odom_pose.pose.position.x, odom_pose.pose.position.y, tf2::getYaw(odom_pose.pose.orientation));
}