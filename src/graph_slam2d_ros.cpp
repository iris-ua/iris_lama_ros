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

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <tf2/utils.h>

#include <lama/image.h>
#include <lama/print.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "lama/ros/graph_slam2d_ros.h"
#include "lama/ros/offline_replay.h"

lama::GraphSlam2DROS::GraphSlam2DROS()
    : BaseROSNode<sensor_msgs::LaserScan>(ros::NodeHandle("~")), nh_(), pnh_("~")
{
    // Load parameters from the server.
    double tmp;

    Vector2d pos;
    pnh_.param("initial_pos_x", pos[0], 0.0);
    pnh_.param("initial_pos_y", pos[1], 0.0);
    pnh_.param("initial_pos_a", tmp, 0.0);
    Pose2D prior(pos, tmp);

    GraphSlam2D::Options options;
    pnh_.param("d_thresh",   options.trans_thresh, 0.01);
    pnh_.param("a_thresh",   options.rot_thresh,   0.25);
    pnh_.param("l2_max",     options.l2_max,        0.5);
    pnh_.param("resolution", options.resolution,   0.05);
    pnh_.param("strategy",   options.strategy, std::string("gn"));

    pnh_.param("key_pose_distance",         options.key_pose_distance, 1.0);
    pnh_.param("key_pose_angular_distance", options.key_pose_angular_distance, 0.5 * M_PI);

    pnh_.param("loop_search_max_distance", options.loop_search_max_distance, 15.0);
    pnh_.param("loop_search_min_distance", options.loop_search_min_distance,  5.0);
    pnh_.param("loop_closure_scan_rmse",   options.loop_closure_scan_rmse,   0.05);
    pnh_.param("ignore_n_chain_poses",     options.ignore_n_chain_poses,       20);

    /* pnh_.param("truncate",   options.truncated_ray, 0.0); */
    /* pnh_.param("truncate_range",   options.truncated_range, 0.0); */

    /* pnh_.param("use_compression",       options.use_compression, false); */
    /* pnh_.param("compression_algorithm", options.calgorithm, std::string("lz4")); */

    int itmp;
    pnh_.param("max_iterations", itmp, 100); options.max_iter   = itmp;
    pnh_.param("patch_size",     itmp,  32); options.patch_size = itmp;
    /* pnh_.param("cache_size",     itmp, 100); options.cache_size = itmp; */

    pnh_.param("mrange",   max_range_, 16.0);
    pnh_.param("beam_step", beam_step_, 1);
    beam_step_ = std::max(1, beam_step_);
    /* pnh_.param("create_summary", options.create_summary, false); */

    pnh_.param("map_publish_period", tmp, 5.0 );
    periodic_publish_ = nh_.createTimer(ros::Duration(tmp),
                                        &GraphSlam2DROS::publishCallback, this);

    slam2d_ = new GraphSlam2D(options);
    slam2d_->Init(prior);

    // Setup publishers
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 2);
    map_pub_  = nh_.advertise<nav_msgs::OccupancyGrid>("map",      1, true);
    dist_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("distance", 1, true);
    kp_pub_   = nh_.advertise<visualization_msgs::MarkerArray>("keypoints", 1, true);

    transient_map_pub_  = nh_.advertise<nav_msgs::OccupancyGrid>("transient_map",      1, true);

    ros_occ_.header.frame_id = global_frame;
    ros_cost_.header.frame_id = global_frame;

    // Setup service
    ss_ = nh_.advertiseService("dynamic_map", &GraphSlam2DROS::onGetMap, this);

    ROS_INFO("Online SLAM node up and running");
}

lama::GraphSlam2DROS::~GraphSlam2DROS()
{
}

void lama::GraphSlam2DROS::onData(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
    Pose3D sensor_origin = getSensorPose(laser_scan->header.frame_id);

    Pose2D odom;
    auto has_odom = getOdometry(odom, laser_scan->header.stamp);
    if (not has_odom)
        return;


    bool update = slam2d_->enoughMotion(odom);
    if ( update ){

        size_t size = laser_scan->ranges.size();

        float max_range;
        if (max_range_ == 0.0 || max_range_ > laser_scan->range_max)
            max_range = laser_scan->range_max;
        else
            max_range = max_range_;

        float min_range = laser_scan->range_min;
        float angle_min = laser_scan->angle_min;
        float angle_inc = laser_scan->angle_increment;

        PointCloudXYZ::Ptr cloud(new PointCloudXYZ);

        cloud->sensor_origin_ = sensor_origin.xyz();
        cloud->sensor_orientation_ = Quaterniond(sensor_origin.state.so3().matrix());

        cloud->points.reserve(laser_scan->ranges.size());
        for(size_t i = 0; i < size; i += beam_step_){
            const double range = laser_scan->ranges[i];

            if (not std::isfinite(range))
                continue;

            if (range >= max_range || range <= min_range)
                continue;

            Eigen::Vector3d point;
            point << range * std::cos(angle_min+(i*angle_inc)),
                     range * std::sin(angle_min+(i*angle_inc)),
                     0;
            cloud->points.push_back( point );
        }

        ros::WallTime start = ros::WallTime::now();

        slam2d_->update(cloud, odom, laser_scan->header.stamp.toSec());

        auto elapsed = (ros::WallTime::now() - start).toSec() * 1000.0;
        if (elapsed > 5.0)
            ROS_INFO("Update time %f ms", elapsed);

        //====
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker marker;

        marker.header.frame_id = global_frame;
        marker.header.stamp = laser_scan->header.stamp;
        marker.ns = "poses";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        if (slam2d_->key_poses.size() > 0){

            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.25;
            marker.scale.y = 0.25;
            marker.scale.z = 0.25;

            std_msgs::ColorRGBA color;
            color.b = 0.0;
            color.a = 1.0;

            const int num_poses = slam2d_->key_poses.size();
            for (auto& kp : slam2d_->key_poses){

                double a = kp.id / (double)num_poses;
                geometry_msgs::Point p;
                p.x = kp.pose.x(); p.y = kp.pose.y(); p.z = 0.0;
                color.r = a;
                color.g = 1 - a;
                marker.points.push_back(p);
                marker.colors.push_back(color);
            }// end for
        }// end if
        markers.markers.push_back(marker);

        marker.ns = "edges";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
        marker.color.a = 1;
        marker.colors.clear();

        markers.markers.push_back(marker);

        marker.ns = "links";
        marker.id = 2;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.points.clear();
        marker.color.g = 1;
        marker.color.b = 0;

        for (auto& link : slam2d_->links){
            geometry_msgs::Point p;
            p.x = slam2d_->key_poses[link.first].pose.x();
            p.y = slam2d_->key_poses[link.first].pose.y();
            p.z = 0.0;
            marker.points.push_back(p);

            p.x = slam2d_->key_poses[link.second].pose.x();
            p.y = slam2d_->key_poses[link.second].pose.y();
            p.z = 0.0;
            marker.points.push_back(p);
        }
        markers.markers.push_back(marker);

        if (not slam2d_->links.empty()){
            auto link = slam2d_->links.back();

            marker.ns = "cloud0";
            marker.id = 3;
            marker.type = visualization_msgs::Marker::POINTS;
            marker.points.clear();
            marker.color.r = 1;
            marker.color.g = 0;
            marker.color.b = 1;

            for (auto& point : slam2d_->key_poses[link.first].cloud->points){
                geometry_msgs::Point p;
                auto tp = slam2d_->key_poses[link.first].pose * point.head<2>();
                p.x = tp.x();
                p.y = tp.y();
                p.z = 0.0;

                marker.points.push_back(p);
            }

            markers.markers.push_back(marker);

        }

        if (slam2d_->failure != -1){

            marker.ns = "cloud-failure";
            marker.id = 4;
            marker.type = visualization_msgs::Marker::POINTS;
            marker.points.clear();
            marker.color.r = 0;
            marker.color.g = 1;
            marker.color.b = 1;

            auto& cloud = slam2d_->key_poses[slam2d_->failure].cloud;
            for (auto& point : cloud->points){
                geometry_msgs::Point p;
                auto tp = slam2d_->key_poses[slam2d_->failure].pose *
                    ((Translation3d(cloud->sensor_origin_) * cloud->sensor_orientation_) * point).head<2>();
                p.x = tp.x();
                p.y = tp.y();
                p.z = 0.0;

                marker.points.push_back(p);
            }
        }// end if
        markers.markers.push_back(marker);

        kp_pub_.publish(markers);

        //====

        Pose2D pose = slam2d_->getPose();

        publishTF(pose, laser_scan->header.stamp);

    } else {
        // Nothing has change, therefore, republish the last transform.
        publishTF(laser_scan->header.stamp);
    } // end if (update)
}

void lama::GraphSlam2DROS::saveLog()
{
    std::ofstream log("slam.log");

    for (auto& node : slam2d_->key_poses){

        log << lama::format("FLASER 0 0 0 0 %f %f %f %f\n",
                node.pose.x(), node.pose.y(), node.pose.rotation(), node.timestamp);

    }// end for

    log.close();

}

bool lama::GraphSlam2DROS::OccupancyMsgFromOccupancyMap(nav_msgs::OccupancyGrid& msg, const lama::OccupancyMap* occ)
{
    auto map = occ;
    Vector3ui imin, imax;
    map->bounds(imin, imax);

    unsigned int width = imax(0) - imin(0);
    unsigned int height= imax(1) - imin(1);

    if (width == 0 || height == 0)
        return false;

    if ( width*height != msg.data.size() ){
        msg.data.clear();
        msg.data.resize(width*height);
    }

    Image image;
    image.alloc(width, height, 1);
    image.fill(50);

    map->visit_all_cells([&image, &map, &imin](const Vector3ui& coords){
        Vector3ui adj_coords = coords - imin;

        if (map->isFree(coords))
            image(adj_coords(0), adj_coords(1)) = 0;
        else if (map->isOccupied(coords))
            image(adj_coords(0), adj_coords(1)) = 100;
        else
            image(adj_coords(0), adj_coords(1)) = 0xff;
    });

    memcpy(&msg.data[0], image.data.get(), width*height);

    msg.info.width = width;
    msg.info.height = height;
    msg.info.resolution = map->resolution;

    Vector3d pos = map->m2w(imin);
    msg.info.origin.position.x = pos.x();
    msg.info.origin.position.y = pos.y();
    msg.info.origin.position.z = 0;
    msg.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);

    return true;
}


void lama::GraphSlam2DROS::publishCallback(const ros::TimerEvent &)
{
    auto time = ros::Time::now();
    if ( map_pub_.getNumSubscribers() > 0 )
    {
      OccupancyMsgFromOccupancyMap(ros_occ_, slam2d_->generateOccupancyMap().get() );
      ros_occ_.header.stamp = time;
      map_pub_.publish(ros_occ_);
    }

    if ( transient_map_pub_.getNumSubscribers() > 0 )
    {
      OccupancyMsgFromOccupancyMap(ros_occ_, slam2d_->slam->getOccupancyMap() );
      ros_occ_.header.stamp = time;
      transient_map_pub_.publish(ros_occ_);
    }
}

bool lama::GraphSlam2DROS::onGetMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
    res.map.header.frame_id = global_frame;
    res.map.header.stamp = ros::Time::now();

    OccupancyMsgFromOccupancyMap(res.map, slam2d_->generateOccupancyMap().get());

    return true;
}

void lama::GraphSlam2DROS::publishMaps()
{
    auto time = ros::Time::now();

    OccupancyMsgFromOccupancyMap(ros_occ_, slam2d_->generateOccupancyMap().get());
    ros_occ_.header.stamp = time;
    map_pub_.publish(ros_occ_);

}

void lama::GraphSlam2DROS::printSummary()
{
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "graph_slam2d_ros");
    lama::GraphSlam2DROS slam2d_ros;

    ros::NodeHandle pnh("~");
    std::string rosbag_filename;

    if( !pnh.getParam("rosbag", rosbag_filename ) || rosbag_filename.empty()) {
        ROS_INFO("Running SLAM in Live Mode");
    } else{
        ROS_INFO("Running SLAM in Rosbag Mode (offline)");
        /* lama::ReplayRosbag(pnh, rosbag_filename); */

        slam2d_ros.runFromBag(rosbag_filename);
        slam2d_ros.optimizePoseGraph();
        slam2d_ros.saveLog();

        ROS_INFO("You can now save your map. Use ctrl-c to quit.");
        // publish the maps a last time
        slam2d_ros.publishMaps();
    }

    ros::spin();
    return 0;
}

