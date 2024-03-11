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

#include <lama/image.h>
#include <lama/pose3d.h>

#include "lama/ros/lidar_odometry2d_ros.h"
#include "lama/ros/offline_replay.h"

lama::LidarOdometry2DROS::LidarOdometry2DROS()
    : nh_(), pnh_("~")
{
    // Load parameters from the server.
    double tmp;

    pnh_.param("odom_frame_id",   odom_frame_id_,   std::string("odom"));
    pnh_.param("base_frame_id",   base_frame_id_,   std::string("base_link"));

    pnh_.param("scan_topic", scan_topic_, std::string("/scan"));

    pnh_.param("transform_tolerance", tmp, 0.1); transform_tolerance_.fromSec(tmp);

    Vector2d pos;
    pnh_.param("initial_pos_x", pos[0], 0.0);
    pnh_.param("initial_pos_y", pos[1], 0.0);
    pnh_.param("initial_pos_a", tmp, 0.0);
    Pose2D prior(pos, tmp);

    LidarOdometry2D::Options options;
    // pnh_.param("d_thresh",   options.trans_thresh, 0.01);
    // pnh_.param("a_thresh",   options.rot_thresh,   0.25);
    // pnh_.param("l2_max",     options.l2_max,        0.5);
    // pnh_.param("truncate",   options.truncated_ray, 0.0);
    // pnh_.param("truncate_range",   options.truncated_range, 0.0);
    pnh_.param("resolution", options.resolution,   0.05);
    pnh_.param("mrange",   max_range_, 0.0);
    pnh_.param("beam_step", beam_step_, 1);

    beam_step_ = std::max(1, beam_step_);

    int itmp;
    options.max_iter = pnh_.param("max_iterations", 100);

    pnh_.param("map_publish_period", tmp, 5.0 );
    periodic_publish_ = nh_.createTimer(ros::Duration(tmp),
                                        &LidarOdometry2DROS::publishCallback, this);

    lidar_odometry = new LidarOdometry2D(options);
    lidar_odometry->odom = prior;

    // Setup TF workers ...
    tf_ = new tf::TransformListener(pnh_, ros::Duration(30));
    tfb_= new tf::TransformBroadcaster();

    // // Syncronized LaserScan messages with odometry transforms. This ensures that an odometry transformation
    // // exists when the handler of a LaserScan message is called.
    // laser_scan_sub_    = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 100, ros::TransportHints().tcpNoDelay());
    // laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, *tf_, base_frame_id_, 100);
    // laser_scan_filter_->registerCallback(boost::bind(&LidarOdometry2DROS::onLaserScan, this, _1));

    laser_scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(scan_topic_, 10, &LidarOdometry2DROS::onLaserScan, this,
                                                            ros::TransportHints().tcpNoDelay());

    // Setup publishers
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 2);
    map_pub_  = nh_.advertise<nav_msgs::OccupancyGrid>("map",      1, true);

    ros_occ_.header.frame_id = odom_frame_id_;

    ROS_INFO("Online Lidar Odometry node up and running");
}

lama::LidarOdometry2DROS::~LidarOdometry2DROS()
{
    delete lidar_odometry;
}

void lama::LidarOdometry2DROS::onLaserScan(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
    int laser_index = -1;

    // verify if it is from a known source
    if ( frame_to_laser_.find( laser_scan->header.frame_id ) == frame_to_laser_.end() ){
        if (not initLaser(laser_scan))
            return;

        laser_index = frame_to_laser_[laser_scan->header.frame_id];
    }else{
        laser_index = frame_to_laser_[laser_scan->header.frame_id];
    }

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

    cloud->sensor_origin_ = lasers_origin_[laser_index].xyz();
    cloud->sensor_orientation_ = Quaterniond(lasers_origin_[laser_index].state.so3().matrix());

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

    lidar_odometry->update(cloud, laser_scan->header.stamp.toSec());

    Pose2D pose = lidar_odometry->odom;
    // publish the pose
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.frame_id = odom_frame_id_;
    msg.header.stamp = laser_scan->header.stamp;
    msg.pose.pose.position.x = pose.x();
    msg.pose.pose.position.y = pose.y();
    msg.pose.pose.position.z = 0.0;

    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose.rotation());

    // TODO: Pose covariance.
    pose_pub_.publish(msg);

    tf::Transform tmp_tf(tf::createQuaternionFromYaw(pose.rotation()), tf::Vector3(pose.x(), pose.y(), 0));
    tf::Stamped<tf::Pose> base_to_odom (tmp_tf, laser_scan->header.stamp, base_frame_id_);

    latest_tf_ = tf::Transform(tf::Quaternion(base_to_odom.getRotation()),
                               tf::Point(base_to_odom.getOrigin()));

    // We want to send a transform that is good up until a
    // tolerance time so that odom can be used
    ros::Time transform_expiration = (laser_scan->header.stamp + transform_tolerance_);
    tf::StampedTransform tmp_tf_stamped(latest_tf_,
                                        transform_expiration,
                                        odom_frame_id_, base_frame_id_);
    tfb_->sendTransform(tmp_tf_stamped);
}

bool lama::LidarOdometry2DROS::initLaser(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
    // find the origin of the sensor in the base frame
    tf::Stamped<tf::Pose> identity(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0)),
                                   ros::Time(), laser_scan->header.frame_id);
    tf::Stamped<tf::Pose> laser_origin;
    try{ tf_->transformPose(base_frame_id_, identity, laser_origin); }
    catch(tf::TransformException& e)
    { ROS_ERROR("Could not find origin of %s", laser_scan->header.frame_id.c_str()); return false; }

    // Validate laser orientation (code taken from slam_gmapping)
    // create a point 1m above the laser position and transform it into the laser-frame
    tf::Vector3 v;
    v.setValue(0, 0, 1 + laser_origin.getOrigin().z());
    tf::Stamped<tf::Vector3> up(v, laser_scan->header.stamp, base_frame_id_);
    try {
        tf_->transformPoint(laser_scan->header.frame_id, up, up);
        ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
    } catch(tf::TransformException& e) {
        ROS_ERROR("Unable to determine orientation of laser: %s", e.what());
        return false;
    }

    // we do not take roll or pitch into account. So check for correct sensor alignment.
    if (std::fabs(std::fabs(up.z()) - 1) > 0.001) {
        ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f", up.z());
        return false;
    }

    double roll, pitch, yaw;
    laser_origin.getBasis().getRPY(roll, pitch, yaw);
    Pose3D lp(laser_origin.getOrigin().x(), laser_origin.getOrigin().y(), 0,
              roll, pitch, yaw);

    lasers_origin_.push_back( lp );

    int laser_index = (int)frame_to_laser_.size();  // simple ID generator :)
    frame_to_laser_[laser_scan->header.frame_id] = laser_index;

    ROS_INFO("New laser configured (id=%d frame_id=%s)", laser_index, laser_scan->header.frame_id.c_str() );
    return true;
}

bool lama::LidarOdometry2DROS::OccupancyMsgFromOccupancyMap(nav_msgs::OccupancyGrid& msg)
{
    if (lidar_odometry->occupancy_map == 0)
        return false;

    const ProbabilisticOccupancyMap* map = new ProbabilisticOccupancyMap(*lidar_odometry->occupancy_map);
    Vector3ui imin, imax;
    map->bounds(imin, imax);

    unsigned int width = imax(0) - imin(0);
    unsigned int height= imax(1) - imin(1);

    if (width == 0 || height == 0)
        return false;

    if ( width*height != msg.data.size() )
        msg.data.resize(width*height);

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

    delete map;
    return true;
}

void lama::LidarOdometry2DROS::publishCallback(const ros::TimerEvent &)
{
    auto time = ros::Time::now();
    if ( map_pub_.getNumSubscribers() > 0 ) {
        OccupancyMsgFromOccupancyMap(ros_occ_);
        ros_occ_.header.stamp = time;
        map_pub_.publish(ros_occ_);
    }
}

void lama::LidarOdometry2DROS::publishMaps()
{
    auto time = ros::Time::now();

    OccupancyMsgFromOccupancyMap(ros_occ_);
    ros_occ_.header.stamp = time;
    map_pub_.publish(ros_occ_);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lidar_odometryros");
    lama::LidarOdometry2DROS lidar_odometry;

    ros::NodeHandle pnh("~");
    std::string rosbag_filename;

    if( !pnh.getParam("rosbag", rosbag_filename ) || rosbag_filename.empty()) {
        ROS_INFO("Running Lidar Odometry in Live Mode");
    } else{
        ROS_INFO("Running Lidar Odometry in Rosbag Mode (offline)");
        lama::ReplayRosbag(pnh, rosbag_filename);

        // publish the maps a last time
        lidar_odometry.publishMaps();
    }

    ros::spin();
    return 0;
}

