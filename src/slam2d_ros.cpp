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

#include "lama/ros/slam2d_ros.h"
#include "lama/ros/offline_replay.h"

lama::Slam2DROS::Slam2DROS(string name)
        : pnh_("~") //nh_(),
{
    nh = rclcpp::Node::make_shared(name);
    ros_clock(RCL_ROS_TIME);

    // Load parameters from the server.
    double tmp;

    pnh_.param("global_frame_id", global_frame_id_, std::string("/map"));
    pnh_.param("odom_frame_id", odom_frame_id_, std::string("/odom"));
    pnh_.param("base_frame_id", base_frame_id_, std::string("/base_link"));

    pnh_.param("scan_topic", scan_topic_, std::string("/scan"));

    pnh_.param("transform_tolerance", tmp, 0.1);
    transform_tolerance_.fromSec(tmp);

    Vector2d pos;
    pnh_.param("initial_pos_x", pos[0], 0.0);
    pnh_.param("initial_pos_y", pos[1], 0.0);
    pnh_.param("initial_pos_a", tmp, 0.0);
    Pose2D prior(pos, tmp);

    Slam2D::Options options;
    pnh_.param("d_thresh", options.trans_thresh, 0.01);
    pnh_.param("a_thresh", options.rot_thresh, 0.25);
    pnh_.param("l2_max", options.l2_max, 0.5);
    pnh_.param("truncate", options.truncated_ray, 0.0);
    pnh_.param("resolution", options.resolution, 0.05);
    pnh_.param("strategy", options.strategy, std::string("gn"));
    pnh_.param("use_compression", options.use_compression, false);
    pnh_.param("compression_algorithm", options.calgorithm, std::string("lz4"));
    pnh_.param("mrange", max_range_, 16.0);

    int itmp;
    pnh_.param("max_iterations", itmp, 100);
    options.max_iter = itmp;
    pnh_.param("patch_size", itmp, 32);
    options.patch_size = itmp;
    pnh_.param("cache_size", itmp, 100);
    options.cache_size = itmp;

    pnh_.param("create_summary", options.create_summary, false);

    pnh_.param("map_publish_period", tmp, 5.0);
    //periodic_publish_ = nh_.createTimer(ros::Duration(tmp), &Slam2DROS::publishCallback, this);
    periodic_publish_ = nh->create_wall_timer(rclcpp::Duration(tmp), std::bind(&Slam2DROS::publishCallback,
                                                                               this)); //TODO https://docs.ros2.org/beta3/api/rclcpp/classrclcpp_1_1node_1_1Node.html

    slam2d_ = new Slam2D(options);
    slam2d_->setPose(prior);

    // Setup TF workers ...
    tf_ = new tf2_ros::TransformListener(pnh_, rclcpp::Duration(30));
    tfb_ = new tf2_ros::TransformBroadcaster();

    // Syncronized LaserScan messages with odometry transforms. This ensures that an odometry transformation
    // exists when the handler of a LaserScan message is called.
    laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::msg::LaserScan>(*nh, scan_topic_, 100);
    laser_scan_filter_ = new tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>(*laser_scan_sub_, *tf_, odom_frame_id_,
                                                                                 100);
    laser_scan_filter_->registerCallback(boost::bind(&Slam2DROS::onLaserScan, this, _1));

    // Setup publishers
    // TODO latch
    pose_pub_ = nh->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose", 2);
    map_pub_ = nh->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
    dist_pub_ = nh->create_publisher<nav_msgs::msg::OccupancyGrid>("/distance", 1);

    /*pose_pub_ = nh_.advertise<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 2);
    map_pub_  = nh_.advertise<nav_msgs::msg::OccupancyGrid>("map",      1, true);
    dist_pub_ = nh_.advertise<nav_msgs::msg::OccupancyGrid>("distance", 1, true);*/

    ros_occ_.header.frame_id = global_frame_id_;
    ros_cost_.header.frame_id = global_frame_id_;

    // Setup service
    ss_ = nh->advertiseService("dynamic_map", &Slam2DROS::onGetMap, this);

    RCLCPP_INFO(nh->get_logger(), "Online SLAM node up and running");
}

lama::Slam2DROS::~Slam2DROS() {
    delete laser_scan_filter_;
    delete laser_scan_sub_;
    delete slam2d_;
}

void lama::Slam2DROS::onLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan) {
    int laser_index = -1;

    // verify if it is from a known source
    if (frame_to_laser_.find(laser_scan->header.frame_id) == frame_to_laser_.end()) {
        if (not initLaser(laser_scan))
            return;

        laser_index = frame_to_laser_[laser_scan->header.frame_id];
    } else {
        laser_index = frame_to_laser_[laser_scan->header.frame_id];
    }

    // Where was the robot at the time of the scan ?
    tf2_ros::Stamped <tf2_ros::Pose> identity(
            tf2_ros::Transform(tf2_ros::createIdentityQuaternion(), tf2_ros::Vector3(0, 0, 0)),
            laser_scan->header.stamp, base_frame_id_);
    tf2_ros::Stamped <tf2_ros::Pose> odom_tf;
    try { tf_->transformPose(odom_frame_id_, identity, odom_tf); }
    catch (tf2_ros::TransformException &e) {
        RCLCPP_WARN(nh->get_logger(), "Failed to compute odom pose, skipping scan %s", e.what());
        return;
    }

    Pose2D odom(odom_tf.getOrigin().x(), odom_tf.getOrigin().y(),
                tf2_ros::getYaw(odom_tf.getRotation()));

    bool update = slam2d_->enoughMotion(odom);

    if (update) {

        size_t size = laser_scan->ranges.size();
        size_t beam_step = 1;

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
        for (size_t i = 0; i < size; i += beam_step) {
            double range;

            if (laser_is_reversed_[laser_index])
                range = laser_scan->ranges[size - i - 1];
            else
                range = laser_scan->ranges[i];

            if (not std::isfinite(range))
                continue;

            if (range >= max_range || range <= min_range)
                continue;


            Eigen::Vector3d point;
            point << range * std::cos(angle_min + (i * angle_inc)),
                    range * std::sin(angle_min + (i * angle_inc)),
                    0;

            cloud->points.push_back(point);
        }

        slam2d_->update(cloud, odom, laser_scan->header.stamp.toSec());

        Pose2D pose = slam2d_->getPose();
        // subtracting base to odom from map to base and send map to odom instead
        tf2_ros::Stamped <tf2_ros::Pose> odom_to_map;
        try {
            tf2_ros::Transform tmp_tf(tf2_ros::createQuaternionFromYaw(pose.rotation()),
                                      tf2_ros::Vector3(pose.x(), pose.y(), 0));
            tf2_ros::Stamped <tf2_ros::Pose> tmp_tf_stamped(tmp_tf.inverse(), laser_scan->header.stamp, base_frame_id_);
            tf_->transformPose(odom_frame_id_, tmp_tf_stamped, odom_to_map);

        } catch (tf2_ros::TransformException) {
            RCLCPP_WARN(nh->get_logger(), "Failed to subtract base to odom transform");
            return;
        }

        latest_tf_ = tf2_ros::Transform(tf2_ros::Quaternion(odom_to_map.getRotation()),
                                        tf2_ros::Point(odom_to_map.getOrigin()));

        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        rclcpp::Time transform_expiration = (laser_scan->header.stamp + transform_tolerance_);
        tf2_ros::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                                 transform_expiration,
                                                 global_frame_id_, odom_frame_id_);
        tfb_->sendTransform(tmp_tf_stamped);
    } else {
        // Nothing has change, therefore, republish the last transform.
        rclcpp::Time transform_expiration = (laser_scan->header.stamp + transform_tolerance_);
        tf2_ros::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), transform_expiration,
                                                 global_frame_id_, odom_frame_id_);
        tfb_->sendTransform(tmp_tf_stamped);
    } // end if (update)
}

bool lama::Slam2DROS::initLaser(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan) {
    // find the origin of the sensor in the base frame
    tf2_ros::Stamped <tf2_ros::Pose> identity(
            tf2_ros::Transform(tf2_ros::createIdentityQuaternion(), tf2_ros::Vector3(0, 0, 0)),
            rclcpp::Time(), laser_scan->header.frame_id);
    tf2_ros::Stamped <tf2_ros::Pose> laser_origin;
    try { tf_->transformPose(base_frame_id_, identity, laser_origin); }
    catch (tf2_ros::TransformException &e) {
        RCLCPP_ERROR(nh->get_logger(), "Could not find origin of %s", laser_scan->header.frame_id.c_str());
        return false;
    }

    // Validate laser orientation (code taken from slam_gmapping)
    // create a point 1m above the laser position and transform it into the laser-frame
    tf2_ros::Vector3 v;
    v.setValue(0, 0, 1 + laser_origin.getOrigin().z());
    tf2_ros::Stamped <tf2_ros::Vector3> up(v, laser_scan->header.stamp, base_frame_id_);
    try {
        tf_->transformPoint(laser_scan->header.frame_id, up, up);
        RCLCPP_DEBUG(nh->get_logger(), "Z-Axis in sensor frame: %.3f", up.z());
    } catch (tf2_ros::TransformException &e) {
        RCLCPP_ERROR(nh->get_logger(), "Unable to determine orientation of laser: %s", e.what());
        return false;
    }

    // we do not take roll or pitch into account. So check for correct sensor alignment.
    if (std::fabs(std::fabs(up.z()) - 1) > 0.001) {
        RCLCPP_WARN(nh->get_logger(), "Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
                    up.z());
        return false;
    }

    if (up.z() > 0) {
        laser_is_reversed_.push_back(laser_scan->angle_min > laser_scan->angle_max);

        Pose3D lp(laser_origin.getOrigin().x(), laser_origin.getOrigin().y(), 0,
                  0, 0, tf2_ros::getYaw(laser_origin.getRotation()));

        lasers_origin_.push_back(lp);
        RCLCPP_INFO(nh->get_logger(), "Laser is mounted upwards.");
    } else {
        laser_is_reversed_.push_back(laser_scan->angle_min < laser_scan->angle_max);

        Pose3D lp(laser_origin.getOrigin().x(), laser_origin.getOrigin().y(), 0,
                  M_PI, 0, tf2_ros::getYaw(laser_origin.getRotation()));

        lasers_origin_.push_back(lp);
        RCLCPP_INFO(nh->get_logger(), "Laser is mounted upside down.");
    }

    int laser_index = (int) frame_to_laser_.size();  // simple ID generator :)
    frame_to_laser_[laser_scan->header.frame_id] = laser_index;

    RCLCPP_INFO(nh->get_logger(), "New laser configured (id=%d frame_id=%s)", laser_index,
                laser_scan->header.frame_id.c_str());
    return true;
}

bool lama::Slam2DROS::OccupancyMsgFromOccupancyMap(nav_msgs::msg::OccupancyGrid &msg) {
    const FrequencyOccupancyMap *map = slam2d_->getOccupancyMap();
    if (map == 0)
        return false;

    Vector3ui imin, imax;
    map->bounds(imin, imax);

    unsigned int width = imax(0) - imin(0);
    unsigned int height = imax(1) - imin(1);

    if (width == 0 || height == 0)
        return false;

    if (width * height > msg.data.size())
        msg.data.resize(width * height);

    Image image;
    image.alloc(width, height, 1);
    image.fill(50);

    map->visit_all_cells([&image, &map, &imin](const Vector3ui &coords) {
        Vector3ui adj_coords = coords - imin;

        if (map->isFree(coords))
            image(adj_coords(0), adj_coords(1)) = 0;
        else if (map->isOccupied(coords))
            image(adj_coords(0), adj_coords(1)) = 100;
        else
            image(adj_coords(0), adj_coords(1)) = 0xff;
    });

    memcpy(&msg.data[0], image.data.get(), width * height);

    msg.info.width = width;
    msg.info.height = height;
    msg.info.resolution = map->resolution;

    Vector3d pos = map->m2w(imin);
    msg.info.origin.position.x = pos.x();
    msg.info.origin.position.y = pos.y();
    msg.info.origin.position.z = 0;
    msg.info.origin.orientation = tf2_ros::createQuaternionMsgFromYaw(0);

    return true;
}

bool lama::Slam2DROS::DistanceMsgFromOccupancyMap(nav_msgs::msg::OccupancyGrid &msg) {
    const DynamicDistanceMap *map = slam2d_->getDistanceMap();
    if (map == 0)
        return false;

    Vector3ui imin, imax;
    map->bounds(imin, imax);

    unsigned int width = imax(0) - imin(0);
    unsigned int height = imax(1) - imin(1);

    double factor = 1.0 / map->maxDistance();

    if (width == 0 || height == 0)
        return false;

    if (width * height > msg.data.size())
        msg.data.resize(width * height);

    Image image;
    image.alloc(width, height, 1);
    image.fill(50);

    map->visit_all_cells([&image, &map, &imin, factor](const Vector3ui &coords) {
        Vector3ui adj_coords = coords - imin;
        image(adj_coords(0), adj_coords(1)) = 100 - 100 * map->distance(coords) * factor;
    });

    memcpy(&msg.data[0], image.data.get(), width * height);

    msg.info.width = width;
    msg.info.height = height;
    msg.info.resolution = map->resolution;

    Vector3d pos = map->m2w(imin);
    msg.info.origin.position.x = pos.x();
    msg.info.origin.position.y = pos.y();
    msg.info.origin.position.z = 0;
    msg.info.origin.orientation = tf2_ros::createQuaternionMsgFromYaw(0);

    return true;
}

void lama::Slam2DROS::publishCallback() {  // const rclcpp::TimerEvent &
    auto time = ros_clock.now(); //rclcpp::Time::now();
    if (map_pub_->get_subscription_count() > 0) {
        OccupancyMsgFromOccupancyMap(ros_occ_);
        ros_occ_.header.stamp = time;
        map_pub_->publish(ros_occ_);
    }

    if (dist_pub_->get_subscription_count() > 0) {
        DistanceMsgFromOccupancyMap(ros_cost_);
        ros_cost_.header.stamp = time;
        dist_pub_->publish(ros_cost_);
    }
}

bool lama::Slam2DROS::onGetMap(nav_msgs::srv::GetMap::Request &req, nav_msgs::srv::GetMap::Response &res) {
    res.map.header.frame_id = global_frame_id_;
    res.map.header.stamp = ros_clock.now(); //rclcpp::Time::now();

    OccupancyMsgFromOccupancyMap(res.map);

    return true;
}

void lama::Slam2DROS::publishMaps() {
    auto time = ros_clock.now(); //rclcpp::Time::now();

    OccupancyMsgFromOccupancyMap(ros_occ_);
    ros_occ_.header.stamp = time;
    map_pub_->publish(ros_occ_);

    DistanceMsgFromOccupancyMap(ros_cost_);
    ros_cost_.header.stamp = time;
    dist_pub_->publish(ros_cost_);
}

void lama::Slam2DROS::printSummary() {
    if (slam2d_->summary)
        std::cout << slam2d_->summary->report() << std::endl;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    lama::Slam2DROS slam2d_ros{"slam2d_ros"};

    rclcpp::Node pnh("~");
    std::string rosbag_filename;

    if (!pnh.get_parameter("rosbag", rosbag_filename) || rosbag_filename.empty()) {
        RCLCPP_INFO(pnh.get_logger(), "Running SLAM in Live Mode");
    } else {
        RCLCPP_INFO(pnh.get_logger(), "Running SLAM in Rosbag Mode (offline)");
        lama::ReplayRosbag(pnh, rosbag_filename);

        if (rclcpp::ok())
            slam2d_ros.printSummary();

        RCLCPP_INFO(pnh.get_logger(), "You can now save your map. Use ctrl-c to quit.");
        // publish the maps a last time
        slam2d_ros.publishMaps();
    }

    rclcpp::spin(slam2d_ros.nh);
    return 0;
}

