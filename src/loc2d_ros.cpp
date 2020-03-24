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

//#include <nav_msgs/GetMap.h>
#include "nav_msgs/srv/get_map.hpp"

#include "lama/ros/loc2d_ros.h"
#include "lama/time.h"

#include <tf2/convert.h>

lama::Loc2DROS::Loc2DROS(const std::string &name) :
        transform_tolerance_(0, 100000000) {
    node = rclcpp::Node::make_shared(name);

    // Load parameters from the server.
    double tmp;
    node->get_parameter_or("global_frame_id", global_frame_id_, std::string("/map"));
    node->get_parameter_or("odom_frame_id", odom_frame_id_, std::string("/odom"));
    node->get_parameter_or("base_frame_id", base_frame_id_, std::string("/base_link"));
    node->get_parameter_or("scan_topic", scan_topic_, std::string("/scan"));
    node->get_parameter_or("transform_tolerance", tmp, 0.1);
    transform_tolerance_ = rclcpp::Duration::from_seconds(tmp);

    // Setup TF workers ...
    // https://github.com/ros-planning/navigation2/blob/eloquent-devel/nav2_costmap_2d/src/costmap_2d_ros.cpp
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            node->get_node_base_interface(),
            node->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    // Setup subscribers
    // Synchronized LaserScan messages with odometry transforms. This ensures that an odometry transformation exists
    //      when the handler of a LaserScan message is called.
    // https://github.com/ros2/message_filters/blob/master/include/message_filters/subscriber.h
    // https://github.com/ros2/ros1_bridge/pull/189/files
    // https://github.com/ros2/geometry2/blob/eloquent/tf2_ros/test/message_filter_test.cpp
    laser_scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
            node, scan_topic_, rclcpp::QoS(rclcpp::KeepLast(100)).get_rmw_qos_profile());
    laser_scan_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
            *laser_scan_sub_, *tf_buffer_, odom_frame_id_, 100,
            node->get_node_logging_interface(), node->get_node_clock_interface());
    laser_scan_filter_->registerCallback(std::bind(&Loc2DROS::onLaserScan, this, std::placeholders::_1));

    // https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/
    pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 1, std::bind(&Loc2DROS::onInitialPose, this, std::placeholders::_1));

    // Set publishers
    pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose", 10);

    RCLCPP_INFO(node->get_logger(), "Requesting the map...");
    auto req = std::make_shared<nav_msgs::srv::GetMap::Request>();
    auto client = node->create_client<nav_msgs::srv::GetMap>("static_map");
    auto result_future = client->async_send_request(req);
    while (rclcpp::ok() and rclcpp::spin_until_future_complete(node, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        // http://docs.ros2.org/latest/api/rclcpp/logging_8hpp.html#a451bee77c253ec72f4984bb577ff818a
        rclcpp::Clock myClock = *node->get_clock();  // Why cant this be directly used as an argument?...
        RCLCPP_WARN_THROTTLE(node->get_logger(), myClock, 1, "Request for map failed; trying again ...");
        rclcpp::Rate r(0.5);
        r.sleep();
    }

    auto result = result_future.get();
    InitLoc2DFromOccupancyGridMsg(result->map);

    RCLCPP_INFO(node->get_logger(), "2D Localization node up and running");
}

lama::Loc2DROS::~Loc2DROS() {

}

void lama::Loc2DROS::onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial_pose) {
    float x = initial_pose->pose.pose.position.x;
    float y = initial_pose->pose.pose.position.y;

    // https://github.com/ros2/geometry2/blob/ros2/tf2_geometry_msgs/test/test_tf2_geometry_msgs.cpp
    // https://answers.ros.org/question/339528/quaternion-to-rpy-ros2/
    tf2::Quaternion q;
    tf2::convert(initial_pose->pose.pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, initial_pose_yaw;
    m.getRPY(roll, pitch, initial_pose_yaw);

    //float yaw = tf2_ros::getYaw(initial_pose->pose.pose.orientation);

    RCLCPP_INFO(node->get_logger(), "Setting pose to (%f, %f, %f)", x, y, initial_pose_yaw);
    lama::Pose2D pose(x, y, initial_pose_yaw);

    loc2d_.setPose(pose);
}

void lama::Loc2DROS::onLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan) {
    /*int laser_index = -1;

    // verify if it is from a known source
    if (frame_to_laser_.find(laser_scan->header.frame_id) == frame_to_laser_.end()) {
        if (not initLaser(laser_scan))
            return;

        laser_index = frame_to_laser_[laser_scan->header.frame_id];
    } else {
        laser_index = frame_to_laser_[laser_scan->header.frame_id];
    }

    //tf::createIdentityQuaternion()

    // Where was the robot at the time of the scan ?
    tf2::Stamped <tf2::Transform> identity(
            tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0)),
            laser_scan->header.stamp, base_frame_id_);
    tf2::Stamped <tf2::Transform> odom_tf;
    try {
        buffer_.transform(identity, odom_tf, odom_frame_id_);
        //tf_->transformPose(odom_frame_id_, identity, odom_tf);
    } catch (tf2::TransformException &e) {
        RCLCPP_WARN(nh->get_logger(), "Failed to compute odom pose, skipping scan %s", e.what());
        return;
    }

    lama::Pose2D odom(odom_tf.getOrigin().x(), odom_tf.getOrigin().y(),
                      tf2_ros::getYaw(odom_tf.getRotation()));

    bool update = loc2d_.enoughMotion(odom);

    if (update) {

        size_t size = laser_scan->ranges.size();
        size_t beam_step = 1;

        float max_range = laser_scan->range_max;
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

        loc2d_.update(cloud, odom, laser_scan->header.stamp.toSec());

        Pose2D pose = loc2d_.getPose();
        // subtracting base to odom from map to base and send map to odom instead
        tf2::Stamped <tf2::Transform> odom_to_map;
        try {
            tf2::Transform tmp_tf(tf2_ros::createQuaternionFromYaw(pose.rotation()),
                                      tf2::Vector3(pose.x(), pose.y(), 0));
            tf2::Stamped <tf2::Transform> tmp_tf_stamped(tmp_tf.inverse(), laser_scan->header.stamp, base_frame_id_);

            buffer_.transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
            //tf_->transformPose(odom_frame_id_, tmp_tf_stamped, odom_to_map);


        } catch (tf2::TransformException) {
            RCLCPP_WARN(nh->get_logger(), "Failed to subtract base to odom transform");
            return;
        }

        latest_tf_ = tf2::Transform(tf2_ros::Quaternion(odom_to_map.getRotation()),
                                        tf2_ros::Point(odom_to_map.getOrigin()));

        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        rclcpp::Time transform_expiration = (laser_scan->header.stamp + transform_tolerance_);
        tf2::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                             transform_expiration,
                                             global_frame_id_, odom_frame_id_);
        tfb_->sendTransform(tmp_tf_stamped);
    } else {
        // Nothing has change, therefore, republish the last transform.
        rclcpp::Time transform_expiration = (laser_scan->header.stamp + transform_tolerance_);
        tf2::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), transform_expiration,
                                             global_frame_id_, odom_frame_id_);
        tfb_->sendTransform(tmp_tf_stamped);
    } // end if (update)
     */
}

void lama::Loc2DROS::InitLoc2DFromOccupancyGridMsg(const nav_msgs::msg::OccupancyGrid &msg) {
    /*Vector2d pos;
    double tmp;
    pnh_.get_parameter_or("initial_pos_x", pos[0], 0.0);
    pnh_.get_parameter_or("initial_pos_y", pos[1], 0.0);
    pnh_.get_parameter_or("initial_pos_a", tmp, 0.0);
    lama::Pose2D prior(pos, tmp);

    Loc2D::Options options;
    pnh_.get_parameter_or("d_thresh", options.trans_thresh, 0.01);
    pnh_.get_parameter_or("a_thresh", options.rot_thresh, 0.2);
    pnh_.get_parameter_or("l2_max", options.l2_max, 0.5);
    pnh_.get_parameter_or("strategy", options.strategy, std::string("gn"));

    int itmp;
    pnh_.get_parameter_or("patch_size", itmp, 32);
    options.patch_size = itmp;

    options.resolution = msg.info.resolution;

    loc2d_.Init(options);
    loc2d_.setPose(prior);

    RCLCPP_INFO(nh->get_logger(), "Localization parameters: d_thresh: %.2f, a_thresh: %.2f, l2_max: %.2f",
                options.trans_thresh, options.rot_thresh, options.l2_max);

    unsigned int width = msg.info.width;
    unsigned int height = msg.info.height;

    for (unsigned int j = 0; j < height; ++j)
        for (unsigned int i = 0; i < width; ++i) {

            Vector3d coords;
            coords.x() = msg.info.origin.position.x + i * msg.info.resolution;
            coords.y() = msg.info.origin.position.y + j * msg.info.resolution;

            char value = msg.data[i + j * width];
            if (value == 0) {
                loc2d_.occupancy_map->setFree(coords);
            } else if (value == 100) {
                loc2d_.occupancy_map->setOccupied(coords);
                loc2d_.distance_map->addObstacle(loc2d_.distance_map->w2m(coords));
            }
        }// end for

    loc2d_.distance_map->update();
     */
}

bool lama::Loc2DROS::initLaser(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan) {
    /*
    //tf::createIdentityQuaternion()

    // find the origin of the sensor in the base frame
    tf2::Stamped <tf2::Transform> identity(
            tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0)),
            rclcpp::Time(), laser_scan->header.frame_id);
    tf2::Stamped <tf2::Transform> laser_origin;
    try {
        buffer_.transform(identity, laser_origin, base_frame_id_);
        //tf_->transformPose(base_frame_id_, identity, laser_origin);
    } catch (tf2::TransformException &e) {
        RCLCPP_ERROR(nh->get_logger(), "Could not find origin of %s", laser_scan->header.frame_id.c_str());
        return false;
    }

    // Validate laser orientation (code taken from slam_gmapping)
    // create a point 1m above the laser position and transform it into the laser-frame
    tf2::Vector3 v;
    v.setValue(0, 0, 1 + laser_origin.getOrigin().z());

    //tf::Stamped<tf::Vector3> up(v, laser_scan->header.stamp, base_frame_id_);
    //tf_->transformPoint(laser_scan->header.frame_id, up, up);

    // laser_scan->header.stamp
    tf2::Stamped<tf2::Vector3> up(v, laser_scan->header.stamp, base_frame_id_);
    try {
        buffer_.transform(up, up, laser_scan->header.frame_id);

        RCLCPP_DEBUG(nh->get_logger(), "Z-Axis in sensor frame: %.3f", up.z());
    } catch (tf2::TransformException &e) {
        RCLCPP_ERROR(nh->get_logger(), "Unable to determine orientation of laser: %s", e.what());
        return false;
    }

    // we do not take roll or pitch into account. So check for correct sensor alignment.
    if (std::fabs(std::fabs(up.z()) - 1) > 0.001) {
        RCLCPP_WARN(nh->get_logger(), "Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
                    up.z());
        return false;
    }

    tf2::Matrix3x3 matrix3x3(laser_origin.getRotation());
    tf2Scalar useless_pitch, useless_roll, laser_origin_yaw;
    matrix3x3.getRPY(useless_pitch, useless_roll, laser_origin_yaw);

    if (up.z() > 0) {
        laser_is_reversed_.push_back(laser_scan->angle_min > laser_scan->angle_max);

        lama::Pose3D lp(laser_origin.getOrigin().x(), laser_origin.getOrigin().y(), 0,
                        0, 0, laser_origin_yaw);

        lasers_origin_.push_back(lp);
        RCLCPP_INFO(nh->get_logger(), "Laser is mounted upwards.");
    } else {
        laser_is_reversed_.push_back(laser_scan->angle_min < laser_scan->angle_max);

        lama::Pose3D lp(laser_origin.getOrigin().x(), laser_origin.getOrigin().y(), 0,
                        M_PI, 0, laser_origin_yaw);

        lasers_origin_.push_back(lp);
        RCLCPP_INFO(nh->get_logger(), "Laser is mounted upside down.");
    }

    int laser_index = (int) frame_to_laser_.size();  // simple ID generator :)
    frame_to_laser_[laser_scan->header.frame_id] = laser_index;

    RCLCPP_INFO(nh->get_logger(), "New laser configured (id=%d frame_id=%s)", laser_index,
                laser_scan->header.frame_id.c_str());
                */
    return true;

}

int main(int argc, char *argv[]) {
    //rclcpp::init(argc, argv);   // "loc2d_ros"
    //lama::Loc2DROS loc2d_ros{"loc2d_ros"};
    //rclcpp::spin(loc2d_ros.nh);

    return 0;
}
