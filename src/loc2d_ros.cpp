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

#include "lama/ros/loc2d_ros.h"

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
    laser_scan_sub_ = std::make_shared < message_filters::Subscriber < sensor_msgs::msg::LaserScan >> (
            node, scan_topic_, rclcpp::QoS(rclcpp::KeepLast(100)).get_rmw_qos_profile());
    laser_scan_filter_ = std::make_shared < tf2_ros::MessageFilter < sensor_msgs::msg::LaserScan >> (
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
    rclcpp::Rate r(1);
    while (rclcpp::ok() and
           rclcpp::spin_until_future_complete(node, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS) {
        // http://docs.ros2.org/latest/api/rclcpp/logging_8hpp.html#a451bee77c253ec72f4984bb577ff818a
        rclcpp::Clock myClock = *node->get_clock();  // Why cant this be directly used as an argument?...
        RCLCPP_WARN_THROTTLE(node->get_logger(), myClock, 1, "Request for map failed; trying again ...");
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

    double initial_pose_yaw = lama_utils::getYaw(initial_pose->pose.pose.orientation);

    RCLCPP_INFO(node->get_logger(), "Setting pose to (%f, %f, %f)", x, y, initial_pose_yaw);
    lama::Pose2D pose(x, y, initial_pose_yaw);

    loc2d_.setPose(pose);
}

void lama::Loc2DROS::onLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan) {
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
    // http://wiki.ros.org/tf2/Terminology
    // tf::Pose equivalent to http://docs.ros.org/groovy/api/tf2/html/c++/classbtTransform.html
    // Is btTransform equivalent to http://docs.ros.org/jade/api/tf2/html/classtf2_1_1Transform.html ?
    // https://github.com/ros2/geometry2/blob/ros2/tf2/include/tf2/transform_datatypes.h
    // https://answers.ros.org/question/309953/lookuptransform-builtin_interfacestime-to-tf2timepoint/
    // http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
    geometry_msgs::msg::PoseStamped msg_odom_tf;
    try {
        geometry_msgs::msg::PoseStamped msg_odom_tf_baseFrame = lama_utils::createPoseStamped(
                tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0)),
                rclcpp::Time(laser_scan->header.stamp), base_frame_id_);
        tf_buffer_->transform(msg_odom_tf_baseFrame, msg_odom_tf, odom_frame_id_);
    } catch (tf2::TransformException &e) {
        RCLCPP_WARN(node->get_logger(), "Failed to compute odom pose, skipping scan %s", e.what());
        return;
    }
    tf2::Stamped <tf2::Transform> odom_tf = lama_utils::createStampedTransform(msg_odom_tf);

    double odom_tf_yaw = lama_utils::getYaw(odom_tf.getRotation());
    lama::Pose2D odom(odom_tf.getOrigin().x(), odom_tf.getOrigin().y(), odom_tf_yaw);

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

        // https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/time.hpp
        // https://github.com/ros2/rcl_interfaces/blob/master/builtin_interfaces/msg/Time.msg
        rclcpp::Time t(laser_scan->header.stamp);
        loc2d_.update(cloud, odom, t.seconds());

        Pose2D pose = loc2d_.getPose();

        // subtracting base to odom from map to base and send map to odom instead
        geometry_msgs::msg::PoseStamped msg_odom_to_map;
        try {
            // http://docs.ros.org/diamondback/api/tf/html/c++/transform__datatypes_8h_source.html
            // http://docs.ros.org/jade/api/tf2/html/classtf2_1_1Quaternion.html
            tf2::Quaternion q;
            q.setRPY(0, 0, pose.rotation());
            geometry_msgs::msg::PoseStamped msg_odom_to_map_baseFrame = lama_utils::createPoseStamped(
                    tf2::Transform(q, tf2::Vector3(pose.x(), pose.y(), 0)),
                    rclcpp::Time(laser_scan->header.stamp), base_frame_id_);
            tf_buffer_->transform(msg_odom_to_map_baseFrame, msg_odom_to_map, odom_frame_id_);
        } catch (tf2::TransformException &e) {
            RCLCPP_WARN(node->get_logger(), "Failed to subtract base to odom transform");
            return;
        }
        tf2::Stamped <tf2::Transform> odom_to_map = lama_utils::createStampedTransform(msg_odom_to_map);

        latest_tf_ = tf2::Transform(tf2::Quaternion(odom_to_map.getRotation()),
                                    tf2::Vector3(odom_to_map.getOrigin()));

        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        // https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/time.hpp
        rclcpp::Time transform_expiration = rclcpp::Time(laser_scan->header.stamp) + transform_tolerance_;

        // http://wiki.ros.org/tf2/Tutorials/Migration/DataConversions
        // http://docs.ros.org/indigo/api/tf2_ros/html/c++/classtf2__ros_1_1TransformBroadcaster.html
        // http://docs.ros.org/indigo/api/tf/html/c++/classtf_1_1StampedTransform.html
        // https://answers.ros.org/question/347582/how-to-efficiently-get-transformstamped-in-eloquent/
        geometry_msgs::msg::TransformStamped tmp_tf_stamped = lama_utils::createTransformStamped(
                latest_tf_.inverse(), transform_expiration, global_frame_id_, odom_frame_id_);
        tfb_->sendTransform(tmp_tf_stamped);

    } else {
        // Nothing has change, therefore, republish the last transform.
        rclcpp::Time transform_expiration = rclcpp::Time(laser_scan->header.stamp) + transform_tolerance_;
        geometry_msgs::msg::TransformStamped tmp_tf_stamped = lama_utils::createTransformStamped(
                latest_tf_.inverse(), transform_expiration, global_frame_id_, odom_frame_id_);
        tfb_->sendTransform(tmp_tf_stamped);
    } // end if (update)

}

void lama::Loc2DROS::InitLoc2DFromOccupancyGridMsg(const nav_msgs::msg::OccupancyGrid &msg) {
    Vector2d pos;
    double tmp;
    node->get_parameter_or("initial_pos_x", pos[0], 0.0);
    node->get_parameter_or("initial_pos_y", pos[1], 0.0);
    node->get_parameter_or("initial_pos_a", tmp, 0.0);
    lama::Pose2D prior(pos, tmp);

    Loc2D::Options options;
    node->get_parameter_or("d_thresh", options.trans_thresh, 0.01);
    node->get_parameter_or("a_thresh", options.rot_thresh, 0.2);
    node->get_parameter_or("l2_max", options.l2_max, 0.5);
    node->get_parameter_or("strategy", options.strategy, std::string("gn"));

    int itmp;
    node->get_parameter_or("patch_size", itmp, 32);
    options.patch_size = itmp;

    options.resolution = msg.info.resolution;

    loc2d_.Init(options);
    loc2d_.setPose(prior);

    RCLCPP_INFO(node->get_logger(), "Localization parameters: d_thresh: %.2f, a_thresh: %.2f, l2_max: %.2f",
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
}

bool lama::Loc2DROS::initLaser(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan) {
    // find the origin of the sensor in the base frame
    geometry_msgs::msg::PoseStamped msg_laser_origin;
    try {
        geometry_msgs::msg::PoseStamped msg_laser_origin_baseFrame = lama_utils::createPoseStamped(
                tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0)),
                rclcpp::Time(), laser_scan->header.frame_id);
        tf_buffer_->transform(msg_laser_origin_baseFrame, msg_laser_origin, odom_frame_id_);
    } catch (tf2::TransformException &e) {
        RCLCPP_ERROR(node->get_logger(), "Could not find origin of %s", laser_scan->header.frame_id.c_str());
        return false;
    }
    tf2::Stamped <tf2::Transform> laser_origin = lama_utils::createStampedTransform(msg_laser_origin);

    // Validate laser orientation (code taken from slam_gmapping)
    // create a point 1m above the laser position and transform it into the laser-frame
    tf2::Vector3 v;
    v.setValue(0, 0, 1 + laser_origin.getOrigin().z());

    geometry_msgs::msg::Vector3Stamped msg_up;
    try {
        geometry_msgs::msg::Vector3Stamped msg_up_baseFrame = lama_utils::createVector3Stamped(
                v, rclcpp::Time(laser_scan->header.stamp), base_frame_id_);
        tf_buffer_->transform(msg_up_baseFrame, msg_up, laser_scan->header.frame_id);
        RCLCPP_DEBUG(node->get_logger(), "Z-Axis in sensor frame: %.3f", msg_up.vector.z);
    } catch (tf2::TransformException &e) {
        RCLCPP_ERROR(node->get_logger(), "Unable to determine orientation of laser: %s", e.what());
        return false;
    }
    tf2::Stamped <tf2::Vector3> up = lama_utils::createStampedVector3(msg_up);

    // we do not take roll or pitch into account. So check for correct sensor alignment.
    if (std::fabs(std::fabs(up.z()) - 1) > 0.001) {
        RCLCPP_WARN(node->get_logger(),
                    "Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f", up.z());
        return false;
    }

    double laser_origin_yaw = lama_utils::getYaw(laser_origin.getRotation());

    if (up.z() > 0) {
        laser_is_reversed_.push_back(laser_scan->angle_min > laser_scan->angle_max);

        lama::Pose3D lp(laser_origin.getOrigin().x(), laser_origin.getOrigin().y(), 0,
                        0, 0, laser_origin_yaw);

        lasers_origin_.push_back(lp);
        RCLCPP_INFO(node->get_logger(), "Laser is mounted upwards.");
    } else {
        laser_is_reversed_.push_back(laser_scan->angle_min < laser_scan->angle_max);

        lama::Pose3D lp(laser_origin.getOrigin().x(), laser_origin.getOrigin().y(), 0,
                        M_PI, 0, laser_origin_yaw);

        lasers_origin_.push_back(lp);
        RCLCPP_INFO(node->get_logger(), "Laser is mounted upside down.");
    }

    int laser_index = (int) frame_to_laser_.size();  // simple ID generator :)
    frame_to_laser_[laser_scan->header.frame_id] = laser_index;

    RCLCPP_INFO(node->get_logger(), "New laser configured (id=%d frame_id=%s)", laser_index,
                laser_scan->header.frame_id.c_str());

    return true;

}

int main(int argc, char *argv[]) {
    // https://github.com/ros2/examples/blob/master/rclcpp/minimal_publisher/not_composable.cpp

    rclcpp::init(argc, argv);
    lama::Loc2DROS loc2d_ros{"loc2d_ros"};
    rclcpp::spin(loc2d_ros.node);
    return 0;
}
