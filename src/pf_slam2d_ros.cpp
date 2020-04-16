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

#include "lama/ros/pf_slam2d_ros.h"

lama::PFSlam2DROS::PFSlam2DROS(std::string name) :
        transform_tolerance_(0, 100000000) {
    node = rclcpp::Node::make_shared(name);
    ros_clock = node->get_clock();

    // Load parameters from the server.
    double tmp;
    node->declare_parameter("global_frame_id");
    node->get_parameter_or("global_frame_id", global_frame_id_, std::string("map"));
    node->declare_parameter("odom_frame_id");
    node->get_parameter_or("odom_frame_id", odom_frame_id_, std::string("odom"));
    node->declare_parameter("base_frame_id");
    node->get_parameter_or("base_frame_id", base_frame_id_, std::string("base_link"));
    node->declare_parameter("scan_topic");
    node->get_parameter_or("scan_topic", scan_topic_, std::string("/scan"));
    node->declare_parameter("transform_tolerance");
    node->get_parameter_or("transform_tolerance", tmp, 0.1);
    transform_tolerance_ = rclcpp::Duration(static_cast<int64_t>(RCL_S_TO_NS(tmp)));

    Vector2d pos;
    node->declare_parameter("initial_pos_x");
    node->get_parameter_or("initial_pos_x", pos[0], 0.0);
    node->declare_parameter("initial_pos_y");
    node->get_parameter_or("initial_pos_y", pos[1], 0.0);
    node->declare_parameter("initial_pos_a");
    node->get_parameter_or("initial_pos_a", tmp, 0.0);
    Pose2D prior(pos, tmp);

    PFSlam2D::Options options;
    node->declare_parameter("srr");
    node->get_parameter_or("srr", options.srr, 0.1);
    node->declare_parameter("str");
    node->get_parameter_or("str", options.str, 0.2);
    node->declare_parameter("stt");
    node->get_parameter_or("stt", options.stt, 0.1);
    node->declare_parameter("srt");
    node->get_parameter_or("srt", options.srt, 0.2);
    node->declare_parameter("sigma");
    node->get_parameter_or("sigma", options.meas_sigma, 0.05);
    node->declare_parameter("lgain");
    node->get_parameter_or("lgain", options.meas_sigma_gain, 3.0);
    node->declare_parameter("d_thresh");
    node->get_parameter_or("d_thresh", options.trans_thresh, 0.5);
    node->declare_parameter("a_thresh");
    node->get_parameter_or("a_thresh", options.rot_thresh, 0.25);
    node->declare_parameter("l2_max");
    node->get_parameter_or("l2_max", options.l2_max, 0.5);
    node->declare_parameter("truncate");
    node->get_parameter_or("truncate", options.truncated_ray, 0.0);
    node->declare_parameter("resolution");
    node->get_parameter_or("resolution", options.resolution, 0.05);
    node->declare_parameter("strategy");
    node->get_parameter_or("strategy", options.strategy, std::string("gn"));
    node->declare_parameter("use_compression");
    node->get_parameter_or("use_compression", options.use_compression, false);
    node->declare_parameter("compression_algorithm");
    node->get_parameter_or("compression_algorithm", options.calgorithm, std::string("zstd"));
    node->declare_parameter("mrange");
    node->get_parameter_or("mrange", max_range_, 16.0);
    node->declare_parameter("threads");
    node->get_parameter_or("threads", options.threads, -1);

    int itmp;
    node->declare_parameter("patch_size");
    node->get_parameter_or("patch_size", itmp, 32);
    options.patch_size = itmp;
    node->declare_parameter("particles");
    node->get_parameter_or("particles", itmp, 30);
    options.particles = itmp;
    node->declare_parameter("cache_size");
    node->get_parameter_or("cache_size", itmp, 100);
    options.cache_size = itmp;
    // ros param does not have unsigned int??
    node->declare_parameter("seed");
    node->get_parameter_or("seed", tmp, 0.0);
    options.seed = tmp;

    node->declare_parameter("create_summary");
    node->get_parameter_or("create_summary", options.create_summary, false);

    node->declare_parameter("map_publish_period");
    node->get_parameter_or("map_publish_period", tmp, 5.0);
    periodic_publish_ = node->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::milliseconds(static_cast<int>(tmp * 1000))),
            std::bind(&PFSlam2DROS::publishCallback, this));

    slam2d_ = std::make_shared<PFSlam2D>(options);
    slam2d_->setPrior(prior);

    // Setup TF workers ...
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    // Syncronized LaserScan messages with odometry transforms. This ensures that an odometry transformation
    // exists when the handler of a LaserScan message is called.
    laser_scan_sub_ = std::make_shared < message_filters::Subscriber < sensor_msgs::msg::LaserScan >> (
            node, scan_topic_, rclcpp::QoS(rclcpp::SystemDefaultsQoS()).keep_last(100).get_rmw_qos_profile());
    laser_scan_filter_ = std::make_shared < tf2_ros::MessageFilter < sensor_msgs::msg::LaserScan >> (
            *laser_scan_sub_, *tf_buffer_, odom_frame_id_, 100,
                    node->get_node_logging_interface(), node->get_node_clock_interface());
    laser_scan_filter_->registerCallback(std::bind(&PFSlam2DROS::onLaserScan, this, std::placeholders::_1));

    // Setup publishers
    // TODO latch https://github.com/ros2/ros2/issues/464
    //      https://answers.ros.org/question/305795/ros2-latching/
    //      http://docs.ros.org/kinetic/api/roscpp/html/classros_1_1NodeHandle.html#a6b655c04f32c4c967d49799ff9312ac6
    pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose", 2);
    map_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1); // latch=true
    dist_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/distance", 1); // latch=true
    patch_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/patch", 1); // latch=true
    poses_pub_ = node->create_publisher<geometry_msgs::msg::PoseArray>("/poses", 1); // latch=true
    path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/path", 1); // latch=true

    ros_occ_.header.frame_id = global_frame_id_;
    ros_cost_.header.frame_id = global_frame_id_;
    ros_patch_.header.frame_id = global_frame_id_;
    poses_.header.frame_id = global_frame_id_;

    // Setup service
    // https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Service-And-Client/#write-the-service-node
    // https://answers.ros.org/question/299126/ros2-error-creating-a-service-server-as-a-member-function/
    service = node->create_service<nav_msgs::srv::GetMap>("/dynamic_map",
                                                          std::bind(&PFSlam2DROS::onGetMap, this, std::placeholders::_1,
                                                                    std::placeholders::_2));

    RCLCPP_INFO(node->get_logger(), "PF SLAM node up and running");
}

lama::PFSlam2DROS::~PFSlam2DROS() {

}

void lama::PFSlam2DROS::onLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan) {
    int laser_index = -1;

    // verify if it is from a known source
    if (frame_to_laser_.find(laser_scan->header.frame_id) == frame_to_laser_.end()) {

        laser_index = (int) frame_to_laser_.size();  // simple ID generator :)
        lasers_update_.push_back(false);            // do not update when a laser is added to the known list.
        frame_to_laser_[laser_scan->header.frame_id] = laser_index;

        // find the origin of the sensor
        geometry_msgs::msg::PoseStamped msg_laser_origin;
        try {
            geometry_msgs::msg::PoseStamped msg_identity = lama_utils::createPoseStamped(
                    tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0)),
                    rclcpp::Time(), laser_scan->header.frame_id);
            tf_buffer_->transform(msg_identity, msg_laser_origin, base_frame_id_);
        } catch (tf2::TransformException &e) {
            RCLCPP_ERROR(node->get_logger(), "Could not find origin of %s", laser_scan->header.frame_id.c_str());
            return;
        }
        tf2::Stamped <tf2::Transform> laser_origin = lama_utils::createStampedTransform(msg_laser_origin);

        double laser_origin_yaw = lama_utils::getYaw(laser_origin.getRotation());
        Pose3D lp(laser_origin.getOrigin().x(), laser_origin.getOrigin().y(), 0,
                  0, 0, laser_origin_yaw);

        lasers_origin_.push_back(lp);

        RCLCPP_INFO(node->get_logger(), "New laser configured (id=%d frame_id=%s)", laser_index,
                    laser_scan->header.frame_id.c_str());
    } else {
        laser_index = frame_to_laser_[laser_scan->header.frame_id];
    }

    // Where was the robot at the time of the scan ?
    geometry_msgs::msg::PoseStamped msg_odom_tf;
    try {
        geometry_msgs::msg::PoseStamped msg_identity = lama_utils::createPoseStamped(
                tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0)),
                rclcpp::Time(laser_scan->header.stamp), base_frame_id_);
        tf_buffer_->transform(msg_identity, msg_odom_tf, odom_frame_id_);
    } catch (tf2::TransformException &e) {
        RCLCPP_WARN(node->get_logger(), "Failed to compute odom pose, skipping scan %s", e.what());
        return;
    }
    tf2::Stamped <tf2::Transform> odom_tf = lama_utils::createStampedTransform(msg_odom_tf);

    Pose2D odom(odom_tf.getOrigin().x(), odom_tf.getOrigin().y(), lama_utils::getYaw(odom_tf.getRotation()));

    bool update;

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
    for(size_t i = 0; i < size; i += beam_step ){

        if (std::isnan(laser_scan->ranges[i]) || std::isinf(laser_scan->ranges[i]))
            continue;

        if ( laser_scan->ranges[i] >= max_range || laser_scan->ranges[i] <= min_range )
            continue;

        Eigen::Vector3d point;
        point << laser_scan->ranges[i] * std::cos(angle_min+(i*angle_inc)),
                 laser_scan->ranges[i] * std::sin(angle_min+(i*angle_inc)),
                 0;

        cloud->points.push_back( point );
    }

    rclcpp::Time start = ros_clock->now();
    update = slam2d_->update(cloud, odom, rclcpp::Time(laser_scan->header.stamp).seconds());
    rclcpp::Time end = ros_clock->now();

    if (update){
        Pose2D pose = slam2d_->getPose();

        // subtracting base to odom from map to base and send map to odom instead
        geometry_msgs::msg::PoseStamped msg_odom_to_map;
        try {
            tf2::Quaternion q;
            q.setRPY(0, 0, pose.rotation());
            geometry_msgs::msg::PoseStamped msg_odom_to_map_baseFrame = lama_utils::createPoseStamped(
                    tf2::Transform(q, tf2::Vector3(pose.x(), pose.y(), 0)).inverse(),
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
        rclcpp::Time transform_expiration = rclcpp::Time(laser_scan->header.stamp) + transform_tolerance_;
        geometry_msgs::msg::TransformStamped tmp_tf_stamped = lama_utils::createTransformStamped(
                latest_tf_.inverse(), transform_expiration, global_frame_id_, odom_frame_id_);
        tfb_->sendTransform(tmp_tf_stamped);

        //--
        nav_msgs::msg::Path path;
        path.header.frame_id = global_frame_id_;
        path.header.stamp    = laser_scan->header.stamp;

        uint32_t idx = slam2d_->getBestParticleIdx();
        const size_t num_poses = slam2d_->getParticles()[idx].poses.size();
        for (size_t i = 0; i < num_poses; ++i){

            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = global_frame_id_;
            p.header.stamp    = laser_scan->header.stamp;
            p.pose.position.x = slam2d_->getParticles()[idx].poses[i].x();
            p.pose.position.y = slam2d_->getParticles()[idx].poses[i].y();
            p.pose.position.z = 0.0;

            tf2::Quaternion quat_tf;
            quat_tf.setRPY(0.0, 0.0, slam2d_->getParticles()[idx].poses[i].rotation());
            p.pose.orientation.x = quat_tf.getX();
            p.pose.orientation.y = quat_tf.getY();
            p.pose.orientation.z = quat_tf.getZ();
            p.pose.orientation.w = quat_tf.getW();
            //tf2::convert(quat_tf, p.pose.orientation);
            //p.pose.orientation = tf2_ros::createQuaternionMsgFromYaw(slam2d_->getParticles()[idx].poses[i].rotation());

            path.poses.push_back(p);
        }
        path_pub_->publish(path);

        RCLCPP_DEBUG(node->get_logger(), "Update time: %.3fms - NEFF: %.2f",
                RCUTILS_NS_TO_MS((end-start).nanoseconds()), slam2d_->getNeff());
        RCLCPP_DEBUG(node->get_logger(), "Sent TF Map->Odom");
    } else {
        // Nothing has change, therefore, republish the last transform.
        rclcpp::Time transform_expiration = rclcpp::Time(laser_scan->header.stamp) + transform_tolerance_;
        geometry_msgs::msg::TransformStamped tmp_tf_stamped = lama_utils::createTransformStamped(
                latest_tf_.inverse(), transform_expiration, global_frame_id_, odom_frame_id_);
        tfb_->sendTransform(tmp_tf_stamped);
        RCLCPP_DEBUG(node->get_logger(), "Nothing sent as TF Map->Odom");
    } // end if (update)

    const size_t num_particles = slam2d_->getParticles().size();
    poses_.poses.resize(num_particles);
    for (size_t i = 0; i < num_particles; ++i){

        poses_.poses[i].position.x = slam2d_->getParticles()[i].pose.x();
        poses_.poses[i].position.y = slam2d_->getParticles()[i].pose.y();
        poses_.poses[i].position.z = 0.0;

        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0.0, 0.0, slam2d_->getParticles()[i].pose.rotation());
        poses_.poses[i].orientation.x = quat_tf.getX();
        poses_.poses[i].orientation.y = quat_tf.getY();
        poses_.poses[i].orientation.z = quat_tf.getZ();
        poses_.poses[i].orientation.w = quat_tf.getW();
        //tf2::convert(quat_tf, poses_.poses[i].orientation);
        //poses_.poses[i].orientation = tf2_ros::createQuaternionMsgFromYaw(slam2d_->getParticles()[i].pose.rotation());
    }

    poses_pub_->publish(poses_);
}

void lama::PFSlam2DROS::publishCallback()       // const rclcpp::TimerEvent &
{
    auto time = ros_clock->now(); //rclcpp::Time::now();

    nav_msgs::msg::OccupancyGrid ros_occ;
    ros_occ.header.frame_id = global_frame_id_;
    ros_occ.header.stamp = time;

    if (map_pub_->get_subscription_count() > 0 ){
        OccupancyMsgFromOccupancyMap(ros_occ);
        ros_occ_.header.stamp = time;
        map_pub_->publish(ros_occ);
    }

    if (dist_pub_->get_subscription_count() > 0){
        DistanceMsgFromOccupancyMap(ros_cost_);
        ros_cost_.header.stamp = time;
        dist_pub_->publish(ros_cost_);
    }

    if (patch_pub_->get_subscription_count() > 0){
        PatchMsgFromOccupancyMap(ros_patch_);
        ros_patch_.header.stamp = time;
        patch_pub_->publish(ros_patch_);
    }
}

void lama::PFSlam2DROS::publishMaps() {
    auto time = ros_clock->now(); //rclcpp::Time::now();

    nav_msgs::msg::OccupancyGrid ros_occ;
    ros_occ.header.frame_id = global_frame_id_;
    ros_occ.header.stamp = time;

    OccupancyMsgFromOccupancyMap(ros_occ);
    ros_occ_.header.stamp = time;
    map_pub_->publish(ros_occ);

    DistanceMsgFromOccupancyMap(ros_cost_);
    ros_cost_.header.stamp = time;
    dist_pub_->publish(ros_cost_);

    PatchMsgFromOccupancyMap(ros_patch_);
    ros_patch_.header.stamp = time;
    patch_pub_->publish(ros_patch_);
}

bool lama::PFSlam2DROS::OccupancyMsgFromOccupancyMap(nav_msgs::msg::OccupancyGrid &msg) {

    const FrequencyOccupancyMap* map = slam2d_->getOccupancyMap();
    if (map == 0)
        return false;

    Vector3ui imin, imax;
    map->bounds(imin, imax);

    unsigned int width = imax(0) - imin(0);
    unsigned int height= imax(1) - imin(1);

    if (width == 0 || height == 0)
        return false;

    if ( width*height > msg.data.size() )
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

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, 0);
    msg.info.origin.orientation.x = quat_tf.getX();
    msg.info.origin.orientation.y = quat_tf.getY();
    msg.info.origin.orientation.z = quat_tf.getZ();
    msg.info.origin.orientation.w = quat_tf.getW();
    //tf2::convert(quat_tf, msg.info.origin.orientation);
    //msg.info.origin.orientation = tf2_ros::createQuaternionMsgFromYaw(0);

    return true;
}

bool lama::PFSlam2DROS::DistanceMsgFromOccupancyMap(nav_msgs::msg::OccupancyGrid &msg) {
    const DynamicDistanceMap* map = slam2d_->getDistanceMap();
    if (map == 0)
        return false;

    Vector3ui imin, imax;
    map->bounds(imin, imax);

    unsigned int width = imax(0) - imin(0);
    unsigned int height= imax(1) - imin(1);

    double factor = 1.0 / map->maxDistance();

    if (width == 0 || height == 0)
        return false;

    if ( width*height > msg.data.size() )
        msg.data.resize(width*height);

    Image image;
    image.alloc(width, height, 1);
    image.fill(50);

    map->visit_all_cells([&image, &map, &imin, factor](const Vector3ui& coords){
        Vector3ui adj_coords = coords - imin;
        image(adj_coords(0), adj_coords(1)) = 100 - 100 * map->distance(coords) * factor;
    });

    memcpy(&msg.data[0], image.data.get(), width*height);

    msg.info.width = width;
    msg.info.height = height;
    msg.info.resolution = map->resolution;

    Vector3d pos = map->m2w(imin);
    msg.info.origin.position.x = pos.x();
    msg.info.origin.position.y = pos.y();
    msg.info.origin.position.z = 0;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, 0);
    msg.info.origin.orientation.x = quat_tf.getX();
    msg.info.origin.orientation.y = quat_tf.getY();
    msg.info.origin.orientation.z = quat_tf.getZ();
    msg.info.origin.orientation.w = quat_tf.getW();
    //tf2::convert(quat_tf, msg.info.origin.orientation);
    //msg.info.origin.orientation = tf2_ros::createQuaternionMsgFromYaw(0);

    return true;
}

bool lama::PFSlam2DROS::PatchMsgFromOccupancyMap(nav_msgs::msg::OccupancyGrid &msg) {
    const FrequencyOccupancyMap* map = slam2d_->getOccupancyMap();
    if (map == 0)
        return false;

    Vector3d min, max;
    map->bounds(min, max);

    Vector3ui imin = map->w2m(min);
    Vector3ui imax = map->w2m(max);

    unsigned int width = imax(0) - imin(0);
    unsigned int height= imax(1) - imin(1);

    if (width == 0 || height == 0)
        return false;

    msg.data.resize(width*height);

    for (unsigned int j = imin(1); j < imax(1); ++j)
        for (unsigned int i = imin(0); i < imax(0); ++i){
            if (map->patchAllocated(Vector3ui(i,j,0))){
                if (map->patchIsUnique(Vector3ui(i,j,0)))
                    msg.data[ (j-imin[1])*width + (i-imin[0]) ] = 100;
                else
                    msg.data[ (j-imin[1])*width + (i-imin[0]) ] = 50;
            }
            else
                msg.data[ (j-imin[1])*width + (i-imin[0]) ] = -1;
        }

    msg.info.width = width;
    msg.info.height = height;
    msg.info.resolution = map->resolution;

    msg.info.origin.position.x = min[0];
    msg.info.origin.position.y = min[1];
    msg.info.origin.position.z = 0;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, 0.0);
    msg.info.origin.orientation.x = quat_tf.getX();
    msg.info.origin.orientation.y = quat_tf.getY();
    msg.info.origin.orientation.z = quat_tf.getZ();
    msg.info.origin.orientation.w = quat_tf.getW();
    //tf2::convert(quat_tf, msg.info.origin.orientation);
    //msg.info.origin.orientation = tf2_ros::createQuaternionMsgFromYaw(0);

    return true;
}

void lama::PFSlam2DROS::onGetMap(const std::shared_ptr <nav_msgs::srv::GetMap::Request> req,
                                 std::shared_ptr <nav_msgs::srv::GetMap::Response> res) {
    res->map.header.frame_id = global_frame_id_;
    res->map.header.stamp = ros_clock->now();

    OccupancyMsgFromOccupancyMap(res->map);
}

void lama::PFSlam2DROS::printSummary() {
    if (slam2d_->summary)
        std::cout << slam2d_->summary->report() << std::endl;
}

int main(int argc, char *argv[]) {
    std::cout << argc << " params: " << std::endl;
    for(int i=0; i<argc; ++i){
        std::cout << "  " << argv[i] << std::endl;
    }

    rclcpp::init(argc, argv);
    lama::PFSlam2DROS slam2d_ros {"pf_slam2d_ros"};
    slam2d_ros.node->declare_parameter("rosbag");

    std::string rosbag_filename;
    if( !slam2d_ros.node->get_parameter("rosbag", rosbag_filename ) || rosbag_filename.empty()) {
        RCLCPP_INFO(slam2d_ros.node->get_logger(), "Running SLAM in Live Mode");
    } else{
        RCLCPP_INFO(slam2d_ros.node->get_logger(), "Running SLAM in Rosbag Mode (offline)");
        lama_utils::ReplayRosbag(slam2d_ros.node, rosbag_filename);

        if (rclcpp::ok())
            slam2d_ros.printSummary();

        RCLCPP_INFO(slam2d_ros.node->get_logger(), "You can now save your map. Use ctrl-c to quit.");
        // publish the maps a last time
        slam2d_ros.publishMaps();
    }

    rclcpp::spin(slam2d_ros.node);
    rclcpp::shutdown();
    return 0;
}

