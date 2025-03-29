//
// Created by david on 30-03-2020.
//

#ifndef IRIS_LAMA_ROS2_UTILS_H
#define IRIS_LAMA_ROS2_UTILS_H

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/message_filter.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2/utils.h"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "lama/pose3d.h"
#include "lama/pose2d.h"
#include "lama/sdm/occupancy_map.h"
#include "lama/image.h"

namespace lama_utils {

    /*
     * Returns a Geometry TransformStamped message, given a tf2::Transform,
     * a rclcpp::Time timestamp, and the global_frame and child_frame std::string's
     */
    geometry_msgs::msg::TransformStamped
    createTransformStamped(const tf2::Transform &myTransform,
                           const rclcpp::Time &myTime,
                           const std::string &global_frame,
                           const std::string &child_frame) {
        // this is better but requires a tf2::TimePoint
        // https://github.com/ros2/geometry2/blob/72b5b179df1818a81290631cb20578e279bffcb3/tf2_geometry_msgs/include/tf2_geometry_msgs/tf2_geometry_msgs.h#L510
        // geometry_msgs::msg::TransformStamped msg2 =
        //    tf2::toMsg(tf2::Stamped<tf2::Transform>(myTransform, tf2::TimePoint(), global_frame));
        // msg.child_frame_id = child_frame;

        geometry_msgs::msg::TransformStamped msg;
        msg.transform.translation.x = myTransform.getOrigin().x();
        msg.transform.translation.y = myTransform.getOrigin().y();
        msg.transform.translation.z = myTransform.getOrigin().z();
        msg.transform.rotation.x = myTransform.getRotation().x();
        msg.transform.rotation.y = myTransform.getRotation().y();
        msg.transform.rotation.z = myTransform.getRotation().z();
        msg.transform.rotation.w = myTransform.getRotation().w();
        msg.child_frame_id = child_frame;
        msg.header.frame_id = global_frame;
        msg.header.stamp = myTime;

        return msg;
    }

    /*
     * Returns a Geometry PoseStamped message, given a tf2::Transform,
     * a rclcpp::Time timestamp, and the std::string global_frame
     */
    geometry_msgs::msg::PoseStamped
    createPoseStamped(const tf2::Transform &myTransform, const rclcpp::Time &myTime,
                      const std::string &global_frame) {
        geometry_msgs::msg::PoseStamped msg;

        msg.pose.position.x = myTransform.getOrigin().x();
        msg.pose.position.y = myTransform.getOrigin().y();
        msg.pose.position.z = myTransform.getOrigin().z();
        msg.pose.orientation.x = myTransform.getRotation().x();
        msg.pose.orientation.y = myTransform.getRotation().y();
        msg.pose.orientation.z = myTransform.getRotation().z();
        msg.pose.orientation.w = myTransform.getRotation().w();
        msg.header.frame_id = global_frame;
        msg.header.stamp = myTime;

        return msg;
    }

    /*
     * Returns a Geometry Vector3Stamped message, given a tf2::Vector3,
     * a rclcpp::Time timestamp, and the std::string global_frame
     */
    geometry_msgs::msg::Vector3Stamped
    createVector3Stamped(const tf2::Vector3 &myVector, const rclcpp::Time &myTime,
                         const std::string &global_frame) {
        geometry_msgs::msg::Vector3Stamped msg;

        msg.vector.x = myVector.x();
        msg.vector.y = myVector.y();
        msg.vector.z = myVector.z();
        msg.header.frame_id = global_frame;
        msg.header.stamp = myTime;

        return msg;
    }

    /*
     * Returns a TF2 Stamped<Transform>, given an equivalent Geometry PoseStamped message
     */
    tf2::Stamped <tf2::Transform>
    createStampedTransform(const geometry_msgs::msg::PoseStamped &myPoseStamped) {
        // Should we use geometry_msgs::msg::TransformStamped?
        // tf2::Stamped <tf2::Transform> converted2;
        // fromMsg(myPoseStamped, converted2);

        // https://answers.ros.org/question/261419/tf2-transformpose-in-c/

        tf2::Stamped <tf2::Transform> converted(
                tf2::Transform(
                        tf2::Quaternion(myPoseStamped.pose.orientation.x, myPoseStamped.pose.orientation.y,
                                        myPoseStamped.pose.orientation.z, myPoseStamped.pose.orientation.w),
                        tf2::Vector3(myPoseStamped.pose.position.x, myPoseStamped.pose.position.y,
                                     myPoseStamped.pose.position.z)),
                tf2_ros::fromMsg(myPoseStamped.header.stamp), myPoseStamped.header.frame_id);
        return converted;
    }

    /*
     * Returns a TF2 Stamped<Vector3>, given an equivalent Geometry Vector3Stamped message
     */
    tf2::Stamped <tf2::Vector3>
    createStampedVector3(const geometry_msgs::msg::Vector3Stamped &myVectorStamped) {
        // TODO why doesnt this work....
        // http://wiki.ros.org/tf2/Tutorials/Migration/DataConversions
        // geometry_msgs::msg::Vector3Stamped m;
        // tf2::convert(up, m);
        // no matching function for call to ‘toMsg(const tf2::Stamped<tf2::Vector3>&)

        tf2::Stamped <tf2::Vector3> converted(
                tf2::Vector3(myVectorStamped.vector.x, myVectorStamped.vector.y, myVectorStamped.vector.z),
                tf2_ros::fromMsg(myVectorStamped.header.stamp), myVectorStamped.header.frame_id);
        return converted;
    }

    /*
     * Returns the 'yaw' component (double) of a TF2 Quaternion
     */
    tf2Scalar getYaw(tf2::Quaternion q) {
        // https://github.com/ros2/geometry2/blob/ros2/tf2_geometry_msgs/test/test_tf2_geometry_msgs.cpp
        // https://answers.ros.org/question/339528/quaternion-to-rpy-ros2/
        // http://docs.ros.org/jade/api/tf2/html/classtf2_1_1Transform.html
        // http://docs.ros.org/jade/api/tf/html/c++/Transform_8h_source.html

        tf2::Matrix3x3 matrix3x3(q);
        tf2Scalar useless_pitch, useless_roll, laser_origin_yaw;
        matrix3x3.getRPY(useless_pitch, useless_roll, laser_origin_yaw);

        return laser_origin_yaw;
    }

    /*
     * Returns the 'yaw' component (double) of a Geometry Quaternion message
     */
    tf2Scalar getYaw(geometry_msgs::msg::Quaternion q_msg) {
        // https://github.com/ros2/geometry2/blob/ros2/tf2_geometry_msgs/test/test_tf2_geometry_msgs.cpp
        // https://answers.ros.org/question/339528/quaternion-to-rpy-ros2/
        tf2::Quaternion q;
        tf2::convert(q_msg, q);

        return getYaw(q);
    }

    /**
     * @brief Convert lama::Pose2D object to tf2::Transform object
     * 
     * @param pose Pose to be converted
     * @return tf2::Transform 
     */
    tf2::Transform createTransform(const lama::Pose2D& pose) {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, pose.rotation());
        return tf2::Transform(q, tf2::Vector3(pose.x(), pose.y(), 0.0));
    }

    /**
     * @brief Create lama::PointCloudXYZ::Ptr object from sensor_msgs::msg::LaserScan object
     * 
     * @param laser_scan LaserScan message to be converted
     * @param transform_base_to_scan Transform message holding transformation data from base to scan
     * @param beam_step Beam step of the laser scan
     * @param min_laser_range Minimum range of the laser scan
     * @param max_laser_range Maximum range of the laser scan
     * @return lama::PointCloudXYZ::Ptr 
     */
    lama::PointCloudXYZ::Ptr createPointCloud(
        const sensor_msgs::msg::LaserScan& laser_scan, 
        const geometry_msgs::msg::Transform& transform_base_to_scan, 
        const std::size_t beam_step, 
        const float min_laser_range, 
        const float max_laser_range) {
        float max_range, min_range;
        if((max_laser_range == 0.0) || (max_laser_range > laser_scan.range_max)) {
            max_range = laser_scan.range_max;
        }
        else {
            max_range = max_laser_range;
        }

        if((min_laser_range == 0.0) || (min_laser_range < laser_scan.range_min)) {
            min_range = laser_scan.range_min;
        }
        else {
            min_range = min_laser_range;
        }

        float angle_min = laser_scan.angle_min;
        float angle_inc = laser_scan.angle_increment;

        auto q = tf2::Quaternion(
            transform_base_to_scan.rotation.x, 
            transform_base_to_scan.rotation.y, 
            transform_base_to_scan.rotation.z, 
            transform_base_to_scan.rotation.w);
        double yaw, pitch, roll;
        tf2::getEulerYPR(q, yaw, pitch, roll);

        auto sensor_origin = lama::Pose3D(
            transform_base_to_scan.translation.x, 
            transform_base_to_scan.translation.y, 
            transform_base_to_scan.translation.z, 
            roll, 
            pitch, 
            yaw);

        lama::PointCloudXYZ::Ptr cloud(new lama::PointCloudXYZ);
        cloud->sensor_origin_ = sensor_origin.xyz();
        cloud->sensor_orientation_ = Eigen::Quaterniond(sensor_origin.state.so3().matrix());
        cloud->points.reserve(laser_scan.ranges.size());

        for(std::size_t i = 0; i < laser_scan.ranges.size(); i += beam_step) {
            const float range = laser_scan.ranges[i];

            if(!std::isfinite(range)) {
                continue;
            }

            if((range >= max_range) || (range <= min_range)) {
                continue;
            }

            cloud->points.push_back(Eigen::Vector3d(range * std::cos(angle_min + (i*angle_inc)), range * std::sin(angle_min + (i*angle_inc)), 0));
        }

        return cloud;
    }

    /**
     * @brief Create nav_msgs::msg::OccupancyGrid object from lama::OccupancyMap object
     * 
     * @param map Map to be converted
     * @param frame_id Frame id to be set on the message
     * @param stamp Timestamp to be set on the message
     * @return nav_msgs::msg::OccupancyGrid 
     */
    nav_msgs::msg::OccupancyGrid createOccupancyGrid(
        const lama::OccupancyMap& map, 
        const std::string& frame_id, 
        const rclcpp::Time& stamp) {
        nav_msgs::msg::OccupancyGrid message;
        message.header.frame_id = frame_id;
        message.header.stamp = stamp;

        Eigen::Vector3ui imin, imax;
        map.bounds(imin, imax);

        unsigned int width = imax(0) - imin(0);
        unsigned int height = imax(1) - imin(1);

        if((width == 0) || (height == 0)) {
            return message;
        }

        message.data.resize(width * height, -1);
        map.visit_all_cells([&message, &map, &imin, width](const lama::Vector3ui& coords) {
            Eigen::Vector3ui adj_coords = coords - imin;

            if(map.isFree(coords)) {
                message.data[adj_coords(1) * width + adj_coords(0)] = 0;
            }
            else if(map.isOccupied(coords)) {
                message.data[adj_coords(1) * width + adj_coords(0)] = 100;
            }
        });

        message.info.width = width;
        message.info.height = height;
        message.info.resolution = map.resolution;

        Eigen::Vector3d pos = map.m2w(imin);
        message.info.origin.position.x = pos.x();
        message.info.origin.position.y = pos.y();
        message.info.origin.position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        message.info.origin.orientation = tf2::toMsg(q);

        return message;
    }

    /**
     * @brief Manage LaMa's 2D visualization markers
     * 
     * @tparam SlamType 
     */
    template <typename SlamType>
    class MarkersManager2D {
    public:
        /**
         * @brief Construct a new Markers Manager 2D object
         * 
         * @param slam Slam executor
         * @param frame_id Frame id to be set on the markers managed by this MarkersManager2D instance
         * @param sphere_maker_scale Scale factor of SPHERE_LIST
         * @param line_maker_scale Scale factor of LINE_STRIP and LINE_LIST
         */
        MarkersManager2D(
            const SlamType& slam, 
            const std::string& frame_id, 
            const double sphere_maker_scale = 0.25,
            const double line_maker_scale = 0.05)
        : slam_(slam) {
            n_sphere_list_markers_ = 0;
            n_line_strip_markers_ = 0;
            n_line_list_markers_ = 0;

            sphere_list_marker_.header.frame_id = frame_id;
            sphere_list_marker_.ns = "pose";
            sphere_list_marker_.id = 0;
            sphere_list_marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            sphere_list_marker_.action = visualization_msgs::msg::Marker::ADD;
            sphere_list_marker_.scale.x = sphere_maker_scale;
            sphere_list_marker_.scale.y = sphere_maker_scale;
            sphere_list_marker_.scale.z = sphere_maker_scale;
            sphere_list_marker_.pose.orientation.w = 1.0;
            
            line_strip_marker_.header.frame_id = frame_id;
            line_strip_marker_.ns = "odom";
            line_strip_marker_.id = 1;
            line_strip_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_strip_marker_.action = visualization_msgs::msg::Marker::ADD;
            line_strip_marker_.scale.x = line_maker_scale;
            line_strip_marker_.scale.y = line_maker_scale;
            line_strip_marker_.scale.z = line_maker_scale;
            line_strip_marker_.color.r = 0;
            line_strip_marker_.color.g = 0;
            line_strip_marker_.color.b = 1;
            line_strip_marker_.color.a = 1;
            line_strip_marker_.pose.orientation.w = 1.0;

            line_list_marker_.header.frame_id = frame_id;
            line_list_marker_.ns = "loop";
            line_list_marker_.id = 2;
            line_list_marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
            line_list_marker_.action = visualization_msgs::msg::Marker::ADD;
            line_list_marker_.scale.x = line_maker_scale;
            line_list_marker_.scale.y = line_maker_scale;
            line_list_marker_.scale.z = line_maker_scale;
            line_list_marker_.color.r = 0;
            line_list_marker_.color.g = 1;
            line_list_marker_.color.b = 0;
            line_list_marker_.color.a = 1;
            line_list_marker_.pose.orientation.w = 1.0;
        }

        /**
         * @brief Clear all markers managed by this MarkersManager2D instance
         * 
         */
        void clearMarkers() {
            n_sphere_list_markers_ = 0;
            n_line_strip_markers_ = 0;
            n_line_list_markers_ = 0;

            sphere_list_marker_.points.clear();
            sphere_list_marker_.colors.clear();
            line_strip_marker_.points.clear();
            line_strip_marker_.colors.clear();
            line_list_marker_.points.clear();
            line_list_marker_.colors.clear();
        }

        /**
         * @brief Update all markers
         * 
         * @param stamp Timestamp to be set on the marker message
         */
        void update(const rclcpp::Time& stamp) {
            updateSphereListMarker(stamp);
            updateLineStripMarker(stamp);
            updateLineListMarker(stamp);
        }

        /**
         * @brief Get the latest MarkerArray message
         * 
         * @return visualization_msgs::msg::MarkerArray 
         */
        visualization_msgs::msg::MarkerArray getMarkers() {            
            visualization_msgs::msg::MarkerArray markers;
            markers.markers.push_back(sphere_list_marker_);
            markers.markers.push_back(line_strip_marker_);
            markers.markers.push_back(line_list_marker_);
            return markers;
        }

    private:
        /**
         * @brief Update the sphere list marker
         * 
         * @param stamp Timestamp to be set on the marker message
         * @return std::size_t Number of sphere markers added in this call
         */
        std::size_t updateSphereListMarker(const rclcpp::Time& stamp) {
            sphere_list_marker_.header.stamp = stamp;

            if(slam_.key_poses.size() <= 0) {
                return 0;
            }

            geometry_msgs::msg::Point p;
            for(auto it = slam_.key_poses.begin() + n_sphere_list_markers_; it != slam_.key_poses.end(); it ++) {
                const auto& key_pose = *it;
                p.x = key_pose.pose.x();
                p.y = key_pose.pose.y();
                p.z = 0.0;
                sphere_list_marker_.points.push_back(p);
            }

            // Re-render marker color
            sphere_list_marker_.colors.clear();
            std_msgs::msg::ColorRGBA color;
            for(auto it = slam_.key_poses.begin(); it != slam_.key_poses.end(); it ++) {
                const auto& key_pose = *it;
                float a = static_cast<float>(key_pose.id) / static_cast<float>(slam_.key_poses.size());
                color.r = a;
                color.g = 1.0 - a;
                color.b = 0.0;
                color.a = 1.0;
                sphere_list_marker_.colors.push_back(color);
            }

            std::size_t n_added = slam_.key_poses.size() - n_sphere_list_markers_;
            n_sphere_list_markers_ = slam_.key_poses.size();
            return n_added;
        }

        /**
         * @brief Update the line strip marker
         * 
         * @param stamp Timestamp to be set on the marker message
         * @return std::size_t Number of line strip markers added in this call
         */
        std::size_t updateLineStripMarker(const rclcpp::Time& stamp) {
            line_strip_marker_.header.stamp = stamp;
            
            if(slam_.key_poses.size() <= 0) {
                return 0;
            }

            geometry_msgs::msg::Point p;
            for(auto it = slam_.key_poses.begin() + n_line_strip_markers_; it != slam_.key_poses.end(); it ++) {
                const auto& key_pose = *it;
                p.x = key_pose.pose.x();
                p.y = key_pose.pose.y();
                p.z = 0.0;
                line_strip_marker_.points.push_back(p);
            }

            std::size_t n_added = slam_.key_poses.size() - n_line_strip_markers_;
            n_line_strip_markers_ = slam_.key_poses.size();
            return n_added;
        }

        /**
         * @brief Update the line list marker
         * 
         * @param stamp Timestamp to be set on the marker message
         * @return std::size_t Number of line markers added in this call
         */
        std::size_t updateLineListMarker(const rclcpp::Time& stamp) {
            line_list_marker_.header.stamp = stamp;
            
            if(slam_.links.size() <= 0) {
                return 0;
            }

            for(auto it = slam_.links.begin() + n_line_list_markers_; it != slam_.links.end(); it ++) {
                const auto& link = *it;
                geometry_msgs::msg::Point p;

                p.x = slam_.key_poses[link.first].pose.x();
                p.y = slam_.key_poses[link.first].pose.y();
                p.z = 0.0;
                line_list_marker_.points.push_back(p);

                p.x = slam_.key_poses[link.second].pose.x();
                p.y = slam_.key_poses[link.second].pose.y();
                p.z = 0.0;
                line_list_marker_.points.push_back(p);
            }

            std::size_t n_added = slam_.links.size() - n_line_list_markers_;
            n_line_list_markers_ = slam_.links.size();
            return n_added;
        }

    private:
        const SlamType& slam_;

        std::size_t n_sphere_list_markers_;
        std::size_t n_line_strip_markers_;
        std::size_t n_line_list_markers_;

        visualization_msgs::msg::Marker sphere_list_marker_;
        visualization_msgs::msg::Marker line_strip_marker_;
        visualization_msgs::msg::Marker line_list_marker_;
    };

    /*
     * Replays a rosbag2 file, logging into a ROS2 Node
     */
    void ReplayRosbag(std::shared_ptr <rclcpp::Node>& node, const std::string& rosbag_filename)
    {
        // TODO how to remap scan topic to something else? maybe impossible
        /*std::string scan_topic;
        node->get_parameter_or("scan_topic", scan_topic, std::string("/scan"));
        ROS_INFO(node->get_logger(), "Scan topic: %s", scan_topic.c_str());
    */

        // https://github.com/ros2/rosbag2/blob/master/rosbag2_tests/test/rosbag2_tests/test_rosbag2_play_end_to_end.cpp
        RCLCPP_INFO(node->get_logger(), "Opening rosbag [%s]", rosbag_filename.c_str());
        auto exitcode = std::system(("ros2 bag info "+rosbag_filename).c_str());
        if(exitcode != 0)
        {
            RCLCPP_FATAL(node->get_logger(), "Unable to open rosbag [%s]", rosbag_filename.c_str());
            return;
        }

        RCLCPP_INFO(node->get_logger(), "Allow time for the subscribers to connect");
        rclcpp::Rate r(1);
        r.sleep();

        RCLCPP_INFO(node->get_logger(), "Playing rosbag [%s]", rosbag_filename.c_str());
        exitcode = std::system(("ros2 bag play "+rosbag_filename).c_str());
        if(exitcode != 0)
        {
            RCLCPP_FATAL(node->get_logger(), "Error playing rosbag [%s]", rosbag_filename.c_str());
            return;
        }

        RCLCPP_INFO(node->get_logger(), "--------- Mapping Completed ---------");
    }


}


#endif //IRIS_LAMA_ROS2_UTILS_H
