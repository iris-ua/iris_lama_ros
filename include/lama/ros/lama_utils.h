//
// Created by david on 30-03-2020.
//

#ifndef IRIS_LAMA_ROS2_UTILS_H
#define IRIS_LAMA_ROS2_UTILS_H

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/message_filter.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/transform_datatypes.h"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace lama_utils {

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


    tf2Scalar getYaw(tf2::Quaternion q){
        // https://github.com/ros2/geometry2/blob/ros2/tf2_geometry_msgs/test/test_tf2_geometry_msgs.cpp
        // https://answers.ros.org/question/339528/quaternion-to-rpy-ros2/
        // http://docs.ros.org/jade/api/tf2/html/classtf2_1_1Transform.html
        // http://docs.ros.org/jade/api/tf/html/c++/Transform_8h_source.html

        tf2::Matrix3x3 matrix3x3(q);
        tf2Scalar useless_pitch, useless_roll, laser_origin_yaw;
        matrix3x3.getRPY(useless_pitch, useless_roll, laser_origin_yaw);

        return laser_origin_yaw;
    }

    tf2Scalar getYaw(geometry_msgs::msg::Quaternion q_msg){
        // https://github.com/ros2/geometry2/blob/ros2/tf2_geometry_msgs/test/test_tf2_geometry_msgs.cpp
        // https://answers.ros.org/question/339528/quaternion-to-rpy-ros2/
        tf2::Quaternion q;
        tf2::convert(q_msg, q);

        return getYaw(q);
    }



}


#endif //IRIS_LAMA_ROS2_UTILS_H
