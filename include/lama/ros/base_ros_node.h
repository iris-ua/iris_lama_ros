
#pragma once

#include <ros/ros.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <rosgraph_msgs/Clock.h>

#include <message_filters/subscriber.h>

#include <tf2/utils.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>

#include <lama/pose2d.h>
#include <lama/pose3d.h>

namespace lama {

inline std::string strip_slash(const std::string& name)
{
    auto out = name;
    if (not out.empty() and out[0] == '/')
        out.erase(0,1);
    return out;
}

template <typename MsgType>
struct BaseROSNode {

    ros::NodeHandle nh;

    tf2_ros::Buffer* tf_buffer;
    tf2_ros::TransformListener* tf_listener;
    tf2_ros::TransformBroadcaster* tf_broadcaster;

    tf2_ros::MessageFilter<MsgType>* message_filter;
    message_filters::Subscriber<MsgType>* data_sub;

    tf2::Transform latest_tf;
    ros::Duration transform_tolerance;
    bool publish_tf;

    std::string odom_frame;
    std::string base_frame;
    std::string global_frame;

    std::string data_topic;

    std::map<std::string, Pose3D> sensor_origin_map;

    BaseROSNode(const ros::NodeHandle& node_handle);
    virtual ~BaseROSNode();

    virtual void onData(const typename MsgType::ConstPtr& data) = 0;
    virtual void saveLog(double runtime){};

    double runFromBag(const std::string& bagname);

    //>> useful functions

    Pose3D getSensorPose(const std::string& sensor_frame);
    bool getOdometry(Pose2D& odom, const ros::Time& stamp);

    bool publishTF(const Pose2D& pose, const ros::Time& stamp);
    bool publishTF(const ros::Time& stamp);

};

template <typename MsgType>
BaseROSNode<MsgType>::BaseROSNode(const ros::NodeHandle& node_handle)
    : nh(node_handle)
{
    transform_tolerance.fromSec(nh.param("transform_tolerance", 0.1));
    publish_tf = nh.param("publish_tf", true);

    odom_frame   = strip_slash(nh.param("odom_frame_id",   std::string("odom")));
    base_frame   = strip_slash(nh.param("base_frame_id",   std::string("base_link")));
    global_frame = strip_slash(nh.param("global_frame_id", std::string("map")));

    data_topic = nh.param("scan_topic", std::string("/scan"));

    tf_buffer = new tf2_ros::Buffer();
    tf_listener = new tf2_ros::TransformListener(*tf_buffer);
    tf_broadcaster = new tf2_ros::TransformBroadcaster();

    data_sub = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, data_topic, 20, ros::TransportHints().tcpNoDelay());
    message_filter = new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*data_sub, *tf_buffer, odom_frame, 20, nh);
    message_filter->registerCallback(boost::bind(&BaseROSNode::onData, this, _1));
}

template <typename MsgType>
BaseROSNode<MsgType>::~BaseROSNode()
{
    delete message_filter;
    delete data_sub;
    delete tf_broadcaster;
    delete tf_listener;
    delete tf_buffer;
}

template <typename MsgType>
double BaseROSNode<MsgType>::runFromBag(const std::string& bagname)
{
    rosbag::Bag bag;

    try { bag.open(bagname,rosbag::bagmode::Read); }
    catch (rosbag::BagException& e) {
        ROS_ERROR("%s", e.what());
        return 0.0;
    }// end catch

    std::vector<std::string> topics = {"tf", "tf_static", "/tf", "/tf_static", data_topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    auto start_sim_time = view.getBeginTime().toSec();
    auto end_sim_time   = view.getEndTime().toSec();
    auto bag_time_span  = (uint32_t)std::round(end_sim_time - start_sim_time);
    ROS_INFO("Bag time span: %d minute(s) and %d second(s)", bag_time_span / 60, bag_time_span % 60);

    int queue_size = 100;
    auto bag_data_pub  = nh.advertise<MsgType>(data_topic, queue_size);
    auto pub_clock = nh.advertise<rosgraph_msgs::Clock>("/clock",   queue_size);
    auto tf_pub  = nh.advertise<tf2_msgs::TFMessage>("/tf", queue_size);
    auto tfs_pub = nh.advertise<tf2_msgs::TFMessage>("/tf_static", queue_size);

    // wait a little so the listeners can connect
    ROS_INFO("Waiting 2 seconds for listeners");
    ros::WallDuration(2).sleep();

    // we do not need the tf listener, we get our tf data directly from the bag
    // to prevent the TF_REPEATED_DATA warning.
    delete tf_listener;
    tf_listener = nullptr;

    double span = 0;

    for (auto& msg : view){
        if (not ros::ok()) break;

        rosgraph_msgs::Clock clock_msg;
        clock_msg.clock = msg.getTime();
        pub_clock.publish(clock_msg);

        auto topic = strip_slash(msg.getTopic());
        if (topic == "tf" || topic == "tf_static"){

            auto tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
            bool is_static = (topic == "tf_static");

            for (auto& tf : tf_msg->transforms)
                tf_buffer->setTransform(tf, "rosbag_auth", is_static);

            if (is_static) tfs_pub.publish(tf_msg);
            else           tf_pub.publish(tf_msg);

        } else {

            auto data_msg = msg.instantiate<MsgType>();
            bag_data_pub.publish(data_msg);
        }// end if

        // process data in the queues
        auto start_time = ros::WallTime::now();

        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration());

        span += (ros::WallTime::now() - start_time).toSec();

    }// end for

    bag.close();

    auto time_span = span; //(double) sstd::round(span);
    ROS_INFO("--------- Mapping Completed ---------");
    ROS_INFO("Bag processed in %d minute(s) and %f second(s)", (int)(time_span / 60), std::fmod(time_span, 60));
    ROS_INFO("Mapping speedup is %.fX", bag_time_span / (double)(time_span));

    return span;
}

template <typename MsgType>
Pose3D BaseROSNode<MsgType>::getSensorPose(const std::string& sensor_frame)
{
    // Check if it is a know sensor. If it is return its origin.
    auto iter = sensor_origin_map.find(sensor_frame);
    if (iter != sensor_origin_map.end())
        return iter->second;

    // The sensor is unknown, lets retrieve its origin and save it.
    geometry_msgs::PoseStamped identity;
    identity.header.frame_id = strip_slash(sensor_frame);
    identity.header.stamp = ros::Time();

    tf2::toMsg(tf2::Transform::getIdentity(), identity.pose);

    geometry_msgs::PoseStamped sensor_origin;
    try { tf_buffer->transform(identity, sensor_origin, base_frame); }
    catch (tf2::TransformException& e) {
        ROS_ERROR("Couldn't transform from %s to %s: %s",
                sensor_frame.c_str(), base_frame.c_str(), e.what());
        return Pose3D();
    }

    tf2::Transform tx;
    tf2::convert(sensor_origin.pose, tx);

    double yaw, pitch, roll;
    tf2::getEulerYPR(tx.getRotation(), yaw, pitch, roll);

    Pose3D so = Pose3D(tx.getOrigin().x(),
                       tx.getOrigin().y(),
                       tx.getOrigin().z(),
                       roll, pitch, yaw);
    sensor_origin_map[sensor_frame] = so;

    ROS_INFO("New laser configured (frame_id=%s)", sensor_frame.c_str() );
    return so;
}

template <typename MsgType>
bool BaseROSNode<MsgType>::getOdometry(Pose2D& odom, const ros::Time& stamp)
{
    geometry_msgs::PoseStamped identity;
    identity.header.frame_id = base_frame;
    identity.header.stamp = stamp;

    tf2::toMsg(tf2::Transform::getIdentity(), identity.pose);

    geometry_msgs::PoseStamped odom_pose;
    try { tf_buffer->transform(identity, odom_pose, odom_frame); }
    catch (tf2::TransformException& e) {
        ROS_ERROR("Failed to compute odom pose: %s", e.what());
        return false;
    }

    odom = Pose2D(odom_pose.pose.position.x, odom_pose.pose.position.y, tf2::getYaw(odom_pose.pose.orientation));
    return true;
}

template <typename MsgType>
bool BaseROSNode<MsgType>::publishTF(const Pose2D& pose, const ros::Time& stamp)
{
    if (not publish_tf) return true;

    tf2::Quaternion q;
    q.setRPY(0.0,0.0, pose.rotation());

    tf2::Transform tx(q, tf2::Vector3(pose.x(), pose.y(), 0.0));

    geometry_msgs::PoseStamped tmp_tf;
    tmp_tf.header.frame_id = base_frame;
    tmp_tf.header.stamp = stamp;

    // subtracting base to odom from map to base and send map to odom instead
    tf2::toMsg(tx.inverse(), tmp_tf.pose);

    geometry_msgs::PoseStamped odom_to_map;
    try { tf_buffer->transform(tmp_tf, odom_to_map, odom_frame); }
    catch (tf2::TransformException& e) {
        ROS_ERROR("Failed to subtract base to odom transform: %s", e.what());
        return false;
    }

    tf2::convert(odom_to_map.pose, latest_tf);

    geometry_msgs::TransformStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id  = global_frame;
    tmp_tf_stamped.header.stamp     = stamp;
    tmp_tf_stamped.child_frame_id   = odom_frame;

    tf2::convert(latest_tf.inverse(), tmp_tf_stamped.transform);
    tf_broadcaster->sendTransform(tmp_tf_stamped);
    return true;
}

template <typename MsgType>
bool BaseROSNode<MsgType>::publishTF(const ros::Time& stamp)
{
    if (not publish_tf) return true;

    geometry_msgs::TransformStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id  = global_frame;
    tmp_tf_stamped.header.stamp     = stamp;
    tmp_tf_stamped.child_frame_id   = odom_frame;

    tf2::convert(latest_tf.inverse(), tmp_tf_stamped.transform);
    tf_broadcaster->sendTransform(tmp_tf_stamped);
    return true;
}


}// namespace lama
