#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_msgs/TFMessage.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/LaserScan.h>

void ReplayRosbag(ros::NodeHandle& pnh, const std::string& rosbag_filename)
{
  std::string scan_topic;
  pnh.param("scan_topic", scan_topic, std::string("/scan"));

  rosbag::Bag bag;

  try {
    ROS_INFO("Opening rosbag [%s]", rosbag_filename.c_str());
    bag.open(rosbag_filename, rosbag::bagmode::Read);
  }
  catch (std::exception& ex)
  {
    ROS_FATAL("Unable to open rosbag [%s]: %s", rosbag_filename.c_str(), ex.what());
  }

  ROS_INFO("%d",bag.getCompression());

  auto pub_scan  = pnh.advertise<sensor_msgs::LaserScan>(scan_topic,10);
  auto pub_tf  = pnh.advertise<tf2_msgs::TFMessage>("/tf",10);
  auto pub_tf_static  = pnh.advertise<tf2_msgs::TFMessage>("/tf_static",10, true);
  auto pub_clock = pnh.advertise<rosgraph_msgs::Clock>("/clock",10);

  std::vector<std::string> topics = {scan_topic, "/tf", "/tf_static"};
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  auto start_real_time = ros::WallTime::now();
  auto start_sim_time = view.getBeginTime();
  auto prev_real_time = start_real_time;
  auto prev_sim_time = start_sim_time;
  int num_scans = 0;

  for(const rosbag::MessageInstance& m: view)
  {
    if( m.getTopic() == "/tf")
    {
      pub_tf.publish(m);
    }
    else if( m.getTopic() == scan_topic)
    {
      pub_scan.publish(m);
      num_scans++;
    }
    else  if( m.getTopic() == "/tf_static")
    {
      pub_tf_static.publish(m);
    }

    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = m.getTime();
    pub_clock.publish( clock_msg );

    auto real_time = ros::WallTime::now();
    if( real_time - prev_real_time > ros::WallDuration(1) )
    {
      auto sim_time = m.getTime();
      auto delta_real = (real_time - prev_real_time).toSec();
      auto delta_sim  = (sim_time - prev_sim_time).toSec();
      ROS_INFO("Processing the rosbag at %.1fX speed.", delta_sim / delta_real);
      prev_sim_time = sim_time;
      prev_real_time = real_time;
    }

    ros::spinOnce();
  }
  auto real_time = ros::WallTime::now();
  auto delta_real = (real_time - start_real_time).toSec();
  auto delta_sim  = (view.getEndTime() - start_sim_time).toSec();
  ROS_INFO("--------- Mapping Completed ---------");
  ROS_INFO("Processed the rosbag at %.1fX speed.", delta_sim / delta_real);
  ROS_INFO("Number of processed Laserscan messages: %d", num_scans);

  bag.close();
}
