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

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_msgs/TFMessage.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/LaserScan.h>

#include <lama/time.h>

#include "lama/ros/offline_replay.h"

#include <algorithm>

void lama::ReplayRosbag(ros::NodeHandle& pnh, const std::string& rosbag_filename)
{
    std::string scan_topic;
    pnh.param("scan_topic", scan_topic, std::string("/scan"));
    int queue_length;
    pnh.param("queue_length", queue_length, 100);

    ROS_INFO("Scan topic: %s", scan_topic.c_str());

    rosbag::Bag bag;
    try {
        ROS_INFO("Opening rosbag [%s]", rosbag_filename.c_str());
        bag.open(rosbag_filename, rosbag::bagmode::Read);
    }
    catch (std::exception& ex) {
        ROS_FATAL("Unable to open rosbag [%s]: %s", rosbag_filename.c_str(), ex.what());
    }

    auto pub_scan  = pnh.advertise<sensor_msgs::LaserScan>(scan_topic, queue_length);
    auto pub_tf  = pnh.advertise<tf2_msgs::TFMessage>("/tf", queue_length);
    auto pub_tf_static  = pnh.advertise<tf2_msgs::TFMessage>("/tf_static", queue_length, true);
    auto pub_clock = pnh.advertise<rosgraph_msgs::Clock>("/clock", queue_length);

    ROS_INFO("Allow time for the subscribers to connect");
    ros::WallDuration wait(2); wait.sleep();

    std::vector<std::string> topics = {scan_topic, "/tf", "/tf_static"};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    auto start_real_time = ros::WallTime::now();
    auto start_sim_time = view.getBeginTime();
    auto prev_real_time = start_real_time;
    auto prev_sim_time = start_sim_time;
    int num_scans = 0;

    ros::WallRate rate(5000); // 5Mhz
    for(const rosbag::MessageInstance& m: view)
    {
        rosgraph_msgs::Clock clock_msg;
        clock_msg.clock = m.getTime();
        pub_clock.publish( clock_msg );

        if (m.getTopic() == "/tf") {
            const auto msg = m.instantiate<tf2_msgs::TFMessage>();
            if (msg)
            {
                if (std::none_of(msg->transforms.begin(), msg->transforms.end(),
                                 [](const auto& transform) { return transform.header.frame_id == "map"; }))
                {
                    pub_tf.publish(m);
                }
            }
        } else if (m.getTopic() == "/tf_static") {
            pub_tf_static.publish(m);
        } else if( m.getTopic() == scan_topic) {
            pub_scan.publish(m);
            num_scans++;

            // Publishing the topics as fast as possible resulted in some weird
            // behaviors where the odometry would create big jumps and the tf buffer
            // would have to extrapolate data inside a tf syncronized callback.
            // Sleeping solved the problem. Why? does it matter? Maybe!
            rate.sleep();
        }

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
        if (!ros::ok()) // stop on ctrl-c
            break;
    }

    auto real_time = ros::WallTime::now();
    auto delta_real = (real_time - start_real_time).toSec(); auto delta_sim  = (view.getEndTime() - start_sim_time).toSec();
    ROS_INFO("--------- Mapping Completed ---------");
    ROS_INFO("Processed the rosbag at %.1fX speed.", delta_sim / delta_real);
    ROS_INFO("Number of processed Laserscan messages: %d", num_scans);

    bag.close();
}
