/*
 * data_publisher.h
 * Description: publish collected data to specific topics
 *
 * Copyright (c) 2013, Centre for Intelligent Mechatronics Systems, University of Technology, Sydney, Australia.
 * All rights reserved.
 *
 * This software was developed as a part of an industry based research project on Assistive Robotic Devices.
 *
 * Author: Kanzhi Wu
 * Date: 08/03/2015
 *
 */

#ifndef DATA_PUBLISHER_H
#define DATA_PUBLISHER_H

#include <apc_msgs/DataPublish.h>
#include <apc_msgs/RecogStatus.h>
#include <apc_msgs/BinObjects.h>
#include <apc_msgs/Object.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>

#include <iostream>
#include <string>


class DataPublisher{
private:
    ros::NodeHandle *nh_;

    std::string dir_;
    int count_;
    int n_frames_;

    ros::Publisher xtion_rgb_pub_;
    ros::Publisher xtion_rgb_info_pub_;
    ros::Publisher xtion_depth_pub_;
    ros::Publisher xtion_depth_info_pub_;
    ros::Publisher xtion_cloud_pub_;
    ros::Publisher camera_rgb_pub_;
    ros::Publisher camera_rgb_info_pub_;

    sensor_msgs::Image xtion_rgb_msg_;
    sensor_msgs::CameraInfo xtion_rgb_info_msg_;
    sensor_msgs::Image xtion_depth_msg_;
    sensor_msgs::CameraInfo xtion_depth_info_msg_;
    sensor_msgs::PointCloud2 xtion_cloud_msg_;

    sensor_msgs::Image camera_rgb_msg_;
    sensor_msgs::CameraInfo camera_rgb_info_msg_;

    ros::ServiceClient client_;


    ros::ServiceServer recog_server_;

    ros::Subscriber obj_sub_;

    boost::thread publish_thread_;

    bool recog_completed_;
    boost::mutex recog_completed_mutex_;
    boost::condition_variable recog_completed_cond_;

public:
    DataPublisher( ros::NodeHandle & nh, std::string dir, int n );

    void recog_callback( const apc_msgs::BinObjectsConstPtr & objs_msg );

    bool recog_srv_callback( apc_msgs::RecogStatus::Request & req,
                             apc_msgs::RecogStatus::Response & resp );

    void info_publisher();

    ~DataPublisher();
};








#endif
