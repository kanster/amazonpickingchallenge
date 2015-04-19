/*
 * system_calibrator.h
 * Description: generate mask image of the bins
 *
 * Copyright (c) 2013, Centre for Intelligent Mechatronics Systems, University of Technology, Sydney, Australia.
 * All rights reserved.
 *
 * This software was developed as a part of an industry based research project on Assistive Robotic Devices.
 *
 * Author: Kanzhi Wu
 * Date: 05/05/2015
 *
 */


#ifndef SYSTEM_CALIBRATOR_H
#define SYSTEM_CALIBRATOR_H

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>

#include <iostream>
#include <vector>
#include <string>


class SystemCalib{
private:
    ros::NodeHandle * nh_;

    ros::Subscriber cloud_sub_;
    std::string cloud_topic_;

    std::string dir_;

    void cloud_callback( const sensor_msgs::PointCloud2ConstPtr cloud_msg );

public:
    SystemCalib( ros::NodeHandle & nh, std::string dir );

    ~SystemCalib();


};

#endif
