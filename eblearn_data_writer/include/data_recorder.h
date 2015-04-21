/*
 * data_recorder.h
 * Description: write collected data to disk, for eblearn
 *
 * Copyright (c) 2013, Centre for Intelligent Mechatronics Systems, University of Technology, Sydney, Australia.
 * All rights reserved.
 *
 * This software was developed as a part of an industry based research project on Assistive Robotic Devices.
 *
 * Author: Kanzhi Wu
 * Date: 13/03/2015
 *
 */

#ifndef DATA_RECORDER_H
#define DATA_RECORDER_H


#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <ros/ros.h>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


class DataRecorder{
private:
    ros::NodeHandle * nh_;
    ros::Subscriber image_sub_;

    std::string dir_;
    int count_;
    cv::Mat image_;
    int box_size_;
    cv::Rect rect_;
    bool start_;

    void image_callback( const sensor_msgs::ImageConstPtr & image_msg );

public:
    DataRecorder( ros::NodeHandle nh, std::string dir );

    ~DataRecorder();
};


#endif
