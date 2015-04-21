/*
 * data_writer.h
 * Description: write collected data to disk and draw correponding mask image
 *
 * Copyright (c) 2013, Centre for Intelligent Mechatronics Systems, University of Technology, Sydney, Australia.
 * All rights reserved.
 *
 * This software was developed as a part of an industry based research project on Assistive Robotic Devices.
 *
 * Author: Kanzhi Wu
 * Date: 06/03/2015
 *
 */


#ifndef DATA_WRITER_H
#define DATA_WRITER_H

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

#include <iostream>
#include <string>

class DataWriter{
private:
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,         // xtion, rgb image
    sensor_msgs::CameraInfo,    // xtion, rgb image info
    sensor_msgs::Image,         // xtion, depth image
    sensor_msgs::CameraInfo,    // xtion, depth image info
                                // !!!!above 4 topics info are not used!!!
    sensor_msgs::PointCloud2,   // xtion, point cloud
    sensor_msgs::Image,         // rgb camera, image
    sensor_msgs::CameraInfo     // rgb camera, image info
    > sensor_sync_policy;


private:
    ros::NodeHandle * nh_;
    ros::Subscriber cloud_sub_;
    std::string cloud_topic_;

    std::string dir_;

    bool use_cloud_;

    int count_;

    std::string xtion_rgb_topic_;
    std::string xtion_rgb_info_topic_;
    std::string xtion_depth_topic_;
    std::string xtion_depth_info_topic_;
    std::string xtion_cloud_topic_;
    std::string camera_rgb_topic_;
    std::string camera_rgb_info_topic_;

    message_filters::Subscriber<sensor_msgs::Image>         xtion_rgb_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>    xtion_rgb_info_sub_;
    message_filters::Subscriber<sensor_msgs::Image>         xtion_depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>    xtion_depth_info_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>   xtion_cloud_sub_;
    message_filters::Subscriber<sensor_msgs::Image>         camera_rgb_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>    camera_rgb_info_sub_;

    // camera info
    image_geometry::PinholeCameraModel xtion_rgb_model_;
    image_geometry::PinholeCameraModel xtion_depth_model_;
    image_geometry::PinholeCameraModel camera_rgb_model_;

    // variables
    boost::shared_ptr<message_filters::Synchronizer<sensor_sync_policy> > sensor_sync_;

    void sensor_callback( const sensor_msgs::ImageConstPtr & xtion_rgb_msg,
                          const sensor_msgs::CameraInfoConstPtr & xtion_rgb_info,
                          const sensor_msgs::ImageConstPtr & xtion_depth_msg,
                          const sensor_msgs::CameraInfoConstPtr & xtion_depth_info,
                          const sensor_msgs::PointCloud2ConstPtr & xtion_cloud2,
                          const sensor_msgs::ImageConstPtr & camera_rgb_msg,
                          const sensor_msgs::CameraInfoConstPtr & camera_rgb_info);

    void save_model( std::string filename, image_geometry::PinholeCameraModel model );

public:
    DataWriter( ros::NodeHandle & nh, std::string dir, bool use_cloud );

    ~DataWriter();
};



#endif
