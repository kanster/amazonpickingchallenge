/*
 * data_publisher.cpp
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

#include "data_publisher.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>

const std::string g_target_srv_name  = "/data_publish_srv";
const std::string g_recog_srv_name = "/recog_publish_srv";
const std::string g_obj_topic_name = "/object_poses";


DataPublisher::DataPublisher(ros::NodeHandle &nh, std::string dir, int n):
    recog_completed_(true){
    nh_ = & nh;
    dir_ = dir;
    n_frames_ = n;
    recog_server_ = nh.advertiseService( g_recog_srv_name, &DataPublisher::recog_srv_callback, this);
    obj_sub_ = nh.subscribe( g_obj_topic_name, 1, &DataPublisher::recog_callback, this );


    client_ = nh.serviceClient<apc_msgs::DataPublish>(g_target_srv_name);

    std::string xtion_rgb_topic = "/xtion/rgb/image";
    std::string xtion_rgb_info_topic = "/xtion/rgb/camera_info";
    std::string xtion_depth_topic = "/xtion/depth/image";
    std::string xtion_depth_info_topic = "/xtion/depth/camera_info";
    std::string xtion_cloud_topic = "/xtion/depth/points";
    std::string camera_rgb_topic = "/camera/image";
    std::string camera_rgb_info_topic = "/camera/camera_info";

    xtion_rgb_pub_ = nh.advertise<sensor_msgs::Image>(xtion_rgb_topic, 1);
    xtion_rgb_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(xtion_rgb_info_topic, 1);
    xtion_depth_pub_ = nh.advertise<sensor_msgs::Image>(xtion_depth_topic, 1);
    xtion_depth_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(xtion_depth_info_topic, 1);
    xtion_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(xtion_cloud_topic, 1);
    camera_rgb_pub_ = nh.advertise<sensor_msgs::Image>(camera_rgb_topic, 1);
    camera_rgb_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(camera_rgb_info_topic, 1);

    publish_thread_ = boost::thread(boost::bind(&DataPublisher::info_publisher, this));
    ros::MultiThreadedSpinner spinner(4);
    ros::Rate loop(1);
    while ( ros::ok() ) {
        spinner.spin();
        loop.sleep();
    }
}


void DataPublisher::recog_callback(const apc_msgs::BinObjectsConstPtr &objs_msg) {
    std::cout << "Results from bin " << objs_msg->bin_id << "\n";
    for ( int i = 0; i < (int)objs_msg->items_in_bin.size(); ++ i ) {
        apc_msgs::Object obj = objs_msg->items_in_bin[i];
        std::cout << "    " << obj.name << ": position: [" << obj.pose.position.x << ", " << obj.pose.position.y << ", " << obj.pose.position.z << "]" << " and orientation [" << obj.pose.orientation.w << ", " << obj.pose.orientation.x << ", " << obj.pose.orientation.y << ", " << obj.pose.orientation.z  << "]\n";
    }
}


bool DataPublisher::recog_srv_callback(apc_msgs::RecogStatus::Request &req,
                                       apc_msgs::RecogStatus::Response &resp) {
    ROS_INFO_ONCE("[recog_srv_callback] recog compeletion request received");
    if ( req.recog == true )
        ROS_INFO( "Object recognition successed" );
    {
        boost::mutex::scoped_lock lock( recog_completed_mutex_ );
        recog_completed_ = true;
    }
    recog_completed_cond_.notify_one();

    resp.pub = true;
    return true;
}


void DataPublisher::info_publisher() {
    while (true) {


        apc_msgs::DataPublish data_srv;

        cv_bridge::CvImage xtion_rgb_cv;
        cv_bridge::CvImage xtion_depth_cv;
        cv_bridge::CvImage camera_rgb_cv;
        count_ = 1;
        while( nh_->ok() ) {
            // sending request iteratively
            for ( count_ = 1; count_ <= n_frames_; ++ count_ ) {
                // generate filenames
                std::string xtion_rgb_name = dir_ + "/xtion_rgb_" + boost::lexical_cast<std::string>(count_) + ".png";
                std::string xtion_depth_name = dir_ + "/xtion_depth_" + boost::lexical_cast<std::string>(count_) + ".png";  // for depth images, .png
                std::string xtion_cloud_name = dir_ + "/xtion_cloud_" + boost::lexical_cast<std::string>(count_) + ".pcd";
                std::string xtion_rgb_info_name = dir_ + "/xtion_rgb_info_" + boost::lexical_cast<std::string>(count_) + ".yml";
                std::string xtion_depth_info_name = dir_ + "/xtion_depth_info_" + boost::lexical_cast<std::string>(count_) + ".yml";
                std::string camera_rgb_name = dir_ + "/camera_rgb_" + boost::lexical_cast<std::string>(count_) + ".png";
                std::string camera_rgb_info_name = dir_ + "/camera_rgb_info_" + boost::lexical_cast<std::string>(count_) + ".yml";

                // publish xtion rgb
                xtion_rgb_cv.image = cv::imread( xtion_rgb_name, CV_LOAD_IMAGE_COLOR );
                xtion_rgb_cv.encoding = sensor_msgs::image_encodings::BGR8;
                xtion_rgb_cv.toImageMsg( xtion_rgb_msg_ );

                // publish xtion depth
    //            cv::Mat depth_image_in( xtion_rgb_cv.image.rows, xtion_rgb_cv.image.cols, CV_32FC1, cv::Scalar::all(0) );
    //            depth_image_in = cv::imread( xtion_depth_name, CV_LOAD_IMAGE_ANYDEPTH );
    //            cv::imshow( "depth_image", depth_image_in );
    //            cv::waitKey(5);
                xtion_depth_cv.image  = cv::imread( xtion_depth_name, CV_LOAD_IMAGE_ANYDEPTH );
                xtion_depth_cv.encoding = sensor_msgs::image_encodings::MONO16;
                xtion_depth_cv.toImageMsg( xtion_depth_msg_ );

                // publish point cloud message
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
                pcl::io::loadPCDFile( xtion_cloud_name, *cloud );
                pcl::toROSMsg( *cloud, xtion_cloud_msg_ );


                // publish camera rgb
                camera_rgb_cv.image = cv::imread( camera_rgb_name, CV_LOAD_IMAGE_COLOR );
                camera_rgb_cv.encoding = sensor_msgs::image_encodings::BGR8;
                camera_rgb_cv.toImageMsg( camera_rgb_msg_ );

                int idx = 0;
                // camera info
                cv::FileStorage xtion_rgb_fs(xtion_rgb_info_name, cv::FileStorage::READ);
                cv::Mat xtion_rgb_proj, xtion_rgb_dist;
                xtion_rgb_fs["Projection"] >> xtion_rgb_proj;
                xtion_rgb_fs["Distortion"] >> xtion_rgb_dist;
                boost::array<double, 12> xtion_rgb_boost_p;
                for ( int y = 0; y < xtion_rgb_proj.rows; ++ y ) {
                    for ( int x = 0; x < xtion_rgb_proj.cols; ++ x ) {
                        xtion_rgb_boost_p[idx] = xtion_rgb_proj.at<double>(y,x);
                        idx ++;
                    }
                }
                idx = 0;
                std::vector<double> xtion_rgb_boost_d;
                for ( int y = 0; y < xtion_rgb_dist.rows; ++ y )
                    for ( int x = 0; x < xtion_rgb_dist.cols; ++ x )
                        xtion_rgb_boost_d.push_back( xtion_rgb_dist.at<double>(y,x) );
                xtion_rgb_info_msg_.height = xtion_rgb_cv.image.rows;
                xtion_rgb_info_msg_.width  = xtion_rgb_cv.image.cols;
                xtion_rgb_info_msg_.P = xtion_rgb_boost_p;
                xtion_rgb_info_msg_.D = xtion_rgb_boost_d;

                cv::FileStorage xtion_depth_fs(xtion_depth_info_name, cv::FileStorage::READ);
                cv::Mat xtion_depth_proj, xtion_depth_dist;
                xtion_depth_fs["Projection"] >> xtion_depth_proj;
                xtion_depth_fs["Distortion"] >> xtion_depth_dist;
                boost::array<double, 12> xtion_depth_boost_p;
                for ( int y = 0; y < xtion_depth_proj.rows; ++ y ) {
                    for ( int x = 0; x < xtion_depth_proj.cols; ++ x ) {
                        xtion_depth_boost_p[idx] = y*xtion_depth_proj.cols+x;
                        idx ++;
                    }
                }
                idx = 0;
                std::vector<double> xtion_depth_boost_d;
                for ( int y = 0; y < xtion_depth_dist.rows; ++ y )
                    for ( int x = 0; x < xtion_depth_dist.cols; ++ x )
                        xtion_depth_boost_d.push_back( xtion_depth_dist.at<double>(y,x) );
                idx = 0;
                xtion_depth_info_msg_.height = xtion_depth_cv.image.rows;
                xtion_depth_info_msg_.width  = xtion_depth_cv.image.cols;
                xtion_depth_info_msg_.P = xtion_depth_boost_p;
                xtion_depth_info_msg_.D = xtion_depth_boost_d;


                cv::FileStorage camera_rgb_fs(camera_rgb_info_name, cv::FileStorage::READ);
                cv::Mat camera_rgb_proj, camera_rgb_dist;
                camera_rgb_fs["Projection"] >> camera_rgb_proj;
                camera_rgb_fs["Distortion"] >> camera_rgb_dist;
                boost::array<double, 12> camera_rgb_boost_p;
                for ( int y = 0; y < camera_rgb_proj.rows; ++ y ) {
                    for ( int x = 0; x < camera_rgb_proj.cols; ++ x ) {
                        camera_rgb_boost_p[idx] = camera_rgb_proj.at<double>(y,x);
                        idx ++;
                    }
                }
                idx = 0;
                std::vector<double> camera_rgb_boost_d;
                for ( int y = 0; y < camera_rgb_dist.rows; ++ y )
                    for ( int x = 0; x < camera_rgb_dist.cols; ++ x )
                        camera_rgb_boost_d.push_back( camera_rgb_dist.at<double>(y,x) );
                camera_rgb_info_msg_.height = camera_rgb_cv.image.rows;
                camera_rgb_info_msg_.width  = camera_rgb_cv.image.cols;
                camera_rgb_info_msg_.P = camera_rgb_boost_p;
                camera_rgb_info_msg_.D = camera_rgb_boost_d;


                // wait for recogniser done before sending request
                {
                    boost::mutex::scoped_lock lock(recog_completed_mutex_);
                    while ( !recog_completed_ ) {
                        recog_completed_cond_.wait(lock);
                    }
                }


                data_srv.request.DataIndex = count_;
                ROS_INFO( "Publish data frame %d and send request ...", count_ );
                if( client_.call(data_srv) ) {
                    ROS_INFO( "return status: %s", data_srv.response.Found? "true" : "false" );
                }
                else {
                    ROS_ERROR( "Failed to call service %s", g_target_srv_name.c_str() );
                }


                ros::Rate rate(10);
                xtion_rgb_pub_.publish( xtion_rgb_msg_ );
                xtion_rgb_info_pub_.publish( xtion_rgb_info_msg_ );

                xtion_depth_pub_.publish( xtion_depth_msg_ );
                xtion_depth_info_pub_.publish( xtion_depth_info_msg_ );

                xtion_cloud_pub_.publish( xtion_cloud_msg_ );

                camera_rgb_pub_.publish( camera_rgb_msg_ );
                camera_rgb_info_pub_.publish( camera_rgb_info_msg_ );

                // after publish set recogniser flag to false
                recog_completed_mutex_.lock();
                recog_completed_ = false;
                recog_completed_mutex_.unlock();
            }
        }
    }

}


DataPublisher::~DataPublisher(){
}

















