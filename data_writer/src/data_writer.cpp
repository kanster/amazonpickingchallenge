/*
 * data_writer.cpp
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


#include "data_writer.h"

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

#include <vector>


const std::string g_xtion_rgb_win   = "xtion_rgb";
const std::string g_xtion_depth_win = "xtion_depth";
const std::string g_camera_rgb_win  = "camera_rgb";

/** Multiple point mouse callback
  * used to generate mask image
  */
class MultiMouseCallback{
public:
    struct PointsImage {
        std::vector< cv::Point > pts;
        cv::Mat image;
    };

    PointsImage points_image;
    std::string window_name;

public:
    MultiMouseCallback() {}

    MultiMouseCallback( cv::Mat image ) {
        window_name = "MouseCallbackMulti::callback()";
        points_image.image = image;
    }

    void release() {
        points_image.image.release();
        cv::destroyWindow( window_name );
        points_image.pts.clear();
    }

    static void on_mouse( int event, int x, int y, int flag, void * ptr ) {
        cv::Mat * image = &(( PointsImage * )ptr)->image;
        MultiMouseCallback mouse_cb( *image );
        if ( event == CV_EVENT_LBUTTONDOWN ) {
            cv::Point pt;
            pt.x = x;   pt.y = y;
            cv::circle( mouse_cb.points_image.image, pt, 5, cv::Scalar(0, 128, 255), 5 );
            (( PointsImage * )ptr)->pts.push_back( pt );
        }
    }

    void callback() {
        cv::namedWindow( window_name, CV_WINDOW_NORMAL );
        cv::setMouseCallback( window_name, on_mouse, &points_image );
        char key = 0;
        while ( key != 27 ) {
            cv::imshow( window_name, points_image.image );
            key = cv::waitKey( 10 );
        }
    }
};


/** constructor
  * @param node handle
  * @param directory to save the data
  */
DataWriter::DataWriter(ros::NodeHandle &nh, std::string dir) {
    nh_ = &nh;
    dir_= dir;
    count_ = 1;
    // start window for visualisation
    cv::namedWindow( g_xtion_rgb_win );     cv::moveWindow( g_xtion_rgb_win, 0, 0 );
    cv::namedWindow( g_xtion_depth_win );   cv::moveWindow( g_xtion_depth_win, 0, 520 );
    cv::namedWindow( g_camera_rgb_win );    cv::moveWindow( g_camera_rgb_win, 650, 0 );

    // resolve topic name
    xtion_rgb_topic_ = nh.resolveName( "/camera/rgb/image_color" );
    ROS_INFO( "subscribing to topic %s", xtion_rgb_topic_.c_str( ));
    xtion_rgb_info_topic_ = nh.resolveName("/camera/rgb/camera_info");
    ROS_INFO( "subscribing to topic %s", xtion_rgb_info_topic_.c_str( ));
    xtion_depth_topic_ = nh.resolveName("/camera/depth/image_raw");
    ROS_INFO( "subscribing to topic %s", xtion_depth_topic_.c_str() );
    xtion_depth_info_topic_ = nh.resolveName( "/camera/depth/camera_info" );
    ROS_INFO( "subscribing to topic %s", xtion_depth_info_topic_.c_str() );
    xtion_cloud_topic_ = nh.resolveName( "/camera/depth_registered/points" );
    ROS_INFO( "subscribing to topic %s", xtion_cloud_topic_.c_str());
    camera_rgb_topic_ = nh.resolveName( "/camera/image_color" );
    ROS_INFO( "subscribing to topic %s", camera_rgb_topic_.c_str());
    camera_rgb_info_topic_ = nh.resolveName( "/camera/camera_info" );
    ROS_INFO( "subscribing to topic %s", camera_rgb_info_topic_.c_str());

    // subscribe to sensors
    xtion_rgb_sub_.subscribe(nh, xtion_rgb_topic_, 1 );
    xtion_rgb_info_sub_.subscribe( nh, xtion_rgb_info_topic_, 1 );
    xtion_depth_sub_.subscribe( nh, xtion_depth_topic_, 1 );
    xtion_depth_info_sub_.subscribe( nh, xtion_depth_info_topic_, 1 );
    xtion_cloud_sub_.subscribe( nh, xtion_cloud_topic_, 1 );
    camera_rgb_sub_.subscribe( nh, camera_rgb_topic_, 1 );
    camera_rgb_info_sub_.subscribe( nh, camera_rgb_info_topic_, 1 );

    // syncroniser init
    sensor_sync_.reset( new message_filters::Synchronizer<sensor_sync_policy>(
                                  sensor_sync_policy(100),
                                  xtion_rgb_sub_,
                                  xtion_rgb_info_sub_,
                                  xtion_depth_sub_,
                                  xtion_depth_info_sub_,
                                  xtion_cloud_sub_,
                                  camera_rgb_sub_,
                                  camera_rgb_info_sub_) );
    sensor_sync_->registerCallback( boost::bind( &DataWriter::sensor_callback, this, _1, _2, _3, _4, _5, _6, _7 ) );
    ros::Rate loop(1);
    while( ros::ok() ) {
        ros::spinOnce();
        loop.sleep();
    }
}

/** destructor */
DataWriter::~DataWriter() {
}


/** sensor callback */
void DataWriter::sensor_callback(const sensor_msgs::ImageConstPtr &xtion_rgb_msg,
                                 const sensor_msgs::CameraInfoConstPtr &xtion_rgb_info,
                                 const sensor_msgs::ImageConstPtr &xtion_depth_msg,
                                 const sensor_msgs::CameraInfoConstPtr &xtion_depth_info,
                                 const sensor_msgs::PointCloud2ConstPtr &xtion_cloud2,
                                 const sensor_msgs::ImageConstPtr &camera_rgb_msg,
                                 const sensor_msgs::CameraInfoConstPtr &camera_rgb_info) {
    ROS_INFO_ONCE( "Sensor information available" );

    // xtion rgb msg ~ xtion_depth_msg will not be used
    // the saved image are generated from rgb point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(  new pcl::PointCloud<pcl::PointXYZRGB>() );
    pcl::fromROSMsg( *xtion_cloud2, *cloud );
    cv::Mat xtion_rgb( cloud->height, cloud->width, CV_8UC3 );
    cv::Mat xtion_depth( cloud->height, cloud->width, CV_16UC1, cv::Scalar::all(0) );
    for ( int y = 0; y < (int)cloud->height; ++ y ) {
        for ( int x = 0; x < (int)cloud->width; ++ x ) {
            pcl::PointXYZRGB & pt = cloud->points[y*cloud->width+x];
            xtion_rgb.at<cv::Vec3b>(y,x)[0] = pt.b;
            xtion_rgb.at<cv::Vec3b>(y,x)[1] = pt.g;
            xtion_rgb.at<cv::Vec3b>(y,x)[2] = pt.r;
            if ( !pcl_isinf(pt.x) && !pcl_isnan(pt.x) &&
                 !pcl_isinf(pt.y) && !pcl_isnan(pt.y) &&
                 !pcl_isinf(pt.z) && !pcl_isnan(pt.z) ) {
                xtion_depth.at<unsigned short>(y,x) = static_cast<unsigned short>(pt.z*1000.0);
            }
        }
    }

    // convert 16 bit depth image to 8 bit, for display only
    double min, max;
    cv::minMaxIdx( xtion_depth, &min, &max );
    cv::Mat vis_xtion_depth;    // image only for display
    cv::convertScaleAbs( xtion_depth, vis_xtion_depth, 255./max );

    cv::Mat camera_rgb = cv_bridge::toCvCopy( camera_rgb_msg, sensor_msgs::image_encodings::BGR8 )->image;

    // read camera info
    xtion_rgb_model_.fromCameraInfo( xtion_rgb_info );
    xtion_depth_model_.fromCameraInfo( xtion_depth_info );
    camera_rgb_model_.fromCameraInfo( camera_rgb_info );
    // rectify camera rgb image
    camera_rgb_model_.rectifyImage( camera_rgb, camera_rgb );

    cv::imshow( g_xtion_rgb_win, xtion_rgb );
    cv::imshow( g_xtion_depth_win, xtion_depth );
    cv::imshow( g_camera_rgb_win, camera_rgb );

    char key = cv::waitKey(5);
    if ( key == 'c' ) {
        // generate filenames
        std::string xtion_rgb_name = dir_ + "/xtion_rgb_" + boost::lexical_cast<std::string>(count_) + ".png";
        std::string xtion_depth_name = dir_ + "/xtion_depth_" + boost::lexical_cast<std::string>(count_) + ".png";  // for depth images, .png
        std::string xtion_cloud_name = dir_ + "/xtion_cloud_" + boost::lexical_cast<std::string>(count_) + ".pcd";
        std::string xtion_rgb_info_name = dir_ + "/xtion_rgb_info_" + boost::lexical_cast<std::string>(count_) + ".yml";
        std::string xtion_depth_info_name = dir_ + "/xtion_depth_info_" + boost::lexical_cast<std::string>(count_) + ".yml";
        std::string camera_rgb_name = dir_ + "/camera_rgb_" + boost::lexical_cast<std::string>(count_) + ".png";
        std::string camera_rgb_info_name = dir_ + "/camera_rgb_info_" + boost::lexical_cast<std::string>(count_) + ".yml";

        // write images
        cv::imwrite( xtion_rgb_name, xtion_rgb );
        cv::imwrite( xtion_depth_name, xtion_depth );
        cv::imwrite( camera_rgb_name, camera_rgb );

        // write point cloud
        pcl::io::savePCDFileASCII( xtion_cloud_name, *cloud);

        // save model
        save_model( xtion_rgb_info_name, xtion_rgb_model_ );
        save_model( xtion_depth_info_name, xtion_depth_model_ );
        save_model( camera_rgb_info_name, camera_rgb_model_ );

        // generate mask image
        cv::Mat xtion_rgb_mask( xtion_rgb.rows, xtion_rgb.cols, CV_8UC1, cv::Scalar::all(0) );
        MultiMouseCallback xtion_rgb_cb( xtion_rgb );
        xtion_rgb_cb.callback();
        cv::Point xtion_rgb_poly_pts[1][xtion_rgb_cb.points_image.pts.size()];
        for ( int i = 0; i < (int)xtion_rgb_cb.points_image.pts.size(); i ++ ) {
            xtion_rgb_poly_pts[0][i].x = xtion_rgb_cb.points_image.pts[i].x;
            xtion_rgb_poly_pts[0][i].y = xtion_rgb_cb.points_image.pts[i].y;
        }
        const cv::Point * xtion_rgb_st_pt[1] = { xtion_rgb_poly_pts[0] };
        int n_xtion_rgb_poly_pts[] = { xtion_rgb_cb.points_image.pts.size() };
        cv::fillPoly( xtion_rgb_mask, xtion_rgb_st_pt, n_xtion_rgb_poly_pts, 1, cv::Scalar::all(255), 8 );
        cv::imshow( "mask_image", xtion_rgb_mask );
        cv::waitKey(0);
        xtion_rgb_cb.release();

        cv::Mat xtion_depth_mask( xtion_depth.rows, xtion_depth.cols, CV_8UC1, cv::Scalar::all(0) );
        MultiMouseCallback xtion_depth_cb( xtion_depth );
        xtion_depth_cb.callback();
        cv::Point xtion_depth_poly_pts[1][xtion_depth_cb.points_image.pts.size()];
        for ( int i = 0; i < (int)xtion_depth_cb.points_image.pts.size(); i ++ ) {
            xtion_depth_poly_pts[0][i].x = xtion_depth_cb.points_image.pts[i].x;
            xtion_depth_poly_pts[0][i].y = xtion_depth_cb.points_image.pts[i].y;
        }
        const cv::Point * xtion_depth_st_pt[1] = { xtion_depth_poly_pts[0] };
        int n_xtion_depth_poly_pts[] = { xtion_depth_cb.points_image.pts.size() };
        cv::fillPoly( xtion_depth_mask, xtion_depth_st_pt, n_xtion_depth_poly_pts, 1, cv::Scalar::all(255), 8 );
        cv::imshow( "mask_image", xtion_depth_mask );
        cv::waitKey(0);
        xtion_depth_cb.release();



        cv::Mat camera_rgb_mask( camera_rgb.rows, camera_rgb.cols, CV_8UC1, cv::Scalar::all(0) );
        MultiMouseCallback camera_rgb_cb( camera_rgb );
        camera_rgb_cb.callback();
        cv::Point camera_rgb_poly_pts[1][camera_rgb_cb.points_image.pts.size()];
        for ( int i = 0; i < (int)camera_rgb_cb.points_image.pts.size(); i ++ ) {
            camera_rgb_poly_pts[0][i].x = camera_rgb_cb.points_image.pts[i].x;
            camera_rgb_poly_pts[0][i].y = camera_rgb_cb.points_image.pts[i].y;
        }
        const cv::Point * camera_rgb_st_pt[1] = { camera_rgb_poly_pts[0] };
        int n_camera_rgb_poly_pts[] = { camera_rgb_cb.points_image.pts.size() };
        cv::fillPoly( camera_rgb_mask, camera_rgb_st_pt, n_camera_rgb_poly_pts, 1, cv::Scalar::all(255), 8 );
        cv::imshow( "mask_image", camera_rgb_mask );
        cv::waitKey(0);
        camera_rgb_cb.release();

        // write mask image to disk
        std::string xtion_rgb_mask_name = dir_ + "/xtion_rgb_mask_" + boost::lexical_cast<std::string>(count_) + ".png";
        std::string xtion_depth_mask_name = dir_ + "/xtion_depth_mask_" + boost::lexical_cast<std::string>(count_) + ".png";
        std::string camera_rgb_mask_name = dir_ + "/camera_rgb_mask_" + boost::lexical_cast<std::string>(count_) + ".png";

        cv::imwrite( xtion_rgb_mask_name, xtion_rgb_mask );
        cv::imwrite( xtion_depth_mask_name, xtion_depth_mask );
        cv::imwrite( camera_rgb_mask_name, camera_rgb_mask );
        count_ ++;
    }
}


/** @todo camera model save to disk */
void DataWriter::save_model(std::string filename,
                           image_geometry::PinholeCameraModel model) {
    cv::FileStorage fs( filename, cv::FileStorage::WRITE );

    fs << "Projection" << model.projectionMatrix();
    fs << "Distortion" << model.distortionCoeffs();

    fs.release();
}






















