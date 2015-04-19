/*
 * system_calibrator.cpp
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

#include "include/helpfun/system_calibrator.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>

#include <boost/bind.hpp>

const std::string g_rgb_win_name = "rgb_image";
const std::string g_depth_win_name = "depth_image";

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


SystemCalib::SystemCalib( ros::NodeHandle & nh, std::string dir ) {
    nh_  = &nh;
    dir_ = dir;
    cv::namedWindow( g_rgb_win_name );
    cv::moveWindow( g_rgb_win_name, 0, 0 );
    cv::namedWindow( g_depth_win_name );
    cv::moveWindow( g_depth_win_name, 0, 500);

    cloud_topic_ = nh.resolveName( "/camera/depth_registered/points" );
    cloud_sub_   = nh.subscribe( cloud_topic_, 1,  &SystemCalib::cloud_callback, this );
    ros::Rate loop(1);
    while ( ros::ok() ) {
        ros::spinOnce();
        loop.sleep();
    }
}


SystemCalib::~SystemCalib() {
}

void SystemCalib::cloud_callback(const sensor_msgs::PointCloud2ConstPtr cloud_msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
    pcl::fromROSMsg( *cloud_msg, *cloud );
    cv::Mat rgb_image( cloud->height, cloud->width, CV_8UC3 );
    cv::Mat depth_image( cloud->height, cloud->width, CV_16UC1, cv::Scalar::all(0) );
    for ( int y = 0; y < (int)cloud->height; ++ y ) {
        for ( int x = 0; x < (int)cloud->width; ++ x ) {
            pcl::PointXYZRGB & pt = cloud->points[y*cloud->width+x];
            rgb_image.at<cv::Vec3b>(y,x)[0] = pt.b;
            rgb_image.at<cv::Vec3b>(y,x)[1] = pt.g;
            rgb_image.at<cv::Vec3b>(y,x)[2] = pt.r;
            if ( !pcl_isinf(pt.x) && !pcl_isnan(pt.x) &&
                 !pcl_isinf(pt.y) && !pcl_isnan(pt.y) &&
                 !pcl_isinf(pt.z) && !pcl_isnan(pt.z) ) {
                depth_image.at<unsigned short>(y,x) = static_cast<unsigned short>(pt.z*1000.0);
            }
        }
    }
    double min, max;
    cv::minMaxIdx( depth_image, &min, &max );
    cv::convertScaleAbs( depth_image, depth_image, 255/max );

    cv::imshow( g_rgb_win_name, rgb_image );
    cv::imshow( g_depth_win_name, depth_image );

    char key = cv::waitKey(10);
    if ( key = 'c' ) {
        std::string filename;
        std::cout << "Please input bin id and this will\n" <<
                     "also used as the filename of the image:\n";
        std::cin >> filename;
        // draw mask image
        cv::Mat rgb_mask_image( rgb_image.rows, rgb_image.cols, CV_8UC1, cv::Scalar::all(0) );
        MultiMouseCallback mouse_cb_rgb( rgb_image );
        mouse_cb_rgb.callback();
        cv::Point rgb_poly_pts[1][mouse_cb_rgb.points_image.pts.size()];
        for ( int i = 0; i < mouse_cb_rgb.points_image.pts.size(); i ++ ) {
            rgb_poly_pts[0][i].x = mouse_cb_rgb.points_image.pts[i].x;
            rgb_poly_pts[0][i].y = mouse_cb_rgb.points_image.pts[i].y;
        }
        const cv::Point * st_rgb_poly_pts[1] = { rgb_poly_pts[0] };
        int n_rgb_poly_pts[] = { mouse_cb_rgb.points_image.pts.size() };
        cv::fillPoly( rgb_mask_image, st_rgb_poly_pts, n_rgb_poly_pts, 1, cv::Scalar::all(255), 8 );
        cv::imshow( "mask_image", rgb_mask_image );
        cv::imwrite( std::string( dir_+"/"+filename+"_rgb.jpg"), rgb_mask_image );

        cv::Mat depth_mask_image( depth_image.rows, depth_image.cols, CV_8UC1, cv::Scalar::all(0) );
        MultiMouseCallback mouse_cb_depth( depth_image );
        mouse_cb_depth.callback();
        cv::Point poly_pts[1][mouse_cb_depth.points_image.pts.size()];
        for ( int i = 0; i < mouse_cb_depth.points_image.pts.size(); i ++ ) {
            poly_pts[0][i].x = mouse_cb_depth.points_image.pts[i].x;
            poly_pts[0][i].y = mouse_cb_depth.points_image.pts[i].y;
        }
        const cv::Point * st_poly_pts[1] = { poly_pts[0] };
        int n_poly_pts[] = { mouse_cb_depth.points_image.pts.size() };
        cv::fillPoly( depth_mask_image, st_poly_pts, n_poly_pts, 1, cv::Scalar::all(255), 8 );
        cv::imshow( "mask_image", depth_mask_image );
        cv::imwrite( std::string( dir_+"/"+filename+"_rgb.jpg"), depth_mask_image );

        cv::destroyWindow( "mask_image" );


    }
    rgb_image.release();
    depth_image.release();
}





void print_usage( char* prog_name ) {
    std::cout   << "\n\nUsage: " << prog_name << " [options]\n\n"
                << "Options:\n"
                << "-------------------------------------------\n"
                << "-h          this help\n"
                << "-dir        directory\n";
}


int main( int argc, char ** argv ) {
    std::string dir;
    if ( pcl::console::parse_argument( argc, argv, "-dir", dir ) != -1 ) {
        ros::init( argc, argv, "calib" );
        ros::NodeHandle nh("~");
        SystemCalib calib( nh, dir );
        return 1;
    }
    else {
        print_usage(argv[0]);
        return 0;
    }
}


