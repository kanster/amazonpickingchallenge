#ifndef OFFLINE_RECOGNISER_H
#define OFFLINE_RECOGNISER_H

#include "include/json_parser.hpp"
#include "include/rgbd_recogniser.h"
#include "include/rgb_recogniser.h"

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include "apc_msgs/ObjectPose.h"
#include "apc_msgs/ObjectPoseList.h"

#include "apc_msgs/Object.h"
#include "apc_msgs/BinObjects.h"
#include "apc_msgs/RowBinObjects.h"

#include "apc_msgs/DataPublish.h"

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

#define WINDOW_NAME "objwindow"

class OfflineRecogniser{

private:
    // recognition method
    typedef enum{RGBD_RECOG, RGB_RECOG} RecogMethod;

    // sync policy of xtion and camera
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image
    , sensor_msgs::CameraInfo
    , sensor_msgs::Image
    , sensor_msgs::CameraInfo
    , sensor_msgs::PointCloud2
    , sensor_msgs::Image
    , sensor_msgs::CameraInfo
    > sensor_sync_policy;

    // sensor data from xtion and camera
    struct SensorData {
        // data from xtion
        cv_bridge::CvImagePtr xtion_rgb_ptr;
        cv_bridge::CvImagePtr xtion_depth_ptr;
        // PointType is set to be PointXYZRGB in utils.h
        typename pcl::PointCloud<PointType>::Ptr xtion_cloud_ptr;

        // data from rgb image
        cv_bridge::CvImagePtr camera_rgb_ptr;

        // xtion availability
        bool xtion_info;
    };

    // target item from request
    struct TargetItem {
        int  target_index;
        string  target_name;
        vector<int> removed_items_indices;
    };

    // methods for all working order item
    struct Item{
        string object_name;
        string method;
    };


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // constructor and destructor
    OfflineRecogniser( ros::NodeHandle & nh );
    ~OfflineRecogniser();

    // main processing function
    void start_monitor( void );

    // sensor callback
    void sensor_callback( const sensor_msgs::ImageConstPtr & rgb_image_msg,
                          const sensor_msgs::CameraInfoConstPtr & rgb_image_info,
                          const sensor_msgs::ImageConstPtr & depth_image_msg,
                          const sensor_msgs::CameraInfoConstPtr & depth_image_info,
                          const sensor_msgs::PointCloud2ConstPtr & cloud_ptr_msg,
                          const sensor_msgs::ImageConstPtr & camera_image_msg,
                          const sensor_msgs::CameraInfoConstPtr & camera_image_info );

    // target service callback
    bool target_srv_callback( apc_msgs::DataPublish::Request & req,
                              apc_msgs::DataPublish::Response & resp);


private:

    // function
    // recogniser main function of processing
    void process();

    // read method configuration file
    void load_method_config( string filename );

private:
    // variables
    boost::shared_ptr<message_filters::Synchronizer<sensor_sync_policy> > m_sensor_sync_;

    // camera info
    image_geometry::PinholeCameraModel xtion_rgb_model_;
    image_geometry::PinholeCameraModel xtion_depth_model_;
    image_geometry::PinholeCameraModel camera_rgb_model_;

    // publisher and subscriber
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher      image_pub_;         // recognition result, areas

    ros::Publisher recog_pub_;                  // recognition result publisher

    string xtion_rgb_topic_;
    string xtion_rgb_info_topic_;
    string xtion_depth_topic_;
    string xtion_depth_info_topic_;
    string xtion_cloud_topic_;
    string camera_rgb_topic_;
    string camera_rgb_info_topic_;

    message_filters::Subscriber<sensor_msgs::Image>         xtion_rgb_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>    xtion_rgb_info_sub_;
    message_filters::Subscriber<sensor_msgs::Image>         xtion_depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>    xtion_depth_info_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>   xtion_cloud_sub_;
    message_filters::Subscriber<sensor_msgs::Image>         camera_rgb_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>    camera_rgb_info_sub_;

    // buffer data and mutex
    SensorData sensor_data_;

    SensorData  *sensor_data_ptr_;
    SensorData dBuf[2];
    SensorData  *imshow_data_ptr_;
    unsigned int cindex;

    bool        sensor_empty_;
    boost::mutex sensor_mutex_;
    boost::condition_variable sensor_cond_;

    // unique lock
    TargetItem  target_item_;
    bool        target_received_;
    bool        image_captured_;
    boost::mutex      srvc_mutex_;
    boost::condition_variable srvc_cond_;

    int         target_count_;  // id for the request

    bool recogniser_done_;      // recogniser is finished
    bool recogniser_success_;   // recogniser is success


    // configuration file
    // json configuration input
    string json_filename_;
    map< string, vector<string> > bin_contents_;
    vector< pair<string, string> > work_order_;

    // target item index
    int data_index_;


    // methods configuration file
    map<string, string> methods_; // 1 -> object name, 2 -> method

    // object name and methods
    vector<Item> items_;

    // Recognition method
    string reco_method_;

    boost::thread process_thread_;

    volatile bool exit_flag_;

    bool debug_;
    // node handler
    ros::NodeHandle * nh_;
};


#endif // OFFLINE_RECOGNISER_H
