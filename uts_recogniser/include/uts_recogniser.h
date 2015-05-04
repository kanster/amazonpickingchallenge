#ifndef OFFLINE_RECOGNISER_H
#define OFFLINE_RECOGNISER_H

#include "include/helpfun/json_parser.hpp"
#include "include/helpfun/rgbd_recogniser.h"
#include "include/helpfun/rgb_recogniser.h"
#include "include/helpfun/kd_recogniser.h"
#include "include/helpfun/eblearn_recogniser.h"

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

#include "apc_msgs/TargetRequest.h"

// srv for ZJU block mode
#include "apc_msgs/RecogniseALG.h"

#include "apc_msgs/DataPublish.h"
#include "apc_msgs/RecogStatus.h"


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




class OfflineRecogniser{

private:
    // recognition method
    typedef enum{RGBD_RECOG, RGB_RECOG} RecogMethod;

    // sync policy of xtion and camera
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> sensor_sync_policy;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> no_cloud_sensor_sync_policy;

    // sensor data from xtion and camera
    struct SensorData {
        cv_bridge::CvImagePtr xtion_rgb_ptr;
        cv_bridge::CvImagePtr xtion_depth_ptr;
        // PointType is set to be PointXYZRGB in utils.h
        typename pcl::PointCloud<PointType>::Ptr xtion_cloud_ptr;

        // data from rgb image
        cv_bridge::CvImagePtr camera_rgb_ptr;

        // xtion availability
        bool use_cloud;
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
                          const sensor_msgs::ImageConstPtr & camera_image_msg,
                          const sensor_msgs::CameraInfoConstPtr & camera_image_info,
                          const sensor_msgs::PointCloud2ConstPtr & cloud_ptr_msg );

    // sensor callback without pcd file
    void sensor_callback_no_cloud(const sensor_msgs::ImageConstPtr &rgb_image_msg,
                         const sensor_msgs::CameraInfoConstPtr &rgb_image_info,
                         const sensor_msgs::ImageConstPtr &depth_image_msg,
                         const sensor_msgs::CameraInfoConstPtr &depth_image_info,
                         const sensor_msgs::ImageConstPtr &camera_image_msg,
                         const sensor_msgs::CameraInfoConstPtr &camera_image_info);

    // target service callback
    bool target_srv_callback( apc_msgs::TargetRequest::Request & req,
                              apc_msgs::TargetRequest::Response & resp);

    // block service callback
    bool target_srv_callback_block( apc_msgs::RecogniseALG::Request & req,
                                    apc_msgs::RecogniseALG::Response & resp);

    vector<pair<string, vector<cv::Point> > > kd_eb_opt( vector<pair<string, vector<cv::Point> > > kd_results,
                                                         vector<pair<string, vector<cv::Point> > > eb_results);

private:
    // recogniser main function of processing
    void process();

    // read method configuration file
    void load_method_config( string filename );

private:
    //! sync policy, with and without cloud
    boost::shared_ptr<message_filters::Synchronizer<sensor_sync_policy> > m_sensor_sync_;
    boost::shared_ptr<message_filters::Synchronizer<no_cloud_sensor_sync_policy> > m_no_cloud_sensor_sync_;

    //! camera info for 3 images
    image_geometry::PinholeCameraModel xtion_rgb_model_;
    image_geometry::PinholeCameraModel xtion_depth_model_;
    image_geometry::PinholeCameraModel camera_rgb_model_;

    //! recognition result publish
    ros::Publisher recog_pub_;

    //! call robotic platform to subscribe results
    ros::ServiceClient recog_client_;

    //! topics for sensor information
    string xtion_rgb_topic_;
    string xtion_rgb_info_topic_;
    string xtion_depth_topic_;
    string xtion_depth_info_topic_;
    string xtion_cloud_topic_;
    string camera_rgb_topic_;
    string camera_rgb_info_topic_;

    //! sensor info subscriber
    message_filters::Subscriber<sensor_msgs::Image>         xtion_rgb_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>    xtion_rgb_info_sub_;
    message_filters::Subscriber<sensor_msgs::Image>         xtion_depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>    xtion_depth_info_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>   xtion_cloud_sub_;
    message_filters::Subscriber<sensor_msgs::Image>         camera_rgb_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>    camera_rgb_info_sub_;

    //! buffer data, pointer and mutex
    SensorData sensor_data_;
    SensorData  *sensor_data_ptr_;
    bool        sensor_empty_;
    boost::mutex sensor_mutex_;
    boost::condition_variable sensor_cond_;

    //! mutex for service call from robotic
    bool        target_received_;
    bool        image_captured_;
    boost::mutex      srvc_mutex_;
    int         target_count_;  // id for the request

    /** recognition finish flag and mutex
      * recogniser_done, finish or not
      * recogniser_mutex, shared only with block-mode service callback
      * recogniser_cond, shared only with block-mode service callback
      */
    bool recogniser_done_;      // recogniser is finished
    boost::mutex recogniser_mutex_;
    boost::condition_variable recogniser_cond_;

    //! json configuration file path
    string json_filename_;

    //! topic and srv names
    string object_topic_name_;
    string object_srv_name_;
    string target_srv_name_;

    //! depth image or point cloud
    bool use_cloud_;
    //! mask image directory
    string mask_dir_;
    //! file path for method, rgb or rgbd
    string method_path_;
    map<string, RecogMethod> methods_; // 1 -> object name, 2 -> method

    //! target request
    string srv_bin_id_;
    string srv_object_name_;
    vector<string> srv_bin_contents_;
    int srv_object_index_;
    vector<int> srv_rm_object_indices_;

    //! eblearn recog and directory
    string eb_dir_;
    string temp_conf_path_;
    EBRecogniser ebr_;

    //! xml model dir for rgb and rgbd
    string xml_dir_;

    //! eblearn recog and directory
    string kd_dir_;
    KDRecogniser kdr_;


    //! main process thread
    boost::thread process_thread_;

    //! exit flag
    volatile bool exit_flag_;

    //! debug mode
    bool debug_;

    //! node handler
    ros::NodeHandle * nh_;


    /** operation mode and service mode */
    /** op_mode_ = 1, kernel descriptor only
      * op_mode_ = 2, eblearn descriptor only
      * op_mode_ = 3, kernel descriptor + eblearn
      * op_mode_ = 4, kernel descriptor + eblearn + optional rgb/rgbd
      */
    int op_mode_;

    /** srv_mode_ = 1, non-block mode
      * srv_mode_ = 2, block mode
      */
    int srv_mode_;

    /** published topics */
    apc_msgs::BinObjects bin_objs_;
};


#endif // OFFLINE_RECOGNISER_H
