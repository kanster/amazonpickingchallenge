#ifndef RGBD_RECOGNISER_H
#define RGBD_RECOGNISER_H

#include "include/utils.h"

#include "include/mask_generator.h"
#include "include/feature_detector.h"
#include "include/feature_matcher.h"
#include "include/svd_pose_estimator.h"
#include "include/feature_cluster.h"

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <dirent.h>
#include <stdlib.h>
#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <cv_bridge/cv_bridge.h>

#include "srv_gen/cpp/include/uts_recogniser/Enable.h"
#include "msg_gen/cpp/include/uts_recogniser/ObjectPose.h"
#include "msg_gen/cpp/include/uts_recogniser/ObjectPoseList.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>


using namespace std;


class RGBDRecogniser{
private:
    string models_dir_;
    string seg_model_dir_;

    cv::Mat rgb_image_;
    cv::Mat depth_image_;
    cv::Mat mask_image_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    int target_idx_;

    Vector4f params_;

    vector<string> target_bin_content_;
    string target_object_;
    int target_in_bin_;

    vector<SP_Model> models_;

    vector<DetectedFeatureRGBD> detected_features_;

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr labeled_cloud_;

    typedef vector<MatchRGBD> Matches;
    Matches matches_;
    vector< list<int> > clusters_;

    list<SP_Object> objects_;
private:
    void filter_objects();


public:
    /** constructor */
    RGBDRecogniser( cv::Mat rgb_image, cv::Mat depth_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string seg_model_dir );

    void load_models( string models_dir );

    /** set target item related variables */
    void set_env_configuration( int idx, vector<pair<string, string> > work_order, map<string, vector<string> > bin_contents );

    /** set camera parameters */
    void set_camera_params( float fx, float fy, float cx, float cy );

    void run( bool visualise );

};

#endif // RGBD_RECOGNISER_H
