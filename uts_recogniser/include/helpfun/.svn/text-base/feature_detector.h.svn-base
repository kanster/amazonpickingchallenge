#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include "include/helpfun/utils.h"
#include "siftfast/siftfast.h"

class FeatureDetector{
private:
    string method_;

public:
    // constructor with parametrisation
    FeatureDetector( string method );

    // process rgb features
    vector< DetectedFeatureRGB > process( const cv::Mat & image, cv::Mat mask_image = cv::Mat( 480, 640, CV_8UC1, cv::Scalar::all(255) ) );

    // process rgbd features
    vector< DetectedFeatureRGBD > process( const cv::Mat & image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat mask_image = cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(255)) );
};

#endif
