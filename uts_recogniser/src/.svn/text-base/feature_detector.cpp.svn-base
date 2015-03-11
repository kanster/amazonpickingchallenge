#include "include/feature_detector.h"

// constructor
FeatureDetector::FeatureDetector(string method) {
    method_ = method;
}

// process rgb features
vector< DetectedFeatureRGB > FeatureDetector::process( const cv::Mat & image, cv::Mat mask_image) {
    vector<DetectedFeatureRGB> detected_features;
    cv::Mat mono_image;
    cv::cvtColor( image, mono_image, CV_RGB2GRAY );
    Image sift_image = CreateImage( image.rows, image.cols );
    for ( int y = 0; y < mono_image.rows; ++ y )
        for ( int x = 0; x < mono_image.cols; ++ x )
            sift_image->pixels[y*mono_image.cols+x] = (float)mono_image.at<uchar>(y,x)/255.0;
    Keypoint keypts = GetKeypoints( sift_image );
    Keypoint keypt  = keypts;
    while ( keypt ) {
        if ( mask_image.at<uchar>( keypt->row, keypt->col ) > 128 ) {
            detected_features.resize( detected_features.size()+1 );
            detected_features.back().descrip.resize(128);
            for ( int i = 0; i < 128; ++ i )
                detected_features.back().descrip[i] = keypt->descrip[i];
            detected_features.back().img2d(0) = keypt->col;
            detected_features.back().img2d(1) = keypt->row;
        }
        keypt = keypt->next;
    }

    FreeKeypoints( keypts );
    DestroyAllImages();

    mono_image.release();
    mask_image.release();

    return detected_features;
}

vector<DetectedFeatureRGBD> FeatureDetector::process(const cv::Mat &image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat mask_image ) {
    vector<DetectedFeatureRGBD> detected_features;
    cv::Mat mono_image;
    cv::cvtColor( image, mono_image, CV_RGB2GRAY );

    Image sift_image = CreateImage( image.rows, image.cols );
    for ( int y = 0; y < mono_image.rows; ++ y )
        for ( int x = 0; x < mono_image.cols; ++ x )
            sift_image->pixels[y*mono_image.cols+x] = (float)mono_image.at<uchar>(y,x)/255.0;
    Keypoint keypts = GetKeypoints( sift_image );
    Keypoint keypt  = keypts;
    map<pair<float, float>, int> existed;
    int count = 0;
    while (keypt) {
        int y = keypt->row, x = keypt->col;
        if ( existed.find( make_pair(keypt->col, keypt->row)) == existed.end() && mask_image.at<uchar>(y,x) > 128 ) {
            detected_features.resize( detected_features.size()+1 );
            detected_features.back().descrip.resize(128);
            for ( int i = 0; i < 128; ++ i )
                detected_features.back().descrip[i] = keypt->descrip[i];
            detected_features.back().img2d(0) = keypt->col;
            detected_features.back().img2d(1) = keypt->row;
            detected_features.back().obs3d(0) = cloud->points[y*cloud->width+x].x*100.0;
            detected_features.back().obs3d(1) = cloud->points[y*cloud->width+x].y*100.0;
            detected_features.back().obs3d(2) = cloud->points[y*cloud->width+x].z*100.0;
            existed[ make_pair(keypt->col, keypt->row) ] = count;
            count ++;
        }
        keypt = keypt->next;
    }
    FreeKeypoints( keypts );
    DestroyAllImages();

    mono_image.release();
    mask_image.release();

    return detected_features;
}
