#ifndef KD_RECOGNISER_H
#define KD_RECOGNISER_H

#include "include/kdes/libkdes.h"
#include "include/helpfun/utils.h"

#include <iostream>
#include <vector>

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

class KDRecogniser{
private:
    class SlidingWindowDetector{
    public:
        SlidingWindowDetector();

        SlidingWindowDetector( string model, string svm_model_file, string kdes_param_file, string models_folder, unsigned int model_type );

        vector<MatrixXf> process( cv::Mat image, vector<int> indices, int patch_size = 128, int step_size = 32 );

        vector<MatrixXf> process( IplImage * ipl_image, vector<int> indices, int patch_size = 128, int step_size = 32  );

        vector<string> * get_models_list();

        KernelDescManager * get_pkdm();

    private:
        KernelDescManager * pkdm_;
    };


private:
    // model filename
    string svm_model_name_;
    string kdes_model_name_;
    string model_folder_;
    string model_; // model name, should be the same as file name
    unsigned int model_type_;

    SlidingWindowDetector * swd_;

    // environment settings
    vector<string> target_bin_content_;
    string target_object_;
    int target_in_bin_;

    // pri-collect information
    cv::Mat empty_image_;
    cv::Mat empty_depth_;
    cv::Mat mask_image_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_cloud_;

    // input sensor information
    cv::Mat rgb_image_;
    cv::Mat depth_image_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    // sliding box settings
    int step_size_;
    int bbox_size_;

    KernelDescManager * kdes_;

    void find_blobs( const cv::Mat & binary, vector< vector<cv::Point2i> > & blobs );

    cv::Mat from_score( MatrixXf score, int scale );

public:
    KDRecogniser();

    void load_sensor_data(cv::Mat rgb_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    void load_sensor_data(cv::Mat rgb_image, cv::Mat depth_image);

    void load_sensor_data( cv::Mat rgb_image );

    void load_info( cv::Mat empty_image, cv::Mat mask_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_cloud );

    void load_info( cv::Mat empty_image, cv::Mat mask_image, cv::Mat empty_depth_image );

    void load_info( cv::Mat empty_image, cv::Mat mask_image );

    void set_env_configuration( string target_item, vector<string> items );

    void init_libkdes( string svm_model_name, string kdes_model_name, string model_folder, string model, unsigned int model_type );

    void process( vector<pair<string, vector<cv::Point> > > & results, bool use_rgb = true );
};




#endif
