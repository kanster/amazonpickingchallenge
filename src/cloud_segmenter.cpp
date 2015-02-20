#include "include/cloud_segmenter.h"
#include <pcl/surface/on_nurbs/sequential_fitter.h>
#include "v4r/SegmenterLight/SegmenterLight.h"

CloudSegmenter::CloudSegmenter(bool fast, int detail_level, string model_dir ) {
    fast_ = fast;
    detail_level_ = detail_level;
    models_dir_ = model_dir;
}

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr CloudSegmenter::process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, vector<DetectedFeatureRGBD> & features) {
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr ori_labeled_cloud( new pcl::PointCloud<pcl::PointXYZRGBL> );
    segment::SegmenterLight seg(models_dir_);
    seg.setFast( fast_ );
    seg.setDetail( detail_level_ );
    ori_labeled_cloud = seg.processPointCloud(cloud);

    // obtain the group numbers
    int max_label = 0;
    for ( size_t i = 0; i < ori_labeled_cloud->points.size(); ++ i )
        if ( (int)ori_labeled_cloud->points[i].label > max_label )
            max_label = ori_labeled_cloud->points[i].label;

    // find how many features in a given group
    vector<int> labels_counter(max_label+1, 0);
    vector<int> useless_labels; // the labels do not have enough number of features
    vector<int> useful_labels;  // the labels contain enough number of features
    for ( size_t i = 0; i < features.size(); ++ i ) {
        int idx = (int)(features[i].img2d(1))*640+(int)(features[i].img2d(0));
        labels_counter[ori_labeled_cloud->points[idx].label] ++;
    }
    for ( size_t i = 0; i < labels_counter.size(); ++ i ) {
        if ( labels_counter[i] <= 10 )  // if the feature number is less than 10, set it as useless
            useless_labels.push_back( i );
        else
            useful_labels.push_back(i);
    }

    // generate new labeled point cloud
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr labeled_cloud( new pcl::PointCloud<pcl::PointXYZRGBL> );
    for ( size_t i = 0; i < ori_labeled_cloud->points.size(); ++ i ) {
        const pcl::PointXYZRGBL & pt = ori_labeled_cloud->points[i];
        pcl::PointXYZRGBL npt = pt;
        if ( find( useless_labels.begin(), useless_labels.end(), pt.label ) != useless_labels.end() )
            npt.label = 0;
        else {
            int pos = find(useful_labels.begin(), useful_labels.end(), pt.label)-useful_labels.begin()+1;
            npt.label = pos;
        }
        labeled_cloud->points.push_back( npt );
    }
    labeled_cloud->width = 640;
    labeled_cloud->height = 480;

    for ( size_t i = 0; i < features.size(); ++ i ) {
        int idx = (int)(features[i].img2d(1))*640+(int)(features[i].img2d(0));
        features[i].groupidx = labeled_cloud->points[idx].label;
    }

    labels_counter.clear();
    useless_labels.clear();
    useful_labels.clear();

    return labeled_cloud;
}

