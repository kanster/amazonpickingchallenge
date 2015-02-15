#ifndef CLOUD_SEGMENTER_H
#define CLOUD_SEGMENTER_H

#include "include/utils.h"

class CloudSegmenter{
private:
    bool fast_;
    int  detail_level_;
    string models_dir_;

public:
    CloudSegmenter( bool fast, int detail_level, string models_dir );

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr process( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, vector<DetectedFeatureRGBD> & features );
};

#endif
