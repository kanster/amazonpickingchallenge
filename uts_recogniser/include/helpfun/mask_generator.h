#ifndef MASK_GENERATOR_H
#define MASK_GENERATOR_H

#include "include/helpfun/utils.h"

class MaskGenerator{
public:
    MaskGenerator();
    cv::Mat process( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr );

    cv::Mat process( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, cv::Mat mask_image );
};


#endif
