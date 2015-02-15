#ifndef MASK_GENERATOR_H
#define MASK_GENERATOR_H

#include "include/utils.h"

class MaskGenerator{
private:
    bool depth_only_;
    int  target_bin_;
public:
    MaskGenerator( bool depth_only, int target_bin = -1 );
    cv::Mat process( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr );
};


#endif
