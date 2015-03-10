#include "include/mask_generator.h"

MaskGenerator::MaskGenerator() {
}

cv::Mat MaskGenerator::process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr) {
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                         cv::Size( 2*2 + 1, 2*2+1 ),
                                         cv::Point( 2, 2 ) );
    cv::Mat mask_image( cloud_ptr->height, cloud_ptr->width, CV_8UC1, cv::Scalar::all(0) );
    for ( int y = 0; y < (int)cloud_ptr->height; y ++ )
        for ( int x = 0; x < (int)cloud_ptr->width; x ++ )
            if ( cloud_ptr->points[y*cloud_ptr->width+x].z > 0.01 )
                mask_image.at<uchar>(y, x) = 255;
    cv::erode( mask_image, mask_image, element );
    element.release();
    return mask_image;
}

cv::Mat MaskGenerator::process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, cv::Mat mask_image) {
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                         cv::Size( 2*2 + 1, 2*2+1 ),
                                         cv::Point( 2, 2 ) );
    cv::Mat depth_mask_image( cloud_ptr->height, cloud_ptr->width, CV_8UC1, cv::Scalar::all(0) );
    for ( int y = 0; y < (int)cloud_ptr->height; y ++ )
        for ( int x = 0; x < (int)cloud_ptr->width; x ++ )
            if ( cloud_ptr->points[y*cloud_ptr->width+x].z > 0.01 )
                depth_mask_image.at<uchar>(y, x) = 255;
    cv::erode( depth_mask_image, depth_mask_image, element );
    cv::Mat mask_image_or( cloud_ptr->height, cloud_ptr->width, CV_8UC1, cv::Scalar::all(0) );
    cv::bitwise_and( mask_image, depth_mask_image, mask_image_or );
    element.release();
    return mask_image_or;
}

