/*
 * data_recorder.cpp
 * Description: write collected data to disk, for eblearn
 *
 * Copyright (c) 2013, Centre for Intelligent Mechatronics Systems, University of Technology, Sydney, Australia.
 * All rights reserved.
 *
 * This software was developed as a part of an industry based research project on Assistive Robotic Devices.
 *
 * Author: Kanzhi Wu
 * Date: 13/03/2015
 *
 */



#include "data_recorder.h"
#include <cv_bridge/cv_bridge.h>
#include <boost/timer.hpp>

DataRecorder::DataRecorder(ros::NodeHandle nh, std::string dir) {
    nh_ = & nh;
    dir_ = dir;
    count_ = 0;
    box_size_ = 64;
    start_ = false;

    cv::namedWindow( "image" );

    std::string image_topic = nh.resolveName("/camera/rgb/image_color");
    ROS_INFO( "Subscribing to topic %s", image_topic.c_str() );
    image_sub_ = nh.subscribe( image_topic, 1, &DataRecorder::image_callback, this );
    ros::Rate loop(10);
    while( ros::ok() ) {
        ros::spin();
        loop.sleep();
    }
}


DataRecorder::~DataRecorder(){
    cv::destroyWindow( "image" );
}



/** main callback function */
void DataRecorder::image_callback(const sensor_msgs::ImageConstPtr &image_msg) {
    ROS_INFO_ONCE("Image information available");

    image_ = cv_bridge::toCvCopy( image_msg, sensor_msgs::image_encodings::BGR8 )->image;
    rect_ = cv::Rect( image_.cols/2-box_size_/2, image_.rows/2-box_size_/2, box_size_, box_size_ );

    cv::Mat vis_image = image_.clone();
    cv::rectangle( vis_image, rect_, cv::Scalar(255, 0, 0), 2, 8 );
    cv::imshow( "image", vis_image );
    char k = cv::waitKey(1);
		/*
    if ( k == 's' && start_ == false ) {
        start_ = true;
    }
    if ( k == 'e' && start_ == true ) {
        start_ = false;
    }

    if ( start_ == true ) {
		*/
    if ( k == 'c' ) {
        std::string image_name = dir_ + "/" + boost::lexical_cast<std::string>(count_) + ".jpg";
        printf("Save image to %s\n", image_name.c_str());
        cv::imwrite( image_name, image_(rect_).clone() );
        count_ ++;
    }

    vis_image.release();
}


int main( int argc, char ** argv ) {
    if ( argc != 2 )
        std::cout << argv[0] << " " << "<directory>\n";
    char* dir = argv[1];
    std::string str_dir(dir);
    ros::init( argc, argv, "data_recorder" );
    ros::NodeHandle nh("~");
    DataRecorder recorder( nh, str_dir );
    return 1;
}
