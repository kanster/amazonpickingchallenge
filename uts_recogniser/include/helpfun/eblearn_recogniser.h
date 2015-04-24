#ifndef EBLEARN_RECOGNISER_H
#define EBLEARN_RECOGNISER_H

#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <string>
#include <iostream>
#include <algorithm>
#include <vector>
#include <sstream>
#include <iomanip>
#include <time.h>

#include <eblearn/detector.h>
#include <eblearn/detector.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eblearn/defines_tools.h>
#include <eblearn/thread.h>
#include <eblearn/netconf.h>
#include <eblearn/configuration.h>
#include <eblearn/bbox.h>
#include <eblearn/bootstrapping.h>

#include <eblearn/libeblearn.h>
#include <eblearn/libeblearntools.h>
#include <eblearn/pascal_xml.h>


class EBRecogniser{
private:
    template <typename T> void init_detector( ebl::detector<T> & detect, ebl::configuration & conf, std::string & outdir, bool silent );

    void set_frame();

private:
    std::string item_name_;
    ebl::detector<float> * pdetect_;

    cv::Mat rgb_image_, depth_image_;
    cv::Mat empty_image_, empty_depth_image_;
    cv::Mat mask_image_;

    ebl::idx<float> frame_;

    std::string target_item_;
    std::vector<std::string> items_;

    std::string conf_dir_;

    bool use_rgb_;


public:
    EBRecogniser();

    void load_sensor_data( cv::Mat rgb_image, cv::Mat depth_image );

    void load_sensor_data(cv::Mat rgb_image);

    void load_info( cv::Mat empty_image, cv::Mat mask_image, cv::Mat empty_depth_image );

    void load_info( cv::Mat empty_image, cv::Mat mask_image );

    void set_conf_dir( std::string conf_dir );

    void set_env_configuration( std::string target_item, std::vector<std::string> items );

    std::vector< std::pair<std::string, std::vector<cv::Point> > > process( bool use_rgb = true );

    void clear();
};





#endif
