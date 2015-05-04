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
    struct EBParam{
        double max_scale;
        double min_scale;
        int inputh;
        int inputw;
        std::string conv0_kernel;
        std::string conv2_kernel;
        std::string conv4_kernel;

        EBParam(){
            max_scale = 0.4;
            min_scale = 1.4;
            inputh = 64;
            inputw = 64;
            conv0_kernel = "9x9";
            conv2_kernel = "9x9";
            conv4_kernel = "10x10";
        }

        EBParam( double max, double min, int h, int w, std::string conv0_k, std::string conv2_k, std::string conv4_k ) {
            max_scale = max;
            min_scale = min;
            inputh = h;
            inputw = w;
            conv0_kernel = conv0_k;
            conv2_kernel = conv2_k;
            conv4_kernel = conv4_k;
        }
    };


private:
    template <typename T> void init_detector( ebl::detector<T> & detect, ebl::configuration & conf, std::string & outdir, bool silent );

    void set_frame();

    void load_temp_conf( std::string conf_path );

    void set_mat_dir( std::string mat_dir );


private:
    ebl::detector<float> * pdetect_;

    cv::Mat rgb_image_, depth_image_;
    cv::Mat empty_image_, empty_depth_image_;
    cv::Mat mask_image_;

    ebl::idx<float> frame_;

    std::string target_item_;
    std::vector<std::string> items_;

    std::string conf_dir_;
    std::string mat_dir_;

    bool use_rgb_;

    ebl::configuration conf_;
    std::map<std::string, EBParam> item_params_map_;

public:
    EBRecogniser();

    void init( std::string tmp_conf_path, std::string mat_dir );

    void load_sensor_data( cv::Mat rgb_image, cv::Mat depth_image );

    void load_sensor_data(cv::Mat rgb_image);

    void load_info( cv::Mat empty_image, cv::Mat mask_image, cv::Mat empty_depth_image );

    void load_info( cv::Mat empty_image, cv::Mat mask_image );

    void set_conf_dir( std::string conf_dir );


    void set_env_configuration( std::string target_item, std::vector<std::string> items );

    void  process( std::vector< std::pair<std::string, std::vector<cv::Point> > > & results, bool use_rgb = true );

    void clear();
};





#endif
