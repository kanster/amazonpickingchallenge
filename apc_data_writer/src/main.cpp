/*
 * main.cpp
 *
 * Copyright (c) 2013, Centre for Intelligent Mechatronics Systems, University of Technology, Sydney, Australia.
 * All rights reserved.
 *
 * This software was developed as a part of an industry based research project on Assistive Robotic Devices.
 *
 * Author: Kanzhi Wu
 * Date: 08/03/2015
 *
 */

#include "data_writer.h"

#include <pcl/console/parse.h>
#include <pcl/console/print.h>

void print_usage( char * prog_name ) {
    std::cout << "\nUsage: " << prog_name << " [options]\n"
              << "Options:\n"
              << "-------------------------------------------\n"
              << "\t-d <string>           save directory [Press s to save the current frame]\n"
              << "\t-c <bool>             save point cloud = 1\n"
              << "\t-h                    this help\n"
              << "\n\n";
}

int main( int argc, char ** argv ) {
    // arg parser
    if (pcl::console::find_argument (argc, argv, "-h") >= 0) {
        print_usage(argv[0]);
        return 0;
    }
    std::string dir;
    bool use_cloud;
    if ( pcl::console::parse(argc, argv, "-d", dir) >= 0 &&
         pcl::console::parse(argc, argv, "-c", use_cloud) >= 0) {
        pcl::console::print_highlight( "Save data to %s\n", dir.c_str() );
        ros::init( argc, argv, "apc_data_writer" );
        ros::NodeHandle nh("~");
        DataWriter writer( nh, dir, use_cloud );
        return 1;
    }
    else {
        print_usage(argv[0]);
    }
    return 0;
}
