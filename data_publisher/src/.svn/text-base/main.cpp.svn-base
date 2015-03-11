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


#include "data_publisher.h"

#include <pcl/console/parse.h>
#include <pcl/console/print.h>

void print_usage( char * prog_name ) {
    std::cout << "\nUsage: " << prog_name << " [options]\n"
              << "Options:\n"
              << "-------------------------------------------\n"
              << "\t-d <string>           save directory\n"
              << "\t-n <int>              number"
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
    int n;
    if ( pcl::console::parse(argc, argv, "-d", dir) >= 0 &&
         pcl::console::parse(argc, argv, "-n", n) >= 0) {
        pcl::console::print_highlight( "Publish %d frames of data from %s\n", n, dir.c_str() );
        ros::init( argc, argv, "data_publisher" );
        ros::NodeHandle nh("~");
        DataPublisher publisher( nh, dir, n );
        return 1;
    }
    else {
        print_usage(argv[0]);
    }
    return 0;
}
