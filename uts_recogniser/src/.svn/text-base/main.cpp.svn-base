#include "include/uts_recogniser.h"
#include "include/json_parser.hpp"

int main( int argc, char ** argv ) {
    ros::init( argc, argv, "uts_recogniser" );
    ros::NodeHandle nh("~");
    UTSRecogniser uts_reco(nh);
    uts_reco.start_monitor();
//    ros::spin();
    return 0;
}
