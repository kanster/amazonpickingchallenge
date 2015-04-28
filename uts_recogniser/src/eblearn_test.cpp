/** eblearn test using eblearn_recogniser*/

#include "include/helpfun/eblearn_recogniser.h"

using namespace std;

int main( int argc, char ** argv ) {
    EBRecogniser ebr;

    cv::Mat rgb_image = cv::imread( string(argv[1]), CV_LOAD_IMAGE_COLOR );
    ebr.load_sensor_data( rgb_image );

    ebr.set_conf_dir( string(argv[2]) );

    vector<string> items;
    for ( int i = 3; i < argc-1; ++ i )
        items.push_back( string(argv[i]) );
    ebr.set_env_configuration( items[0], items );

    vector<pair<string, vector<cv::Point> > > results = ebr.process();
    for ( int i = 0; i < (int)results.size(); ++ i ) {
        cv::putText( rgb_image, results[i].first, results[i].second[0], CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.5, cv::Scalar(0,0,255));
        for ( int j = 0; j < (int)results[i].second.size(); ++ j )
            cv::line( rgb_image, results[i].second[j], results[i].second[(j+1)%results[i].second.size()], cv::Scalar(255, 0, 0), 1 );
    }

    cv::imshow( "rgb_image", rgb_image );
    string results_path = string(argv[argc-1]);
    cv::imwrite( results_path, rgb_image );
    cv::waitKey(0);
}

