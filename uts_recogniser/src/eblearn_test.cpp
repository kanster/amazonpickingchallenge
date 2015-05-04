/** eblearn test using eblearn_recogniser*/

#include "include/helpfun/eblearn_recogniser.h"

using namespace std;

int main( int argc, char ** argv ) {

    EBRecogniser ebr;
    ebr.init( string(argv[1]), string(argv[2]) );
    cout << "load image\n";
    cv::Mat rgb_image = cv::imread( string(argv[3]), CV_LOAD_IMAGE_COLOR );

    ebr.load_sensor_data( rgb_image );

    vector<string> items;
    for ( int i = 4; i < argc; ++ i )
        items.push_back( string(argv[i]) );
    ebr.set_env_configuration( items.front(), items );
    cout << "set env config\n";

    vector<pair<string, vector<cv::Point> > > results;
    ebr.process( results );
    for ( int i = 0; i < (int)results.size(); ++ i ) {
        cv::putText( rgb_image, results[i].first, results[i].second[0], CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.5, cv::Scalar(0,0,255));
        for ( int j = 0; j < (int)results[i].second.size(); ++ j )
            cv::line( rgb_image, results[i].second[j], results[i].second[(j+1)%results[i].second.size()], cv::Scalar(255, 0, 0), 1 );
    }

    cv::imshow( "rgb_image", rgb_image );
    string results_path = string(argv[argc-1]);
    cv::imwrite( results_path+".jpg", rgb_image );
    cv::waitKey(0);
    return 0;
}

