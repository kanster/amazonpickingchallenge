#ifndef RGB_RECOGNISER_H
#define RGB_RECOGNISER_H

#include "include/utils.h"
#include "include/json_parser.hpp"

#include "include/feature_detector.h"
#include "include/feature_matcher.h"
#include "include/feature_cluster.h"
#include "include/levmar_pose_estimator.h"
#include "include/projection_filter.h"

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <dirent.h>
#include <stdlib.h>
#include <algorithm>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>


using namespace std;


class RGBRecogniser{
private:
    string models_dir_;

    cv::Mat rgb_image_;
    int target_idx_;

    Vector4f params_;

    vector<string> target_bin_content_;
    string target_object_;
    int target_in_bin_;

    vector<SP_Model> models_;
    void load_models();

    vector<DetectedFeatureRGB> detected_features_;

    typedef vector<MatchRGB> Matches;
    Matches matches_;

    typedef list<int> Cluster;
    vector<Cluster> clusters_;

    list<SP_Object> objects_;


public:
    /** constructor */
    RGBRecogniser( cv::Mat rgb_image, int idx, string models_dir );

    /** load target object and configuration list */
    void set_configuration( map<string, vector<string> >  bin_contents, vector< pair<string, string> > work_order );

    void run( bool visualise );

};

#endif // RGB_RECOGNISER_H
