#include "include/rgbd_recogniser.h"

// public functions
/** constructor, load sensor data */
RGBD_Recogniser::RGBD_Recogniser(cv::Mat rgb_image, cv::Mat depth_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int idx, string models_dir ) {
    rgb_image_      = rgb_image;
    depth_image_    = depth_image;
    cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() );
    cloud_ = cloud;

    target_idx_ = idx;
    models_dir_ = models_dir;

    params_ << 518.2731, 314.1237, 518.7466, 274.2845;
}

/** set configuration work order and bin settings */
void RGBD_Recogniser::set_configuration(map<string, vector<string> >  bin_contents, vector< pair<string, string> > work_order) {
    string target_bin_idx = work_order[target_idx_].first;
    target_object_ = work_order[target_idx_].second;
    target_bin_content_ = bin_contents[target_bin_idx];
    cout << "== bin content objects: \n";
    for ( size_t i = 0; i < target_bin_content_.size(); ++ i ) {
        cout << "\tobject " << i << " " << target_bin_content_[i] << "\n";
        if ( target_bin_content_[i] == target_object_ )
            target_in_bin_ = i;
    }
    // load models
    load_models();
}

// private functions
void RGBD_Recogniser::load_models() {
    // load all the models
    for ( size_t i = 0; i < target_bin_content_.size(); ++ i ) {
        string content_object_path = models_dir_ + target_bin_content_[i] + ".xml";
        SP_Model m(new Model);
        m->load_model( content_object_path );
        this->models_.push_back( m );
    }
}

/** run */
void RGBD_Recogniser::run() {
    MaskGenerator mask_generator( true );
    this->mask_image_ = mask_generator.process( this->cloud_ );

    FeatureDetector feature_detector( "sift" );
    this->detected_features_ = feature_detector.process( this->rgb_image_, this->cloud_, this->mask_image_ );

    CloudSegmenter cloud_segmenter( false, 0, "../data/model/" );
    this->labeled_cloud_ = cloud_segmenter.process( this->cloud_, this->detected_features_ );

    FeatureMatcher feature_matcher( 5.0, 0.8, 128, "sift" );
    this->matches_ = feature_matcher.process( this->detected_features_, target_in_bin_, this->match_clusters_ );

    SVDPoseEstimator svd_pose_estimator( 0.5, 7 ); // cm
    this->objects_ = svd_pose_estimator.process( this->matches_, this->models_[target_in_bin_], this->match_clusters_ );
}
