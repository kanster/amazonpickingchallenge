#include "include/rgb_recogniser.h"

/** constructor with image copier */
RGBRecogniser::RGBRecogniser(cv::Mat rgb_image, int idx, string models_dir ) {
    rgb_image_  = rgb_image.clone();
    target_idx_ = idx;
    models_dir_ = models_dir;
    /** @param: camera calibration results */
    /** point grey camera from Dinuka
      * 1074.6 0.000000 637.1
      * 0.000000 1076.6 532.6
      * 0.000000 0.000000 0.0
      */
    params_ <<  1074.6, 1076.6, 637.1, 532.6;
}

/** load configuration file */
/** set configuration work order and bin settings */
void RGBRecogniser::set_configuration(map<string, vector<string> >  bin_contents, vector< pair<string, string> > work_order) {
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

/** load object models in bin content */
void RGBRecogniser::load_models() {
    // load all models in directory
    for ( size_t i = 0; i < target_bin_content_.size(); ++ i ) {
        string content_object_path = models_dir_ + "/" + target_bin_content_[i] + ".xml";
        SP_Model m( new Model );
        m->load_model( content_object_path );
        this->models_.push_back( m );
    }
}

/** main process */
void RGBRecogniser::run( bool visualise ) {
    FeatureDetector feature_detector( "sift" );
    this->detected_features_ = feature_detector.process( this->rgb_image_ );


    FeatureMatcher feature_matcher( 5.0, 0.8, 128, "sift" );
    feature_matcher.load_models( this->models_ );
    this->matches_ = feature_matcher.process( this->detected_features_, target_in_bin_ );
    /*

    FeatureCluster feature_cluster( 200, 20, 7, 100 );
    this->clusters_ = feature_cluster.process( this->matches_ );

    cout << "start pose estimation ...\n";
    LevmarPoseEstimator levmar_estimator_1;
    levmar_estimator_1.init_params( 600, 200, 4, 5, 6, 10, params_ );
    levmar_estimator_1.process( this->matches_, this->models_[target_in_bin_], this->clusters_, this->objects_ );

    ProjectionFilter projection_filter_1;
    projection_filter_1.init_params( 5, (float)4096., 2, params_ );
    projection_filter_1.process( (this->models_)[target_in_bin_], this->matches_, this->clusters_, this->objects_ );

    foreach( object, this->objects_ ) {
        cout << object->model_->name_ << " with pose " << object->pose_.t_.vector().transpose() << "\n";
    }


    LevmarPoseEstimator levmar_estimator_2;
    levmar_estimator_2.init_params( 100, 500, 4, 6, 8, 5, params_ );
    levmar_estimator_2.process( this->matches_, this->models_[target_in_bin_], this->clusters_, this->objects_ );

    ProjectionFilter projection_filter_2;
    projection_filter_2.init_params( 7, (float)4096., 3, params_ );
    projection_filter_2.process( (this->models_)[target_in_bin_], this->matches_, this->clusters_, this->objects_ );

    cout << "\n-----------------------------------------\n";
    foreach( object, this->objects_ ) {
        cout << object->model_->name_ << " with pose " << object->pose_.t_.vector().transpose() << "\n";
    }
    */

    // visualisation
    if ( visualise == true ) {
        display_features( this->rgb_image_, this->detected_features_ );
        display_matches( this->rgb_image_, this->matches_ );
        /*
        display_clusters( this->rgb_image_, this->matches_, this->clusters_ );
        display_pose( this->rgb_image_, this->objects_, this->params_ );
        */
    }
}

