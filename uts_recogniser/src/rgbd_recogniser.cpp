#include "include/rgbd_recogniser.h"

// public functions
/** constructor, load sensor data */
RGBDRecogniser::RGBDRecogniser(cv::Mat rgb_image, cv::Mat depth_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud/*, string seg_model_dir*/) {
    rgb_image_      = rgb_image;
    depth_image_    = depth_image;
    cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() );
    cloud_ = cloud;
//    seg_model_dir_ = seg_model_dir;
}

/** constructor, load sensor data */
RGBDRecogniser::RGBDRecogniser(cv::Mat rgb_image, cv::Mat mask_image, cv::Mat depth_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud/*, string seg_model_dir*/) {
    rgb_image_      = rgb_image;
    mask_image_     = mask_image;
    depth_image_    = depth_image;
    cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() );
    cloud_ = cloud;
    cout << "constructor of rgbd recogniser\n";
//    seg_model_dir_ = seg_model_dir;
}


/** set configuration work order and bin settings */
void RGBDRecogniser::set_env_configuration(int idx, vector<pair<string, string> > work_order, map<string, vector<string> > bin_contents) {
    target_idx_ = idx;
    string target_bin_idx = work_order[target_idx_].first;
    target_object_ = work_order[target_idx_].second;
    target_bin_content_ = bin_contents[target_bin_idx];
    cout << "== bin content objects: \n";
    for ( size_t i = 0; i < target_bin_content_.size(); ++ i ) {
        cout << "\tobject " << i << " " << target_bin_content_[i] << "\n";
        if ( target_bin_content_[i] == target_object_ )
            target_in_bin_ = i;
    }
}

/** set target item and neighboured items */
void RGBDRecogniser::set_env_configuration( string target_item, vector<string> items ) {
    target_object_      = target_item;
    target_bin_content_ = items;
    cout << "Target item " << target_object_ << endl;

    for ( size_t i = 0; i < target_bin_content_.size(); ++ i ) {
        cout << "   object " << i << " " << target_bin_content_[i] << "\n";
        if ( target_bin_content_[i] == target_object_ )
            target_in_bin_ = i;
    }
}


/** set camera params */
void RGBDRecogniser::set_camera_params(float fx, float fy, float cx, float cy) {
    params_ << fx, fy, cx, cy;
}


/** load object models in bin content */
void RGBDRecogniser::load_models( string models_dir ) {
    models_dir_ = models_dir;

    // load all models in directory
    for ( size_t i = 0; i < target_bin_content_.size(); ++ i ) {
        string content_object_path = models_dir_ + "/" + target_bin_content_[i] + ".xml";
        SP_Model m( new Model );
        m->load_model( content_object_path );
        this->models_.push_back( m );
    }
}

/** filter objects using the bin_contents_ */
void RGBDRecogniser::filter_objects() {
    int n_target_object = 0;
    for ( int i = 0; i < (int)target_bin_content_.size(); ++ i ) {
        if ( target_bin_content_[i] == target_object_ )
            n_target_object ++;
    }

    if ( (int)this->objects_.size() > n_target_object ) {
        list<SP_Object> filtered_objects;
        this->objects_.sort(&compare_sp_object);
        int count = 0;
        for ( list<SP_Object>::iterator it = this->objects_.begin(); it != this->objects_.end(); ++ it ) {
            if ( count < n_target_object ) {
                filtered_objects.push_back( *it );
                break;
            }
            count ++;
        }
        this->objects_.clear();
        this->objects_ = filtered_objects;
    }
}


/** run */
bool RGBDRecogniser::run( bool visualise ) {
    MaskGenerator mask_generator;
    this->mask_image_ = mask_generator.process( this->cloud_, this->mask_image_ );


    FeatureDetector feature_detector( "sift" );
    this->detected_features_ = feature_detector.process( this->rgb_image_, this->cloud_, this->mask_image_ );

    FeatureMatcher feature_matcher( 5.0, 0.9, 128, "sift" );
    feature_matcher.load_models( this->models_ );
    this->matches_ = feature_matcher.process( this->detected_features_, target_in_bin_ );


    FeatureCluster feature_cluster( 30., 5., 7, 100 );
    this->clusters_ = feature_cluster.process( this->matches_ );

    SVDPoseEstimator svd_pose_estimator( 1.0, 7 ); // cm
    this->objects_ = svd_pose_estimator.process( this->matches_, this->models_[target_in_bin_], this->clusters_ );

    cout << "\n-----------------------------------------\n";
    foreach( object, this->objects_ ) {
        pcl::console::print_highlight( "Recognise %s with translation [%f, %f, %f] and score %f\n", object->model_->name_.c_str(), object->pose_.t_.x(), object->pose_.t_.y(), object->pose_.t_.z(), object->score_ );
    }
    cout << "\n-----------------------------------------\n";

    if ( visualise == true ) {
//        display_mask( this->mask_image_ );
        display_features( this->rgb_image_, this->detected_features_, false );
        display_matches( this->rgb_image_, this->matches_ );
        display_clusters( this->rgb_image_, this->matches_, this->clusters_ );
        display_pose( this->rgb_image_, this->objects_, this->params_ );
    }

    return true;
}


list<SP_Object> RGBDRecogniser::get_objects() {
    return this->objects_;
}
