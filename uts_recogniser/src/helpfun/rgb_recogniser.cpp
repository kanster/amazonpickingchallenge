#include "include/helpfun/rgb_recogniser.h"
#include <boost/timer.hpp>
#include <boost/progress.hpp>

/** constructor with image copier */
RGBRecogniser::RGBRecogniser(cv::Mat rgb_image) {
    rgb_image_  = rgb_image.clone();
}


RGBRecogniser::RGBRecogniser(cv::Mat rgb_image, cv::Mat mask_image) {
    rgb_image_  = rgb_image.clone();
    mask_image_ = mask_image.clone();
}

/** set target item related variables */
void RGBRecogniser::set_env_configuration(int idx, vector<pair<string, string> > work_order, map<string, vector<string> > bin_contents) {
    target_idx_ = idx;

    // read items in bin and the target item
    target_object_      = work_order[target_idx_].second;
    target_bin_content_ = bin_contents[work_order[target_idx_].first];
    cout << "Target index " << target_idx_ << " " << target_object_ << endl;
    // list the items
    cout << "== bin content objects: \n";
    for ( size_t i = 0; i < target_bin_content_.size(); ++ i ) {
        cout << "   object " << i << " " << target_bin_content_[i] << "\n";
        if ( target_bin_content_[i] == target_object_ )
            target_in_bin_ = i;
    }

}

/** set target item and neighboured items */
void RGBRecogniser::set_env_configuration( string target_item, vector<string> items ) {
    target_object_      = target_item;
    target_bin_content_ = items;
    cout << "Target item " << target_object_ << endl;

    for ( size_t i = 0; i < target_bin_content_.size(); ++ i ) {
        cout << "   object " << i << " " << target_bin_content_[i] << "\n";
        if ( target_bin_content_[i] == target_object_ )
            target_in_bin_ = i;
    }
}


//! set bin contents
void RGBRecogniser::set_bin_contents( vector<string> contents ) {
    target_bin_content_ = contents;
}

//! set target item
void RGBRecogniser::set_target_item(string target) {
    target_object_ = target;
    cout << "Target item " << target_object_ << endl;
    for ( size_t i = 0; i < target_bin_content_.size(); ++ i ) {
        cout << "   object " << i << " " << target_bin_content_[i] << "\n";
        if ( target_bin_content_[i] == target_object_ )
            target_in_bin_ = i;
    }
}


/** load object models in bin content */
void RGBRecogniser::load_models( string models_dir ) {
    models_dir_ = models_dir;

//    boost::timer t;
//    string content_object_path = models_dir_ + "/" + target_object_ + ".xml";
//    SP_Model m( new Model );
//    m->load_model( content_object_path );
//    this->models_.push_back( m );
//    pcl::console::print_value( "Load model %s takes %f s\n", target_object_.c_str(), t.elapsed() );
    // load all models in directory

    for ( size_t i = 0; i < target_bin_content_.size(); ++ i ) {
        {
            boost::timer t;
            string content_object_path = models_dir_ + "/" + target_bin_content_[i] + ".xml";
            SP_Model m( new Model );
            m->load_model( content_object_path );
            this->models_.push_back( m );
            pcl::console::print_value( "Load model %s takes %f s\n", target_bin_content_[i].c_str(), t.elapsed() );
        }
    }

}




/** filter objects using the bin_contents_ */
void RGBRecogniser::filter_objects( list<SP_Object> & objects ) {
    int n_target_object = 0;
    for ( int i = 0; i < (int)target_bin_content_.size(); ++ i ) {
        if ( target_bin_content_[i] == target_object_ )
            n_target_object ++;
    }

    if ( (int)objects.size() > n_target_object ) {
        list<SP_Object> filtered_objects;
        objects.sort(&compare_sp_object);
        int count = 0;
        for ( list<SP_Object>::iterator it = objects.begin(); it != objects.end(); ++ it ) {
            if ( count < n_target_object ) {
                filtered_objects.push_back( *it );
                break;
            }
            count ++;
        }
        objects.clear();
        objects = filtered_objects;
    }
}


/** main process */
bool RGBRecogniser::run( list<SP_Object> & objects, RGBParam param, int min_matches, int min_filtered_matches, bool visualise ) {
    // add mask image into feature detector
    FeatureDetector feature_detector( param.dp );
    this->detected_features_ = feature_detector.process( this->rgb_image_, this->mask_image_ );

    FeatureMatcher feature_matcher( param.mp );
    feature_matcher.load_models( this->models_ );
    this->matches_ = feature_matcher.process( this->detected_features_, target_in_bin_ );
    if ( (int)this->matches_.size() < min_matches ) {
        return false;
    }


    FeatureCluster feature_cluster( param.cp );
    this->clusters_ = feature_cluster.process( this->matches_ );


    LevmarPoseEstimator levmar_estimator_1( param.lmp1 );
    levmar_estimator_1.process( this->matches_, this->models_[target_in_bin_], this->clusters_, objects );

    ProjectionFilter projection_filter_1( param.pp1 );
    projection_filter_1.process( (this->models_)[target_in_bin_], this->matches_, this->clusters_, objects );



//    LevmarPoseEstimator levmar_estimator_2( param.lmp2 );
//    levmar_estimator_2.process( this->matches_, this->models_[target_in_bin_], this->clusters_, objects );

//    ProjectionFilter projection_filter_2(param.pp2);
//    projection_filter_2.process( (this->models_)[target_in_bin_], this->matches_, this->clusters_, objects );

    filter_objects( objects );
    foreach( object, objects ) {
        pcl::console::print_highlight( "Recognise %s with translation [%f, %f, %f] and score %f\n", object->model_->name_.c_str(), object->pose_.t_.x(), object->pose_.t_.y(), object->pose_.t_.z(), object->score_ );
    }
    // visualisation
    if ( visualise == true ) {
        display_features( this->rgb_image_, this->detected_features_ );
        display_matches( this->rgb_image_, this->matches_ );
        display_clusters( this->rgb_image_, this->matches_, this->clusters_, cv::Point2i(640, 0) );
        display_pose( this->rgb_image_, objects, param.lmp1.camera_param );
    }

    if ( !objects.empty() )
        return true;
    else
        return false;
}


