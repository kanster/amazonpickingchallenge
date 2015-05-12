#include "include/helpfun/rgbd_recogniser.h"


//! constructor
RGBDRecogniser::RGBDRecogniser(cv::Mat rgb_image, cv::Mat mask_image, cv::Mat depth_image, Vector4f param) {
    cv::Mat depth_mask_image( depth_image.rows, depth_image.cols, CV_8UC1, cv::Scalar(0) );

    cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() );
    cloud_->width = depth_image.cols;
    cloud_->height = depth_image.rows;
    cloud_->is_dense = true;
    cloud_->points.resize( cloud_->width*cloud_->height );
    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    for ( int y = 0; y < depth_image.rows; ++ y ) {
        for ( int x = 0; x < depth_image.cols; ++ x ) {
            pcl::PointXYZRGB & pt = cloud_->points[y*cloud_->width+x];
            if ( depth_image.at<unsigned short>(y,x) != 0 ) {
                depth_mask_image.at<unsigned char>(y,x) = 255;
                float depth = (float)depth_image.at<unsigned short>(y,x)/10.0;
                pt.x = (x-param(2))*depth/param(0);
                pt.y = (y-param(3))*depth/param(1);
                pt.z = depth;

            }
            else {
                pt.x = bad_point;
                pt.y = bad_point;
                pt.z = bad_point;
            }
            uint8_t r = rgb_image.at<cv::Vec3b>(y,x)[2];
            uint8_t g = rgb_image.at<cv::Vec3b>(y,x)[1];
            uint8_t b = rgb_image.at<cv::Vec3b>(y,x)[0];
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            pt.rgb = *reinterpret_cast<float *>(&rgb);
        }
    }

    // generate mask image using availble depth information
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*2 + 1, 2*2+1 ), cv::Point( 2, 2 ) );
    cv::erode( depth_mask_image, depth_mask_image, element );
    cv::bitwise_and( mask_image, depth_mask_image, mask_image );

    rgb_image_ = rgb_image;
    mask_image_ = mask_image;
    depth_image_ = depth_image;
}


//! set bin contents
void RGBDRecogniser::set_bin_contents(vector<string> items) {
    target_bin_content_ = items;
}


//! set target item
void RGBDRecogniser::set_target_item(string target) {
    target_object_ = target;
    cout << "Target item " << target_object_ << endl;
    for ( size_t i = 0; i < target_bin_content_.size(); ++ i ) {
        cout << "   object " << i << " " << target_bin_content_[i] << "\n";
        if ( target_bin_content_[i] == target_object_ )
            target_in_bin_ = i;
    }
}



//! load object models in bin content
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

//! filter objects using the bin_contents_
void RGBDRecogniser::filter_objects( list<SP_Object> & objects ) {
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


/** run */
bool RGBDRecogniser::run( list<SP_Object> &objects, RGBDParam param, bool visualise ) {
    // feature detector
    FeatureDetector feature_detector( param.dp );
    this->detected_features_ = feature_detector.process( this->rgb_image_, this->cloud_, this->mask_image_ );

    FeatureMatcher feature_matcher( param.mp );
    feature_matcher.load_models( this->models_ );
    this->matches_ = feature_matcher.process( this->detected_features_, target_in_bin_ );

    FeatureCluster feature_cluster( param.cp );
    this->clusters_ = feature_cluster.process( this->matches_ );

    SVDPoseEstimator svd_pose_estimator( param.sp ); // cm
    objects.clear();
    objects = svd_pose_estimator.process( this->matches_, this->models_[target_in_bin_], this->clusters_ );

    foreach( object, objects ) {
        pcl::console::print_highlight( "Recognise %s with translation [%f, %f, %f] and score %f\n", object->model_->name_.c_str(), object->pose_.t_.x(), object->pose_.t_.y(), object->pose_.t_.z(), object->score_ );
    }

    if ( visualise == true ) {
        display_features( this->rgb_image_, this->detected_features_, false );
        display_matches( this->rgb_image_, this->matches_ );
        display_clusters( this->rgb_image_, this->matches_, this->clusters_ );
        display_pose( this->rgb_image_, objects, this->params_ );
        cv::waitKey(0);
    }
    if ( !objects.empty() )
        return true;
    else
        return false;
}

