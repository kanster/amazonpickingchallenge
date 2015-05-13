/** kernel descriptor */

#include "include/helpfun/kd_recogniser.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <queue>

KDRecogniser::SlidingWindowDetector::SlidingWindowDetector() {
}


KDRecogniser::SlidingWindowDetector::SlidingWindowDetector( string model, string svm_model_file, string kdes_param_file, string models_folder, unsigned int model_type ) {
    pkdm_ = new KernelDescManager( model, svm_model_file, kdes_param_file, models_folder, model_type, false );
}


vector<MatrixXf> KDRecogniser::SlidingWindowDetector::process(cv::Mat image, vector<int> indices, int patch_size, int step_size) {
    IplImage * ipl_image = new IplImage(image);
    return process( ipl_image, indices, patch_size, step_size );
}


vector<MatrixXf> KDRecogniser::SlidingWindowDetector::process(IplImage *ipl_image, vector<int> indices, int patch_size, int step_size) {
//    int n_stepsx = floor((ipl_image->width-patch_size)*1.0/step_size)+1;
//    int n_stepsy = floor((ipl_image->height-patch_size)*1.0/step_size)+1;

    int n_stepsx = ceil((ipl_image->width-patch_size)*1.0/step_size)+1;
    int n_stepsy = ceil((ipl_image->height-patch_size)*1.0/step_size)+1;

    vector<MatrixXf> all_scores( (int)pkdm_->GetModelList()->size(), MatrixXf::Zero(n_stepsy, n_stepsx) ); // item scores for all items

    int count = 0;
    for ( int iy = n_stepsy-1; iy >= 0; -- iy ) {
        for ( int ix = 0; ix < n_stepsx; ++ ix ) {
            int y = iy*step_size;
            int x = ix*step_size;
            int height = std::min( patch_size, ipl_image->height-y );
            int width = std::min( patch_size, ipl_image->width-x );
            CvRect roi = cvRect( x, y, width, height );
            cvSetImageROI(ipl_image, roi);
            IplImage * patch = cvCreateImage( cvSize(roi.width, roi.height), ipl_image->depth, ipl_image->nChannels );
            cvCopy(ipl_image, patch);

            MatrixXf imfea;
            VectorXf scores;

            pkdm_->Process( imfea, patch );
            pkdm_->Classify( scores, imfea );

            for ( int i = 0; i < (int)pkdm_->GetModelList()->size(); ++ i ) {
                all_scores[i](iy, ix) = scores(i);
            }
            count ++;
        }
    }


    vector<MatrixXf> item_scores;
    for ( int i = 0; i < (int)indices.size(); ++ i ) {
        item_scores.push_back( all_scores[indices[i]] );
    }

    return item_scores;
}


vector<string> * KDRecogniser::SlidingWindowDetector::get_models_list() {
    return this->pkdm_->GetModelList();
}


KernelDescManager * KDRecogniser::SlidingWindowDetector::get_pkdm() {
    return this->pkdm_;
}


bool KDRecogniser::find_blobs(const cv::Mat &binary, vector<vector<cv::Point2i> > &blobs) {
    blobs.clear();

    // Fill the label_image with the blobs
    // 0  - background
    // 1  - unlabelled foreground
    // 2+ - labelled foreground

    cv::Mat label_image;
    binary.convertTo(label_image, CV_32SC1);

    int label_count = 2; // starts at 2 because 0,1 are used already
    for(int y=0; y < label_image.rows; y++) {
        int *row = (int*)label_image.ptr(y);
        for(int x=0; x < label_image.cols; x++) {
            if(row[x] != 255.0) {
                continue;
            }

            cv::Rect rect;
            cv::floodFill(label_image, cv::Point(x,y), label_count, &rect, 0, 0, 4);
            vector<cv::Point2i> blob;
            for(int i=rect.y; i < (rect.y+rect.height); i++) {
                int *row2 = (int*)label_image.ptr(i);
                for(int j=rect.x; j < (rect.x+rect.width); j++) {
                    if(row2[j] != label_count) {
                        continue;
                    }
                    blob.push_back(cv::Point2i(j,i));
                }
            }
            if ( blob.size() > 100 ) {
                blobs.push_back(blob);
                label_count++;
            }
        }
    }
    return !blobs.empty();
}

//! visualise matrixxf scores
cv::Mat KDRecogniser::from_score( MatrixXf score, int scale ) {
    // find min and max value in score
    int maxx, maxy, minx, miny;
    float min = score.minCoeff(&miny, &minx);
    float max = score.maxCoeff(&maxy, &maxx);
    float range = max-min;
//    cout /*<< " range: " << range */<< " max: " << max << endl;
//    cout << "score size: " << score.rows() << ", " << score.cols() << endl;
    cv::Mat img(score.rows(), score.cols(), CV_8UC1);
    for ( int y = 0; y < img.rows; ++ y )
        for ( int x = 0; x < img.cols; ++ x )
//            img.at<uchar>(y,x) = (int)((score(y,x)-min)*255);
            img.at<uchar>(y,x) = (int)((score(y,x)-min)/range*255);
    cv::resize( img, img, cv::Size(img.cols*scale, img.rows*scale), 0, 0, CV_INTER_NN );
    return img;
}


//! load image and point cloud
KDRecogniser::KDRecogniser(){
}


void KDRecogniser::set_flags(bool display) {
    display_ = display;
}


void KDRecogniser::load_sensor_data(cv::Mat rgb_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    this->rgb_image_ = rgb_image;
    this->cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB> () );
    this->cloud_ = cloud;
}


void KDRecogniser::load_sensor_data(cv::Mat rgb_image, cv::Mat depth_image) {
    this->rgb_image_ = rgb_image;
    this->depth_image_ = depth_image;
}

void KDRecogniser::load_sensor_data(cv::Mat rgb_image) {
    this->rgb_image_ = rgb_image;
}


//! set target item and neighboured items
void KDRecogniser::set_env_configuration( string target_item, vector<string> items ) {
    target_object_      = target_item;
    target_bin_content_ = items;
    cout << "Target item " << target_object_ << endl;

    for ( size_t i = 0; i < target_bin_content_.size(); ++ i ) {
        cout << "   object " << i << " " << target_bin_content_[i] << "\n";
        if ( target_bin_content_[i] == target_object_ )
            target_in_bin_ = i;
    }
}



//! load empty information
void KDRecogniser::load_info(cv::Mat empty_image, cv::Mat mask_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_cloud) {
    this->empty_image_ = empty_image;
    this->mask_image_ = mask_image;
    this->empty_cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB> () );
    this->empty_cloud_ = empty_cloud;
}

void KDRecogniser::load_info(cv::Mat empty_image, cv::Mat mask_image, cv::Mat empty_depth_image) {
    this->empty_image_ = empty_image;
    this->mask_image_ = mask_image;
    this->empty_depth_ = empty_depth_image;
}

void KDRecogniser::load_info(cv::Mat empty_image, cv::Mat mask_image) {
    this->empty_image_ = empty_image;
    this->mask_image_ = mask_image;
}



//! init kernel descriptor
void KDRecogniser::init_libkdes(string svm_model_name, string kdes_model_name, string model_folder, string model, unsigned int model_type){
    svm_model_name_ = svm_model_name;
    kdes_model_name_ = kdes_model_name;
    model_folder_ = model_folder;
    model_ = model;
    model_type_ = model_type;
    swd_ = new SlidingWindowDetector( model_, svm_model_name_, kdes_model_name_, model_folder_, model_type_ );

}

//! get convex hull
vector<cv::Point> KDRecogniser::convex_hull(cv::Mat detect_img, cv::Mat depth_img) {
    vector<cv::Point> convex_pts;
    cv::bitwise_and( detect_img, depth_img, detect_img );
    vector< vector<cv::Point2i> > blobs;
    find_blobs(detect_img, blobs );
    if ( !blobs.empty() ) {
        int max_blob_idx;
        int max_blob_sz = 0;
        for ( int i = 0; i < (int)blobs.size(); ++ i ) {
            if ( blobs[i].size() > max_blob_sz ) {
                max_blob_sz = (int)blobs[i].size();
                max_blob_idx = i;
            }
        }
        detect_img.setTo( 0 );
        for ( int i = 0; i < (int)blobs[max_blob_idx].size(); ++ i )
            detect_img.at<uchar>(blobs[max_blob_idx][i].y,blobs[max_blob_idx][i].x) = static_cast<unsigned char>(255);
        // simple edge detector
        vector<cv::Point> contour;
        for ( int y = 1; y < detect_img.rows-1; ++ y )
            for ( int x = 1; x < detect_img.cols-1; ++ x )
                if ( detect_img.at<uchar>(y,x) > 0 )
                    if ( detect_img.at<uchar>(y+1,x) == 0 ||
                         detect_img.at<uchar>(y-1,x) == 0 ||
                         detect_img.at<uchar>(y,x+1) == 0 ||
                         detect_img.at<uchar>(y,x-1) == 0)
                        contour.push_back( cv::Point(x, y) );
        if ( !contour.empty() )
            convex_pts = get_convex_hull( contour );
    }
    return convex_pts;
}


//! process
void KDRecogniser::patch_process(vector<pair<string, vector<cv::Point> > > &results, bool use_rgb) {
    results.clear();
    // convert depth image in online mode and offline mode
    if ( this->depth_image_.type() != 5 ) {
        cv::Mat new_depth_image( this->depth_image_.rows, this->depth_image_.cols, CV_32FC1 );
        for ( int y = 0; y < this->depth_image_.rows; ++ y )
            for ( int x = 0; x < this->depth_image_.cols; ++ x )
                new_depth_image.at<float>(y,x) = static_cast<float>(this->depth_image_.at<unsigned short>(y,x));
        this->depth_image_.release();
        this->depth_image_ = new_depth_image.clone();
        new_depth_image.release();
    }


    // generate binary image from depth image subtraction
    cv::Mat bin_depth_image( this->depth_image_.rows, this->depth_image_.cols, CV_8UC1, cv::Scalar(0) );
    for ( int y = 0; y < bin_depth_image.rows; ++ y )
        for ( int x = 0; x < bin_depth_image.cols; ++ x ) {
            if ( this->depth_image_.at<float>(y,x) > 0 ) {
                if ( abs(this->depth_image_.at<float>(y,x)-
                         this->empty_depth_.at<unsigned short>(y,x)) > 3 )
                    bin_depth_image.at<uchar>(y,x) = static_cast<unsigned char>(255);
            }
            else
                bin_depth_image.at<uchar>(y,x) = static_cast<unsigned char>(255);
        }




    // threshold for rgb each channel
    int threshold = 50;

    // generate subtracted image using rgb information
    sub_image_.setTo( cv::Scalar(0) );
    if ( !use_rgb ) {
        cv::Mat empty_image_mono;
        cv::cvtColor( empty_image_, empty_image_mono, CV_BGR2GRAY );

        cv::Mat item_image_mono;
        cv::cvtColor( rgb_image_, item_image_mono, CV_BGR2GRAY );

        cv::Mat diff_image_mono;
        cv::absdiff( empty_image_mono, item_image_mono, diff_image_mono );
        cv::threshold( diff_image_mono, sub_image_, threshold, 255.0, CV_THRESH_BINARY );
    }
    else {
        vector<cv::Mat> empty_image_rgbs;
        cv::split( empty_image_, empty_image_rgbs );
        vector<cv::Mat> item_image_rgbs;
        cv::split( rgb_image_, item_image_rgbs );
        vector<cv::Mat> diff_image_rgbs;
        for ( int i = 0; i < 3; i ++ ) {
            cv::Mat diff_image_rgb;
            cv::absdiff( empty_image_rgbs[i], item_image_rgbs[i], diff_image_rgb );
            diff_image_rgbs.push_back( diff_image_rgb );
            cv::threshold( diff_image_rgbs[i], diff_image_rgbs[i], threshold, 255.0, CV_THRESH_BINARY );
        }
        cv::bitwise_or( diff_image_rgbs[0], diff_image_rgbs[1], diff_image_rgbs[1] );
        cv::bitwise_or( diff_image_rgbs[1], diff_image_rgbs[2], sub_image_ );
    }

    // find blobs in the subtracted images and find the bbox for each blob
    vector< vector<cv::Point2i> > blobs;
    vector< pair<cv::Point2i, cv::Point2i> > blob_bbox;
    cv::Mat mask_bbox( sub_image_.rows, sub_image_.cols, CV_8UC1, cv::Scalar::all(0) );
    if ( find_blobs( sub_image_, blobs ) ) {
        for ( int i = 0; i < (int)blobs.size(); ++ i ) {
            pair<cv::Point2i, cv::Point2i> bbox; // min, max
            bbox.first.x = rgb_image_.cols;
            bbox.first.y = rgb_image_.rows;
            bbox.second.x = 0;
            bbox.second.y = 0;
            for ( int j = 0; j < (int)blobs[i].size(); ++ j ) {
                if ( blobs[i][j].x < bbox.first.x )
                    bbox.first.x = blobs[i][j].x;
                if ( blobs[i][j].y < bbox.first.y )
                    bbox.first.y = blobs[i][j].y;
                if ( blobs[i][j].x > bbox.second.x )
                    bbox.second.x = blobs[i][j].x;
                if ( blobs[i][j].y > bbox.second.y )
                    bbox.second.y = blobs[i][j].y;
                cv::Point2i & pt = blobs[i][j];
                mask_bbox.at<uchar>(pt.y, pt.x) = (unsigned char)255;
            }
        }
    }

    // generated the new mask image by bitwise and
    cv::bitwise_and( this->mask_image_, mask_bbox, this->mask_image_ );

    // remove small bbox
    cv::Mat blob_rgb = rgb_image_.clone();
    blob_bbox.clear();
    mask_bbox = cv::Mat( sub_image_.rows, sub_image_.cols, CV_8UC1, cv::Scalar::all(0) );
    if ( find_blobs( this->mask_image_, blobs ) ) {
        for ( int i = 0; i < (int)blobs.size(); ++ i ) {
            pair<cv::Point2i, cv::Point2i> bbox; // min, max
            bbox.first.x = sub_image_.cols;
            bbox.first.y = sub_image_.rows;
            bbox.second.x = 0;
            bbox.second.y = 0;
            for ( int j = 0; j < (int)blobs[i].size(); ++ j ) {
                if ( blobs[i][j].x < bbox.first.x )
                    bbox.first.x = blobs[i][j].x;
                if ( blobs[i][j].y < bbox.first.y )
                    bbox.first.y = blobs[i][j].y;
                if ( blobs[i][j].x > bbox.second.x )
                    bbox.second.x = blobs[i][j].x;
                if ( blobs[i][j].y > bbox.second.y )
                    bbox.second.y = blobs[i][j].y;
            }
            // minima size for subtracted images
            if ( bbox.second.y-bbox.first.y > 32 && bbox.second.x-bbox.first.x > 32 ) {
                blob_bbox.push_back( bbox );
                cv::rectangle( blob_rgb, bbox.first, bbox.second, cv::Scalar::all(255));
            }
        }
    }
    if ( display_ ) {
        cv::imshow( "blob_rgb", blob_rgb );
        cv::waitKey( 500 );
    }

    if ( blob_bbox.empty() )
        return;

    // generate indices for items
    vector<string> svm_models_list = *(this->swd_->get_models_list());
    dup_items_.clear();
    dup_items_ = duplicated_bin_contents( this->target_bin_content_ );
    vector<int> content_in_svm_indices;     // content item indices in svm model
    if ( !dup_items_.empty() ) {
        for ( int i = 0; i < (int)dup_items_.size(); ++ i ) {
            int pos = find(svm_models_list.begin(), svm_models_list.end(), dup_items_[i].first) - svm_models_list.begin();
            cout << "Name: " << dup_items_[i].first << ", " << pos << endl;
            if ( pos < svm_models_list.size() )
                content_in_svm_indices.push_back( pos );
        }
    }
    if ( content_in_svm_indices.empty() )
        return;


    // sliding window parameters
    int step_size = 12;
    int patch_size = 64;
    IplImage * image = new IplImage(this->rgb_image_);

    double exec_time;
    Timer timer;
    timer.start();
    // generate score
    vector< vector<MatrixXf> > scores_all;
    for ( int i = 0; i < (int)blob_bbox.size(); ++ i ) {
        // copy roi image
        CvRect roi = cvRect( blob_bbox[i].first.x, blob_bbox[i].first.y, blob_bbox[i].second.x-blob_bbox[i].first.x, blob_bbox[i].second.y-blob_bbox[i].first.y );
        cvSetImageROI(image, roi);
        IplImage * patch = cvCreateImage( cvSize(roi.width, roi.height), image->depth, image->nChannels );
        cvCopy(image, patch);
        vector< MatrixXf > scores_per_patch;
        if ( patch->height < patch_size || patch->width < patch_size )  {
            KernelDescManager * pkdm = swd_->get_pkdm();
            MatrixXf imfea;
            VectorXf score;
            pkdm->Process( imfea, patch );
            pkdm->Classify( score, imfea );
            for ( int y = 0; y < (int)content_in_svm_indices.size(); ++ y ) {
                MatrixXf item_score(1,1);
                item_score << score(content_in_svm_indices[y],0);
                scores_per_patch.push_back( item_score );
            }
        }
        else {
            vector<MatrixXf> scores = swd_->process( patch, content_in_svm_indices, patch_size, step_size );
            for ( int j = 0; j < (int)scores.size(); ++ j )
                scores_per_patch.push_back( scores[j] );
        }
        scores_all.push_back( scores_per_patch );
    }

    exec_time = timer.get();
    cout << "Execution time: " << setw(8) << exec_time << endl;

    // generate sq values
    if ( display_ == true ) {
        cout << "\n------------------------\n";
        for ( int i = 0; i < (int)scores_all.size(); ++ i )
            for ( int oi = 0; oi < (int)scores_all[i].size(); ++ oi )
                cout << dup_items_[oi].first << ": \n" << scores_all[i][oi] << "\n";
        cout << "\n------------------------\n";

        if ( dup_items_.size() != 1 ) {
            for ( int i = 0; i < (int)scores_all.size(); ++ i ) {
                int ny = scores_all[i].front().rows();
                int nx = scores_all[i].front().cols();
                int no = scores_all[i].size();
                for ( int y = 0; y < ny; ++ y ) {
                    for ( int x = 0; x < nx; ++ x ) {
                        float sq = 0.;
                        for ( int oi = 0; oi < no; ++ oi )
                            sq += scores_all[i][oi](y, x)*scores_all[i][oi](y, x);
                        sq = sqrt(sq);
                        for ( int oi = 0; oi < no; ++ oi )
                            scores_all[i][oi](y,x) /= sq;
                    }
                }
            }
        }
        for ( int i = 0; i < (int)scores_all.size(); ++ i )
            for ( int oi = 0; oi < (int)scores_all[i].size(); ++ oi ){
                cout << dup_items_[oi].first << ": \n" << scores_all[i][oi] << "\n";
            }
        cout << "\n------------------------\n";
    }

    // analysis based on item number
    if ( target_bin_content_.size() == 1 ){
        cout << "only one item exists\n";
        // find the largest value in all the scores
        vector<cv::Point2i> pts;
        vector<float> maxs;
        for ( int i = 0; i < (int)scores_all.size(); ++ i ) {
            cv::Point2i pt;
            float max = scores_all[i][0].maxCoeff(&pt.y, &pt.x);
            pts.push_back( pt );
            maxs.push_back( max );
        }

        float max = -100.;
        int max_idx;
        for ( int i = 0; i < (int)maxs.size(); ++ i ) {
            if ( maxs[i] > max ) {
                max = maxs[i];
                max_idx = i;
            }
        }
        cv::Mat detect_rect( rgb_image_.rows, rgb_image_.cols, CV_8UC1, cv::Scalar(0) );

        if ( scores_all[max_idx][0].rows() != 1 || scores_all[max_idx][0].cols() != 1 ) {
            cv::Point tlpt;
            tlpt.x = blob_bbox[max_idx].first.x + pts[max_idx].x*step_size;
            tlpt.y = blob_bbox[max_idx].first.y + pts[max_idx].y*step_size;
            cv::rectangle( detect_rect, cv::Rect(tlpt.x, tlpt.y, patch_size, patch_size), cv::Scalar(255), CV_FILLED );
        }
        else {
            cv::rectangle( detect_rect, cv::Rect(blob_bbox[max_idx].first.x, blob_bbox[max_idx].first.y, blob_bbox[max_idx].second.x-blob_bbox[max_idx].first.x, blob_bbox[max_idx].second.y-blob_bbox[max_idx].first.y), cv::Scalar(255), CV_FILLED );
        }
        cv::bitwise_and( mask_image_, detect_rect, detect_rect );
        // get contour
        vector<cv::Point> contour = convex_hull( detect_rect, bin_depth_image );
        results.push_back( make_pair( target_bin_content_[0], contour ) );
        return;
    }
    else {
        float thresh_val;
        if ( dup_items_.size() == 1 ) {
            if ( blob_bbox.size() < dup_items_[0].second ) {
                cout << "multiple same items overlapped\n";
                for ( int i = 0; i < scores_all.size(); ++ i ) {
                    if ( scores_all[i].size() != 1 )
                        return;
                    MatrixXf &m = scores_all[i].front();
                    int r, c;
                    float maxval = m.maxCoeff( &r, &c );
                    cv::Point tlpt;
                    tlpt.x = blob_bbox[i].first.x + c*step_size;
                    tlpt.y = blob_bbox[i].first.y + r*step_size;
                    cv::Mat detect_rect( rgb_image_.rows, rgb_image_.cols, CV_8UC1, cv::Scalar(0) );
                    cv::rectangle( detect_rect, cv::Rect(tlpt.x, tlpt.y, patch_size, patch_size), cv::Scalar(255), CV_FILLED );
                    cv::bitwise_and( mask_image_, detect_rect, detect_rect );
                    // get contour
                    vector<cv::Point> contour = convex_hull( detect_rect, bin_depth_image );
                    results.push_back( make_pair( target_bin_content_[0], contour ) );
                }
                return;
            }
            else {
                thresh_val = -0.5;
                // check different patches
                vector< pair<float, pair<int, cv::Point2i> > > scores_per_obj;
                for ( int i = 0; i < (int)scores_all.size(); ++ i ) {
                    if ( scores_all[i].size() != 1 )
                        return;
                    cv::Point2i pt;
                    float s = scores_all[i][0].maxCoeff(&pt.y, &pt.x);
                    scores_per_obj.push_back( make_pair( s, make_pair(i, pt) ) );
                }
                sort(scores_per_obj.begin(), scores_per_obj.end(), boost::bind(&pair<float, pair<int, cv::Point2i> >::first, _1) > boost::bind(&pair<float, pair<int, cv::Point2i> >::first, _2));

                for ( int i = 0; i < dup_items_[0].second; ++ i ) {
                    if ( scores_per_obj[i].first > thresh_val ) {
                        int patch_idx = scores_per_obj[i].second.first;
                        cv::Point2i max_in_patch = scores_per_obj[i].second.second;
                        cv::Point tlpt;
                        tlpt.x = blob_bbox[patch_idx].first.x + max_in_patch.x*step_size;
                        tlpt.y = blob_bbox[patch_idx].first.y + max_in_patch.y*step_size;
                        cv::Mat detect_rect( rgb_image_.rows, rgb_image_.cols, CV_8UC1, cv::Scalar(0) );
                        cv::rectangle( detect_rect, cv::Rect(tlpt.x, tlpt.y, patch_size, patch_size), cv::Scalar(255), CV_FILLED );
                        cv::bitwise_and( mask_image_, detect_rect, detect_rect );
                        // get contour
                        vector<cv::Point> contour = convex_hull( detect_rect, bin_depth_image );
                        results.push_back( make_pair( target_bin_content_[0], contour ) );
                    }
                }
                return results;
            }
        }
        else {

            switch ( dup_items_.size() ) {
            case 2:
                thresh_val = -0.9;
                break;
            case 3:
                thresh_val = -0.75;
                break;
            default:
                thresh_val = -0.6;
                break;
            }

            // item name (patch index, score)
            vector<pair<string, pair<int, float> > > items_distribs;

            for ( int i = 0; i < (int)scores_all.size(); ++ i ) { // patch
                int ny = scores_all[i][0].rows();
                int nx = scores_all[i][0].cols();
                int no = scores_all[i].size();

                vector< pair<cv::Point2i, int> > indices;// the minimum value in the window is less than thresh

//                vector<int> count_per_obj( scores_all[i].size(), 0 );
//                vector< vector<cv::Point2i> > count_indices( scores_all[i].size() );
                for ( int y = 0; y < ny; ++ y ) {
                    for ( int x = 0; x < nx; ++ x ) {
                        float minval = 10.;
                        int minidx;
                        //! @brief add maximum value comparison
                        float maxval = -10.;
                        int maxidx;
                        for ( int oi = 0; oi < no; ++ oi ) {
                            if ( scores_all[i][oi](y,x) < minval ) {
                                minval = scores_all[i][oi](y,x);
                                minidx = oi;// object index
                            }
                            if ( scores_all[i][oi](y,x) > maxval ) {
                                maxval = scores_all[i][oi](y,x);
                                maxidx = oi;
                            }
                        }

//                        count_indices[maxidx].push_back( cv::Point2i(x, y) );
//                        count_per_obj[maxidx] ++;

                        if ( minval < thresh_val || maxval > 0.0 ) {
                            indices.push_back( make_pair(cv::Point2i(x, y), minidx) );
                        }
                    }
                }

//                indices.clear();
//                int maxcount_pos = std::distance( count_per_obj.begin(), std::max_element( count_per_obj.begin(), count_per_obj.end() ) );
//                for ( int ic = 0; ic < count_indices[maxcount_pos].size(); ++ ic ) {
//                    indices.push_back( make_pair( count_indices[maxcount_pos][ic], maxcount_pos) );
//                }

//                if ( display_ == true ) {
//                    for ( int ic = 0; ic < count_per_obj.size(); ++ ic )
//                        cout << dup_items_[ic].first << ": " << count_per_obj[ic] << endl;
//                    cout << "\n-------------------------------\n";
//                }

                if ( !indices.empty() ) {
                    vector< vector<cv::Point> > indices_per_obj( dup_items_.size() );
                    for ( int j = 0; j < (int)indices.size(); ++ j ) {
                        cv::Point2i pos = indices[j].first;
                        float maxval = -10.;
                        int maxidx;
                        for ( int oi = 0; oi < no; ++ oi ) {
                            // find the largest one in the window
                            if ( scores_all[i][oi](pos.y, pos.x) > maxval ) {
                                maxval = scores_all[i][oi](pos.y, pos.x);
                                maxidx = oi;
                            }
                        }
                        indices_per_obj[maxidx].push_back( indices[j].first );
                    }



                    for ( int j = 0; j < (int)dup_items_.size(); ++ j ) {
                        // put the scores which is less than thresh to array
                        if ( !indices_per_obj[j].empty() ) {
                            vector< pair<float, cv::Point2i > > lscores_per_obj;
                            for ( int k = 0; k < (int)indices_per_obj[j].size(); ++ k ) {
                                lscores_per_obj.push_back( make_pair(scores_all[i][j](indices_per_obj[j][k].y, indices_per_obj[j][k].x), indices_per_obj[j][k] ));
                            }
                            sort(lscores_per_obj.begin(), lscores_per_obj.end(), boost::bind(&pair<float, cv::Point2i>::first, _1) > boost::bind(&pair<float, cv::Point2i>::first, _2));


                            // find the top n == dup_items_[j].second
                            int n_items = dup_items_[j].second > lscores_per_obj.size()? lscores_per_obj.size(): dup_items_[j].second;
                            for ( int k = 0; k < n_items; ++ k ) {
                                if ( scores_all[i][j].rows() != 1 || scores_all[i][j].cols() != 1 ) {
                                    cv::Point tlpt;
                                    tlpt.x = blob_bbox[i].first.x+lscores_per_obj[k].second.x*step_size;
                                    tlpt.y = blob_bbox[i].first.y+lscores_per_obj[k].second.y*step_size;
                                    cv::Mat detect_rect( rgb_image_.rows, rgb_image_.cols, CV_8UC1, cv::Scalar(0) );
                                    cv::rectangle( detect_rect, cv::Rect(tlpt.x, tlpt.y, patch_size, patch_size), cv::Scalar(255), CV_FILLED );
                                    cv::bitwise_and( mask_image_, detect_rect, detect_rect );
                                    // get contour
                                    vector<cv::Point> contour = convex_hull( detect_rect, bin_depth_image );
                                    if ( !contour.empty() ) {
                                        results.push_back( make_pair( dup_items_[j].first, contour ) );
                                        items_distribs.push_back( make_pair(dup_items_[j].first, make_pair( i, lscores_per_obj[k].first ) ) );
                                    }
                                }
                                else {
                                    cv::Mat detect_rect( rgb_image_.rows, rgb_image_.cols, CV_8UC1, cv::Scalar(0) );
                                    cv::rectangle( detect_rect, cv::Rect( blob_bbox[i].first.x, blob_bbox[i].first.y, blob_bbox[i].second.x-blob_bbox[i].first.x, blob_bbox[i].second.y-blob_bbox[i].first.y), cv::Scalar(255), CV_FILLED );
                                    cv::bitwise_and( mask_image_, detect_rect, detect_rect );
                                    // get contour
                                    vector<cv::Point> contour = convex_hull( detect_rect, bin_depth_image );
                                    if ( !contour.empty() ) {
                                        results.push_back( make_pair( dup_items_[j].first, contour ) );
                                        items_distribs.push_back( make_pair(dup_items_[j].first, make_pair( i, lscores_per_obj[k].first ) ) );
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // find the largest one among patches
            vector<pair<string, vector<cv::Point> > > new_results;

            vector<string> err_items;
            map<string, int> n_results;
            cout << "remove dup results\n";
            for ( int ires = 0; ires < (int)items_distribs.size(); ++ ires ) {
                cout << items_distribs[ires].first << "\n";
                if ( n_results.find( items_distribs[ires].first ) == n_results.end() )
                    n_results[items_distribs[ires].first] = 1;
                else
                    n_results[items_distribs[ires].first] += 1;
            }
            for ( int ibc = 0; ibc < (int)dup_items_.size(); ++ ibc ) {
                if ( n_results[dup_items_[ibc].first] > dup_items_[ibc].second ) {
                    err_items.push_back( dup_items_[ibc].first );
                }
            }
            if ( !err_items.empty() ) {
                // find the results of err item
                for ( int ei = 0; ei < (int)err_items.size(); ++ ei ) {
                    cout << err_items[ei] << endl;
                    vector<int> idxs; // idxs for wrong results
                    for ( int ii = 0; ii < (int)items_distribs.size(); ++ ii ) {
                        if ( items_distribs[ii].first == err_items[ei] )
                            idxs.push_back( ii );
                    }
                    map<int, pair<float, int> > cor_results;
                    for ( int ii = 0; ii < (int)idxs.size(); ++ ii ) {
                        if ( cor_results.find( items_distribs[idxs[ii]].second.first ) == cor_results.end() ) { // patch dose not exists
                            cor_results[items_distribs[idxs[ii]].second.first] = make_pair( items_distribs[idxs[ii]].second.second, idxs[ii] );
                        }
                        else {
                            if ( cor_results[items_distribs[idxs[ii]].second.first].first < items_distribs[idxs[ii]].second.second )
                                cor_results[items_distribs[idxs[ii]].second.first] = make_pair( items_distribs[idxs[ii]].second.second, idxs[ii] );
                        }
                    }
                    int optn;
                    for ( int ii = 0; ii < (int)dup_items_.size(); ++ ii )
                        if ( dup_items_[ii].first == err_items[ei] )
                            optn = dup_items_[ii].second;
                    vector< pair<float, pair<int, int> > > scores_dup;
                    typedef map<int, pair<float, int> >::iterator it_type;
                    for( map<int, pair<float, int> >::iterator iterator = cor_results.begin(); iterator != cor_results.end(); iterator++) {
                        scores_dup.push_back( make_pair( iterator->second.first, make_pair(iterator->first, iterator->second.second) ) );

                    }
                    sort( scores_dup.begin(), scores_dup.end(), boost::bind(&pair<float, pair<int, int> >::first, _1) > boost::bind(&pair<float, pair<int, int> >::first, _2) );
                    int n_item = optn > cor_results.size()? cor_results.size(): optn;
                    for ( int ii = 0; ii < n_item; ++ ii ) {
                        int idx = scores_dup[ii].second.second;
                        new_results.push_back( results[idx] );
                    }
                }
            }

            // put correct items to the new results
            for ( int ir = 0; ir < (int)results.size(); ++ ir ) {
                bool found = false;
                for ( int ie = 0; ie < (int)err_items.size(); ++ ie )
                    if ( err_items[ie] == results[ir].first )
                        found = true;
                if ( found == false )
                    new_results.push_back( results[ir] );
            }
            results.clear();
            for ( int in = 0; in < (int)new_results.size(); ++ in ) {
                results.push_back( new_results[in] );
            }
            new_results.clear();
            return;
        }
    }
}


//! process
void KDRecogniser::process(vector<pair<string, vector<cv::Point> > > & results, bool use_rgb) {
    results.clear();
    // detailed mask image generation using subtraction
    // rgb image to gray scale image conversion
    int threshold = 50;

    // generate subtracted image using rgb information
    sub_image_.setTo( cv::Scalar(0) );
    if ( !use_rgb ) {
        cv::Mat empty_image_mono;
        cv::cvtColor( empty_image_, empty_image_mono, CV_BGR2GRAY );

        cv::Mat item_image_mono;
        cv::cvtColor( rgb_image_, item_image_mono, CV_BGR2GRAY );

        cv::Mat diff_image_mono;
        cv::absdiff( empty_image_mono, item_image_mono, diff_image_mono );
        cv::threshold( diff_image_mono, sub_image_, threshold, 255.0, CV_THRESH_BINARY );
    }
    else {
        vector<cv::Mat> empty_image_rgbs;
        cv::split( empty_image_, empty_image_rgbs );
        vector<cv::Mat> item_image_rgbs;
        cv::split( rgb_image_, item_image_rgbs );
        vector<cv::Mat> diff_image_rgbs;
        for ( int i = 0; i < 3; i ++ ) {
            cv::Mat diff_image_rgb;
            cv::absdiff( empty_image_rgbs[i], item_image_rgbs[i], diff_image_rgb );
            diff_image_rgbs.push_back( diff_image_rgb );
            cv::threshold( diff_image_rgbs[i], diff_image_rgbs[i], threshold, 255.0, CV_THRESH_BINARY );
        }
        cv::bitwise_or( diff_image_rgbs[0], diff_image_rgbs[1], diff_image_rgbs[1] );
        cv::bitwise_or( diff_image_rgbs[1], diff_image_rgbs[2], sub_image_ );
    }

    // find blobs in the subtracted images and find the bbox for each blob
    vector< vector<cv::Point2i> > blobs;
    find_blobs( sub_image_, blobs );
    vector< pair<cv::Point2i, cv::Point2i> > blob_bbox;
    cv::Mat mask_bbox( sub_image_.rows, sub_image_.cols, CV_8UC1, cv::Scalar::all(0) );
    for ( int i = 0; i < (int)blobs.size(); ++ i ) {        
        pair<cv::Point2i, cv::Point2i> bbox; // min, max
        bbox.first.x = rgb_image_.cols;
        bbox.first.y = rgb_image_.rows;
        bbox.second.x = 0;
        bbox.second.y = 0;
        for ( int j = 0; j < (int)blobs[i].size(); ++ j ) {
            if ( blobs[i][j].x < bbox.first.x )
                bbox.first.x = blobs[i][j].x;
            if ( blobs[i][j].y < bbox.first.y )
                bbox.first.y = blobs[i][j].y;
            if ( blobs[i][j].x > bbox.second.x )
                bbox.second.x = blobs[i][j].x;
            if ( blobs[i][j].y > bbox.second.y )
                bbox.second.y = blobs[i][j].y;
            cv::Point2i & pt = blobs[i][j];
            mask_bbox.at<uchar>(pt.y, pt.x) = (unsigned char)255;
        }
    }

    // generated the new mask image by bitwise and
    cv::bitwise_and( this->mask_image_, mask_bbox, this->mask_image_ );

    // remove small bbox
    find_blobs( this->mask_image_, blobs );
    blob_bbox.clear();
    mask_bbox = cv::Mat( sub_image_.rows, sub_image_.cols, CV_8UC1, cv::Scalar::all(0) );
    for ( int i = 0; i < (int)blobs.size(); ++ i ) {
        pair<cv::Point2i, cv::Point2i> bbox; // min, max
        bbox.first.x = sub_image_.cols;
        bbox.first.y = sub_image_.rows;
        bbox.second.x = 0;
        bbox.second.y = 0;
        for ( int j = 0; j < (int)blobs[i].size(); ++ j ) {
            if ( blobs[i][j].x < bbox.first.x )
                bbox.first.x = blobs[i][j].x;
            if ( blobs[i][j].y < bbox.first.y )
                bbox.first.y = blobs[i][j].y;
            if ( blobs[i][j].x > bbox.second.x )
                bbox.second.x = blobs[i][j].x;
            if ( blobs[i][j].y > bbox.second.y )
                bbox.second.y = blobs[i][j].y;
        }
        // minima size for subtracted images
        if ( bbox.second.y-bbox.first.y > 32 && bbox.second.x-bbox.first.x > 32 ) {
            blob_bbox.push_back( bbox );
            cv::rectangle( mask_bbox, bbox.first, bbox.second, cv::Scalar::all(255));
        }
    }


    // generate indices for items
    vector<string> svm_models_list = *(this->swd_->get_models_list());
    dup_items_.clear();
    dup_items_ = duplicated_bin_contents( this->target_bin_content_ );
    vector<int> content_in_svm_indices;     // content item indices in svm model
    for ( int i = 0; i < (int)dup_items_.size(); ++ i ) {
        int pos = find(svm_models_list.begin(), svm_models_list.end(), dup_items_[i].first) - svm_models_list.begin();
        cout << "Name: " << dup_items_[i].first << ", " << pos << endl;
        content_in_svm_indices.push_back( pos );
    }


    // 1st vector: each item, 2 nd vector each patch
    vector< vector<MatrixXf> > scores_for_contents( dup_items_.size() );
    int step_size = 16;
    int patch_size = 64;
    IplImage * image = new IplImage(this->rgb_image_);

    double exec_time;
    Timer timer;
    timer.start();

    for ( int i = 0; i < (int)blob_bbox.size(); ++ i ) {    // for each patch
        CvRect roi = cvRect( blob_bbox[i].first.x, blob_bbox[i].first.y, blob_bbox[i].second.x-blob_bbox[i].first.x, blob_bbox[i].second.y-blob_bbox[i].first.y );
        cvSetImageROI(image, roi);
        IplImage * patch = cvCreateImage( cvSize(roi.width, roi.height), image->depth, image->nChannels );
        cvCopy(image, patch);
        if ( patch->height < patch_size || patch->width < patch_size ) {
            KernelDescManager * pkdm = swd_->get_pkdm();
            MatrixXf imfea;
            VectorXf score;
            pkdm->Process( imfea, patch );
            pkdm->Classify( score, imfea );
            VectorXf item_score(content_in_svm_indices.size());
            for ( int y = 0; y < item_score.rows(); ++ y ) {
                item_score(y,0) = score(content_in_svm_indices[y],0);
            }
            for ( int j = 0; j < (int)dup_items_.size(); ++ j ) {
                MatrixXf score(1,1);
                score << item_score(j, 0);
                scores_for_contents[j].push_back(score);
            }
        }
        else {
            vector<MatrixXf> scores = swd_->process( patch, content_in_svm_indices, patch_size, step_size );
            for ( int j = 0; j < (int)scores.size(); ++ j ) {
                scores_for_contents[j].push_back( scores[j] );
            }
        }
    }
    exec_time = timer.get();
    cout << "Execution time: " << setw(8) << exec_time << endl;


    for ( int i = 0; i < (int)scores_for_contents.front().size(); ++ i ) {
        for ( int y = 0; y < scores_for_contents.front()[i].rows(); ++ y ) {
            for ( int x = 0; x < scores_for_contents.front()[i].cols(); ++ x ) {
                float sq = 0.0;
                for ( int j = 0; j < (int)scores_for_contents.size(); ++ j ) {
                    cout << scores_for_contents[j][i](y,x) << " ";
                    sq += scores_for_contents[j][i](y,x)*scores_for_contents[j][i](y,x);
                }
                cout << " | ";
                sq = sqrt(sq);
                for ( int j = 0; j < (int)scores_for_contents.size(); ++ j ) {
                    cout << scores_for_contents[j][i](y,x)/sq << " ";
                }

                cout << "\n";
            }
        }
        cout << "\n--\n";
    }
    cout << "\n=====================\n";

    //! 2nd maximum suppression
    /*
    if ( scores_for_contents.size() > 1 ) {
        for ( int pi = 0; pi < (int)scores_for_contents.front().size(); ++ pi ){
            // score for different item in one patch
            vector<MatrixXf> scores_patch;
            for ( int oi = 0; oi < (int)scores_for_contents.size(); ++ oi ) {
                scores_patch.push_back( scores_for_contents[oi][pi] );
            }

            int ny = scores_patch.front().rows();
            int nx = scores_patch.front().cols();
            vector< pair<float, int> > scores_items;
            for ( int y = 0; y < ny; ++ y ) {
                for ( int x = 0; x < nx; ++ x ) {
                    // put score for different item in one window together
                    for ( int oi = 0; oi < (int)scores_patch.size(); ++ oi ) {
                        scores_items.push_back( make_pair(scores_patch[oi](y,x), oi) );
                    }
                    // sort the local window score for different item
                    sort(scores_items.begin(), scores_items.end(), boost::bind(&std::pair<float, int>::first, _1) > boost::bind(&std::pair<float, int>::first, _2));

                    // find out the difference between the largest and the second largest score
                    // for different items in the same window
                    scores_for_contents[scores_items[0].second][pi](y,x) -= scores_items[1].first;
                    for ( int oi = 1; oi < (int)scores_patch.size(); ++ oi )
                        scores_for_contents[scores_items[oi].second][pi](y,x) -= scores_items[0].first;
//                    for ( int oi = 0; oi < (int)scores_patch.size(); ++ oi )
//                        scores_for_contents[ scores_items[oi].second ][pi](y,x) -= scores_items[1].first;
                    scores_items.clear();
                }
            }
        }
    }

    for ( int i = 0; i < (int)scores_for_contents.front().size(); ++ i ) {
        for ( int y = 0; y < scores_for_contents.front()[i].rows(); ++ y ) {
            for ( int x = 0; x < scores_for_contents.front()[i].cols(); ++ x ) {
                for ( int j = 0; j < (int)scores_for_contents.size(); ++ j ) {
                    cout << scores_for_contents[j][i](y,x) << " ";
                }
                cout << "\n";
            }
        }
        cout << "\n--\n";
    }
    */

    // convert depth image in online mode and offline mode
    if ( this->depth_image_.type() != 5 ) {
        cv::Mat new_depth_image( this->depth_image_.rows, this->depth_image_.cols, CV_32FC1 );
        for ( int y = 0; y < this->depth_image_.rows; ++ y )
            for ( int x = 0; x < this->depth_image_.cols; ++ x )
                new_depth_image.at<float>(y,x) = static_cast<float>(this->depth_image_.at<unsigned short>(y,x));
        this->depth_image_.release();
        this->depth_image_ = new_depth_image.clone();
        new_depth_image.release();
    }


    // generate binary image from depth image subtraction
    cv::Mat bin_depth_image( this->depth_image_.rows, this->depth_image_.cols, CV_8UC1, cv::Scalar(0) );
    for ( int y = 0; y < bin_depth_image.rows; ++ y )
        for ( int x = 0; x < bin_depth_image.cols; ++ x )
            if ( this->depth_image_.at<float>(y,x) > 0 )
                if ( abs(this->depth_image_.at<float>(y,x)-
                         this->empty_depth_.at<unsigned short>(y,x)) > 3 )
                    bin_depth_image.at<uchar>(y,x) = static_cast<unsigned char>(255);



    // items-windows(score, window-tl point)
    vector< vector<pair<float, cv::Point2i> > > scores_pt( scores_for_contents.size() );
    for ( int i = 0; i < (int)scores_for_contents.size(); ++ i ) {
        for ( int j = 0; j < (int)scores_for_contents[i].size(); ++ j ) {
            MatrixXf score = scores_for_contents[i][j];
            cv::Point2i startpt = blob_bbox[j].first;

            for ( int y = 0; y < score.rows(); ++ y ) {
                for ( int x = 0; x < score.cols(); ++ x ) {
                    cv::Point2i pt;// pt in image space
                    pt.x = startpt.x + x*step_size;
                    pt.y = startpt.y + y*step_size;
                    scores_pt[i].push_back( make_pair(score(y,x), pt) );
                }
            }
        }

        sort(scores_pt[i].begin(), scores_pt[i].end(),
             boost::bind(&pair<float, cv::Point2i>::first, _1) > boost::bind(&pair<float, cv::Point2i>::first, _2));
        for ( int ii = 0; ii < scores_pt[i].size(); ++ ii )
            cout << scores_pt[i][ii].first << " ";
        cout << endl;
        // select top n items according to objects no. in this bin
        int n_objecti = dup_items_[i].second;
        for ( int j = 0; j < n_objecti; ++ j ) {
            string object_name = dup_items_[i].first;
            vector<cv::Point> pts;
            pts.push_back( cv::Point(scores_pt[i][j].second.x, scores_pt[i][j].second.y) );
            pts.push_back( cv::Point(scores_pt[i][j].second.x, scores_pt[i][j].second.y+patch_size) );
            pts.push_back( cv::Point(scores_pt[i][j].second.x+patch_size, scores_pt[i][j].second.y+patch_size) );
            pts.push_back( cv::Point(scores_pt[i][j].second.x+patch_size, scores_pt[i][j].second.y) );

            // draw rectangle on image
            cv::Mat detected_rect(this->rgb_image_.rows, this->rgb_image_.cols, CV_8UC1, cv::Scalar(0));
            cv::rectangle( detected_rect, cv::Rect(scores_pt[i][j].second.x, scores_pt[i][j].second.y, patch_size, patch_size), cv::Scalar(255), CV_FILLED );
            cv::bitwise_and( this->mask_image_, detected_rect, detected_rect );

            // use depth image to provide the patch which has depth informaiton
            vector<cv::Point> convex_pts;
            if ( !this->empty_depth_.empty() && !this->depth_image_.empty() ) {
                convex_pts.clear();
                cv::bitwise_and( detected_rect, bin_depth_image, detected_rect );
                vector< vector<cv::Point2i> > blobs;
                find_blobs( detected_rect, blobs);
                int max_blob_idx;
                int max_blob_sz = 0;
                for ( int ii = 0; ii < (int)blobs.size(); ++ ii ) {
                    if ( (int)blobs[ii].size() > max_blob_sz ) {
                        max_blob_idx = ii;
                        max_blob_sz = (int)blobs[ii].size();
                    }
                }

                detected_rect = cv::Mat( detected_rect.rows, detected_rect.cols, CV_8UC1, cv::Scalar(0) );
                for ( int ii = 0; ii < (int)blobs[max_blob_idx].size(); ++ ii )
                    detected_rect.at<uchar>( blobs[max_blob_idx][ii].y, blobs[max_blob_idx][ii].x )
                            = static_cast<unsigned char>(255);

                // simple edge detector
                vector<cv::Point> contour;
                for ( int y = 1; y < detected_rect.rows-1; ++ y )
                    for ( int x = 1; x < detected_rect.cols-1; ++ x )
                        if ( detected_rect.at<uchar>(y,x) > 0 )
                            if ( detected_rect.at<uchar>(y+1,x) == 0 ||
                                 detected_rect.at<uchar>(y-1,x) == 0 ||
                                 detected_rect.at<uchar>(y,x+1) == 0 ||
                                 detected_rect.at<uchar>(y,x-1) == 0)
                                contour.push_back( cv::Point(x, y) );
                convex_pts = get_convex_hull( contour );
            }
            results.push_back( make_pair(object_name, convex_pts) );
        }
    }
    cv::imshow("sub_image", sub_image_);
    cv::imshow("mask_image_", mask_image_);
    cv::imshow("bin_depth_image", bin_depth_image);
    cv::waitKey(0);

}


cv::Mat KDRecogniser::get_mask_image() {
    return mask_image_;
}



vector<pair<string, int> > KDRecogniser::get_dup_items() {
    return dup_items_;
}
