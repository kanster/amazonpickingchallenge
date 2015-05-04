/** kernel descriptor */

#include "include/helpfun/kd_recogniser.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <opencv2/imgproc/imgproc.hpp>

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
    int n_stepsx = floor((ipl_image->width-patch_size)*1.0/step_size)+1;
    int n_stepsy = floor((ipl_image->height-patch_size)*1.0/step_size)+1;
//    cout << "image size: " << ipl_image->width << ", " << ipl_image->height << endl;
//    cout << "step sizes: " << n_stepsx << ", " << n_stepsy << endl;

    vector<MatrixXf> all_scores( (int)pkdm_->GetModelList()->size(), MatrixXf::Zero(n_stepsy, n_stepsx) ); // item scores for all items
    double exec_time;
    Timer timer;
    timer.start();
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

            assert( scores.rows() == (int)pkdm_->GetModelList()->size() );
            for ( int i = 0; i < (int)pkdm_->GetModelList()->size(); ++ i ) {
                all_scores[i](iy, ix) = scores(i);
            }

        }
    }
    exec_time = timer.get();
    cout << "Execution time: " << setw(8) << exec_time << "    " << endl;

    /*
    CvFont font;
    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.6, 0.6, 0, 1);

    cv::namedWindow( "scores" );
    for ( int i = 0; i < (int)pkdm_->GetModelList()->size(); ++ i ) {
        cout << (*pkdm_->GetModelList())[i] << " : ";
        cv::Mat score_img = from_score( all_scores[i], 32 );
        string object_name = (*pkdm_->GetModelList())[i];
        cv::putText(score_img, object_name, cvPoint(30,30), CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.6 ,cvScalar(255,255,255));
        cv::imshow( "scores", score_img );
        cv::waitKey(0);
    }
    */
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


void KDRecogniser::find_blobs(const cv::Mat &binary, vector<vector<cv::Point2i> > &blobs) {
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
}

// visualise matrixxf scores
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


// load image and point cloud
KDRecogniser::KDRecogniser(){
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


// set target item and neighboured items
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



// load empty information
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



// init kernel descriptor
void KDRecogniser::init_libkdes(string svm_model_name, string kdes_model_name, string model_folder, string model, unsigned int model_type){
    svm_model_name_ = svm_model_name;
    kdes_model_name_ = kdes_model_name;
    model_folder_ = model_folder;
    model_ = model;
    model_type_ = model_type;
    swd_ = new SlidingWindowDetector( model_, svm_model_name_, kdes_model_name_, model_folder_, model_type_ );

}


// process
void KDRecogniser::process(vector<pair<string, vector<cv::Point> > > & results, bool use_rgb) {
    results.clear();
    // detailed mask image generation using subtraction
    // rgb image to gray scale image conversion
    int threshold = 20;
    cv::Mat sub_image;
    if ( !use_rgb ) {
        cv::Mat empty_image_mono;
        cv::cvtColor( empty_image_, empty_image_mono, CV_BGR2GRAY );

        cv::Mat item_image_mono;
        cv::cvtColor( rgb_image_, item_image_mono, CV_BGR2GRAY );

        cv::Mat diff_image_mono;
        cv::absdiff( empty_image_mono, item_image_mono, diff_image_mono );
        cv::threshold( diff_image_mono, sub_image, threshold, 255.0, CV_THRESH_BINARY );
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
        cv::bitwise_or( diff_image_rgbs[1], diff_image_rgbs[2], sub_image );
    }

//    cv::imshow( "sub_image", sub_image );


    // find blobs
    vector< vector<cv::Point2i> > blobs;
    find_blobs( sub_image, blobs );
    vector< pair<cv::Point2i, cv::Point2i> > blob_bbox;
    cv::Mat mask_bbox( sub_image.rows, sub_image.cols, CV_8UC1, cv::Scalar::all(0) );
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

    // add with mask image of the BIN
    cv::bitwise_and( this->mask_image_, mask_bbox, this->mask_image_ );

//    cv::imshow( "mask_image", this->mask_image_ );


    // remove small bbox
    find_blobs( this->mask_image_, blobs );
    blob_bbox.clear();
    mask_bbox = cv::Mat( sub_image.rows, sub_image.cols, CV_8UC1, cv::Scalar::all(0) );
    for ( int i = 0; i < (int)blobs.size(); ++ i ) {
        pair<cv::Point2i, cv::Point2i> bbox; // min, max
        bbox.first.x = sub_image.cols;
        bbox.first.y = sub_image.rows;
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
        if ( bbox.second.y-bbox.first.y > 32 || bbox.second.x-bbox.first.x > 32 ) {
            blob_bbox.push_back( bbox );
            // draw rectangle bbox
            cv::rectangle( mask_bbox, bbox.first, bbox.second, cv::Scalar::all(255));
        }
    }
    cv::imshow( "rectangle", mask_bbox );



    // generate indices for items
    // retrieve item classes
    vector<string> svm_models_list = *(this->swd_->get_models_list());
    vector<pair<string, int> > objects_n = duplicated_bin_contents( this->target_bin_content_ );

    vector<int> content_in_svm_indices;
    for ( int i = 0; i < (int)objects_n.size(); ++ i ) {
        int pos = find(svm_models_list.begin(), svm_models_list.end(), objects_n[i].first) - svm_models_list.begin();
        cout << "Name: " << objects_n[i].first << ", " << pos << endl;
        content_in_svm_indices.push_back( pos );
    }


    // 1st vector: each item, 2 nd vector each patch
    vector< vector<MatrixXf> > scores_for_contents( objects_n.size() );
    int step_size = 16;
    int patch_size = 64;
    IplImage * image = new IplImage(this->rgb_image_);
    // for loop for each patch
    for ( int i = 0; i < (int)blob_bbox.size(); ++ i ) {
        CvRect roi = cvRect( blob_bbox[i].first.x, blob_bbox[i].first.y, blob_bbox[i].second.x-blob_bbox[i].first.x, blob_bbox[i].second.y-blob_bbox[i].first.y );
        cvSetImageROI(image, roi);
        IplImage * patch = cvCreateImage( cvSize(roi.width, roi.height), image->depth, image->nChannels );
        cvCopy(image, patch);
        // object detector
        if ( patch->height < 64 || patch->width < 64 ) {
            KernelDescManager * pkdm = swd_->get_pkdm();
            MatrixXf imfea;
            VectorXf score;
            pkdm->Process( imfea, patch );
            pkdm->Classify( score, imfea );
            VectorXf item_score(content_in_svm_indices.size());
            for ( int y = 0; y < item_score.rows(); ++ y ) {
                item_score(y,0) = score(content_in_svm_indices[y],0);
            }
            for ( int j = 0; j < (int)objects_n.size(); ++ j ) {
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

    /*
    for ( int i = 0; i < (int)scores_for_contents.size(); ++ i ) {
        for ( int j = 0; j < (int)scores_for_contents[i].size(); ++ j ) {
            MatrixXf score = scores_for_contents[i][j];
            cout << score << "\n\n";
            cv::namedWindow( "score" );
            cv::Mat score_img = from_score( score, 32 );
            cv::imshow( "score", score_img );
            cv::waitKey(0);
        }
    }
    */


    for ( int pi = 0; pi < (int)scores_for_contents.front().size(); ++ pi ){
        vector<MatrixXf> scores_patch;
        for ( int oi = 0; oi < (int)scores_for_contents.size(); ++ oi ) {
            scores_patch.push_back( scores_for_contents[oi][pi] );
        }
        // check the size of the scores should be the same
        int ny = scores_patch.front().rows();
        int nx = scores_patch.front().cols();
        vector< pair<float, int> > scores_items;
        for ( int y = 0; y < ny; ++ y ) {
            for ( int x = 0; x < nx; ++ x ) {
                for ( int oi = 0; oi < (int)scores_patch.size(); ++ oi ) {
                    scores_items.push_back( make_pair(scores_patch[oi](y,x), oi) );
                }
                sort(scores_items.begin(), scores_items.end(), boost::bind(&std::pair<float, int>::first, _1) > boost::bind(&std::pair<float, int>::first, _2));
                // reassign score to the largest in the remaining largest
                scores_for_contents[scores_items[0].second][pi](y,x) -= scores_items[1].first;
                for ( int oi = 1; oi < (int)scores_patch.size(); ++ oi )
                    scores_for_contents[scores_items[oi].second][pi](y,x) -= scores_items[0].first;
                scores_items.clear();
            }
        }
    }


    // find maximum response in each item
    // 1st vector: each item, 2nd vector: each patch

    // convert depth image into binary image

    if ( this->depth_image_.type() != 5 ) {
        cv::Mat new_depth_image( this->depth_image_.rows, this->depth_image_.cols, CV_32FC1 );
        for ( int y = 0; y < this->depth_image_.rows; ++ y )
            for ( int x = 0; x < this->depth_image_.cols; ++ x )
                new_depth_image.at<float>(y,x) = static_cast<float>(this->depth_image_.at<unsigned short>(y,x));
        this->depth_image_.release();
        this->depth_image_ = new_depth_image.clone();
        new_depth_image.release();
    }

    cv::Mat bin_depth_image( this->depth_image_.rows, this->depth_image_.cols, CV_8UC1, cv::Scalar(0) );
    for ( int y = 0; y < bin_depth_image.rows; ++ y )
        for ( int x = 0; x < bin_depth_image.cols; ++ x )
            if ( this->depth_image_.at<float>(y,x) > 0 )
                if ( abs(this->depth_image_.at<float>(y,x)-
                         this->empty_depth_.at<unsigned short>(y,x)) > 3 )
                    bin_depth_image.at<uchar>(y,x) = static_cast<unsigned char>(255);
//    cv::imshow( "bin_depth_image", bin_depth_image );

    vector< vector<pair<float, cv::Point2i> > > scores_pt( scores_for_contents.size() );
    for ( int i = 0; i < (int)scores_for_contents.size(); ++ i ) {
        for ( int j = 0; j < (int)scores_for_contents[i].size(); ++ j ) {
            MatrixXf score = scores_for_contents[i][j];
            cv::Point2i startpt = blob_bbox[j].first;
//            cv::namedWindow( "score" );
//            cv::Mat score_img = from_score( score, 32 );
//            cv::imshow( "score", score_img );
//            cv::waitKey(0);

//            cout << "score: " << score.rows() << ", " << score.cols() << endl;
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

        // select top n items according to objects no. in this bin
        int n_objecti = objects_n[i].second;
        for ( int j = 0; j < n_objecti; ++ j ) {
            string object_name = objects_n[i].first;
            vector<cv::Point> pts;
            pts.push_back( cv::Point(scores_pt[i][j].second.x, scores_pt[i][j].second.y) );
            pts.push_back( cv::Point(scores_pt[i][j].second.x, scores_pt[i][j].second.y+patch_size) );
            pts.push_back( cv::Point(scores_pt[i][j].second.x+patch_size, scores_pt[i][j].second.y+patch_size) );
            pts.push_back( cv::Point(scores_pt[i][j].second.x+patch_size, scores_pt[i][j].second.y) );

            // draw rectangle on image
            cv::Mat detected_rect(this->rgb_image_.rows, this->rgb_image_.cols, CV_8UC1, cv::Scalar(0));
            cv::rectangle( detected_rect, cv::Rect(scores_pt[i][j].second.x, scores_pt[i][j].second.y, patch_size, patch_size), cv::Scalar(255), CV_FILLED );
            cv::bitwise_and( this->mask_image_, detected_rect, detected_rect );

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
}

