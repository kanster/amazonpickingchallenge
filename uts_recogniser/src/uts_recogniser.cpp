#include "include/uts_recogniser.h"
#include <boost/bind.hpp>
#include <unistd.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <iterator>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

#include <pcl/console/parse.h>

#define foreach( i, c ) for( typeof((c).begin()) i##_hid=(c).begin(), *i##_hid2=((typeof((c).begin())*)1); i##_hid2 && i##_hid!=(c).end(); ++i##_hid) for( typeof( *(c).begin() ) &i=*i##_hid, *i##_hid3=(typeof( *(c).begin() )*)(i##_hid2=NULL); !i##_hid3 ; ++i##_hid3, ++i##_hid2)

const string g_rgb_win_name     = "rgb_image";
const string g_mask_win_name    = "mask_image";

// constructor
OfflineRecogniser::OfflineRecogniser(ros::NodeHandle &nh)
    : exit_flag_(false)
    , sensor_empty_(true)
    , target_received_(false)
    , target_count_(0)
    , recogniser_done_(true)
    , image_captured_(true)
    , debug_(true){
    nh_ = & nh;
    nh_->param<std::string>("json", json_filename_, "/tmp/apc/amazon.json");
    nh_->param<std::string>("method", method_path_, "/tmp/apc/method.txt");
    nh_->param<std::string>("kd_dir", kd_dir_, "/tmp/apc/kd/");
    nh_->param<std::string>("eb_dir", eb_dir_, "/tmp/apc/eb/");
    nh_->param<std::string>("eb_temp_conf", temp_conf_path_, "/tmp/apc/eb/temp.conf");
    nh_->param<std::string>("xml_dir", xml_dir_, "/tmp/apc/xml/");
    nh_->param<std::string>("mask_dir", mask_dir_, "/tmp/apc/mask/");
    nh_->param<bool>("use_cloud", use_cloud_, false);

    nh_->param<std::string>("object_topic_name", object_topic_name_, "/object_poses");
    nh_->param<std::string>("object_srv_name", object_srv_name_, "/recog_publish_srv");
    nh_->param<std::string>("target_srv_name", target_srv_name_, "/data_publish_srv");

    nh_->param<int>("op_mode", op_mode_, 1);
    nh_->param<int>("srv_mode", srv_mode_, 1);

    sensor_data_ptr_ = &sensor_data_;
    load_method_config( method_path_ );

    string svm_model_name = "modelrgbkdes";
    string kdes_model_name = "kpcaRGBDes";
    string model_folder = string(kd_dir_);
    string model = "RGB Kernel Descriptor";
    unsigned int model_type = 2;
    kdr_.init_libkdes( svm_model_name, kdes_model_name, model_folder, model, model_type );
    ROS_INFO( "Initializing kernel descriptor recogniser" );

    //! init eblearn recogniser
    ebr_.init( temp_conf_path_, eb_dir_ );


    if ( srv_mode_ == 1 ) {
        // resolve topic name
        nh_->param<std::string>("xtion_rgb_image", xtion_rgb_topic_, "/camera/lowres_rgb/image");
        nh_->param<std::string>("xtion_depth_image", xtion_depth_topic_, "/camera/depth/image");
        nh_->param<std::string>("xtion_rgb_info", xtion_rgb_info_topic_, "/camera/lowres_rgb/camera_info");
        nh_->param<std::string>("xtion_depth_info", xtion_depth_info_topic_, "/camera/depth/camera_info");
        nh_->param<std::string>("pg_rgb_image", camera_rgb_topic_, "/camera/highres_rgb/image");
        nh_->param<std::string>("pg_rgb_info", camera_rgb_info_topic_, "/camera/highres_rgb/camera_info");


        ROS_INFO( "subscribing to topic %s", xtion_rgb_topic_.c_str( ));
        ROS_INFO( "subscribing to topic %s", xtion_rgb_info_topic_.c_str( ));
        ROS_INFO( "subscribing to topic %s", xtion_depth_topic_.c_str() );
        ROS_INFO( "subscribing to topic %s", xtion_depth_info_topic_.c_str() );
        ROS_INFO( "subscribing to topic %s", camera_rgb_topic_.c_str());
        ROS_INFO( "subscribing to topic %s", camera_rgb_info_topic_.c_str());

        if ( use_cloud_ == true ) {
            nh_->param<std::string>("xtion_points", xtion_cloud_topic_, "/camera/points");
            ROS_INFO( "subscribing to topic %s", xtion_cloud_topic_.c_str());
        }
        recog_pub_ = nh.advertise<apc_msgs::BinObjects>(object_topic_name_, 100);
        recog_client_ = nh.serviceClient<apc_msgs::RecogStatus>( object_srv_name_ );
    }
    else if ( srv_mode_ == 2 ) {
        ROS_INFO("constructor in block mode");
    }


}

// desctructor
OfflineRecogniser::~OfflineRecogniser() {
    exit_flag_ = true;
    sensor_cond_.notify_all();
    process_thread_.interrupt();
    process_thread_.join();

    //Remove the window to display the image
//    cv::destroyWindow(g_rgb_win_name);
//    cv::destroyWindow( g_mask_win_name );
}


// sensor callback with point cloud
void OfflineRecogniser::sensor_callback( const sensor_msgs::ImageConstPtr & rgb_image_msg,
                                         const sensor_msgs::CameraInfoConstPtr & rgb_image_info,
                                         const sensor_msgs::ImageConstPtr & depth_image_msg,
                                         const sensor_msgs::CameraInfoConstPtr & depth_image_info,
                                         const sensor_msgs::ImageConstPtr & camera_image_msg,
                                         const sensor_msgs::CameraInfoConstPtr & camera_image_info,
                                         const sensor_msgs::PointCloud2ConstPtr & cloud_ptr_msg ) {
    ROS_INFO_ONCE( "[sensor_callback] Sensor information available" );

    bool lrecg, icap;

    srvc_mutex_.lock();
    lrecg = recogniser_done_;
    icap = image_captured_;
    srvc_mutex_.unlock();

    if (!lrecg && !icap) {
        SensorData* data = sensor_data_ptr_;
        xtion_rgb_model_.fromCameraInfo( rgb_image_info );
        xtion_depth_model_.fromCameraInfo( depth_image_info );
        camera_rgb_model_.fromCameraInfo( camera_image_info );

        // set buffer info
        data->xtion_rgb_ptr     = cv_bridge::toCvCopy( rgb_image_msg, sensor_msgs::image_encodings::BGR8 );
        data->xtion_depth_ptr   = cv_bridge::toCvCopy( depth_image_msg, sensor_msgs::image_encodings::TYPE_32FC1 );
        data->use_cloud = true;
        data->xtion_cloud_ptr   = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>( ));
        pcl::fromROSMsg( *cloud_ptr_msg, *(data->xtion_cloud_ptr) );

        data->camera_rgb_ptr    = cv_bridge::toCvCopy( camera_image_msg, sensor_msgs::image_encodings::BGR8 );
//        camera_rgb_model_.rectifyImage( data->camera_rgb_ptr->image, data->camera_rgb_ptr->image );
        {
            boost::mutex::scoped_lock lock(sensor_mutex_);
            sensor_empty_ = false;
        }
        sensor_cond_.notify_one();

        srvc_mutex_.lock();
        image_captured_ = true;
        srvc_mutex_.unlock();
    }
}


// sensor callback without point cloud
void OfflineRecogniser::sensor_callback_no_cloud(const sensor_msgs::ImageConstPtr &rgb_image_msg,
                                                const sensor_msgs::CameraInfoConstPtr &rgb_image_info,
                                                const sensor_msgs::ImageConstPtr &depth_image_msg,
                                                const sensor_msgs::CameraInfoConstPtr &depth_image_info,
                                                const sensor_msgs::ImageConstPtr &camera_image_msg,
                                                const sensor_msgs::CameraInfoConstPtr &camera_image_info) {
    ROS_INFO_ONCE( "[sensor_callback] Sensor information available" );

    bool lrecg, icap;

    srvc_mutex_.lock();
    lrecg = recogniser_done_;
    icap = image_captured_;
    srvc_mutex_.unlock();
    if (!lrecg && !icap) {
        SensorData* data = sensor_data_ptr_;
        xtion_rgb_model_.fromCameraInfo( rgb_image_info );
        xtion_depth_model_.fromCameraInfo( depth_image_info );
        camera_rgb_model_.fromCameraInfo( camera_image_info );

        // set buffer info
        data->xtion_rgb_ptr     = cv_bridge::toCvCopy( rgb_image_msg, sensor_msgs::image_encodings::BGR8 );
        data->xtion_depth_ptr   = cv_bridge::toCvCopy( depth_image_msg, sensor_msgs::image_encodings::TYPE_32FC1 );
        data->use_cloud = false;

        data->camera_rgb_ptr    = cv_bridge::toCvCopy( camera_image_msg, sensor_msgs::image_encodings::BGR8 );
//        camera_rgb_model_.rectifyImage( data->camera_rgb_ptr->image, data->camera_rgb_ptr->image );
        {
            boost::mutex::scoped_lock lock(sensor_mutex_);
            sensor_empty_ = false;
        }
        sensor_cond_.notify_one();

        srvc_mutex_.lock();
        image_captured_ = true;
        srvc_mutex_.unlock();
    }
}



/** callback for target object request */
bool OfflineRecogniser::target_srv_callback(apc_msgs::TargetRequest::Request &req,
                                            apc_msgs::TargetRequest::Response &resp) {
    srvc_mutex_.lock();

    if ( recogniser_done_ ) {
        ROS_INFO_ONCE("[target_srv_callback] target item request received");
        // srv index and data on disk
        recogniser_done_ = false;
        image_captured_  = false;

        // set data index value
        srv_bin_id_ = req.BinID;
        srv_rm_object_indices_.clear();
        for ( int i = 0; i < (int)req.RemovedObjectIndices.size(); ++ i )
            srv_rm_object_indices_.push_back( (int)req.RemovedObjectIndices[i] );
        srv_object_name_ = req.ObjectName;
        srv_bin_contents_.clear();
        for ( int i = 0; i < (int)req.BinContents.size(); ++ i )
            srv_bin_contents_.push_back( req.BinContents[i] );

        ROS_INFO( "Target item bin index %s", srv_bin_id_.c_str() );

        resp.Found = true;
        target_received_ = true;
        srvc_mutex_.unlock();
    }
    else {
        resp.Found = false;
        srvc_mutex_.unlock();
    }
    return true;
}


/** callback for target request, ZJU block mode */
bool OfflineRecogniser::target_srv_callback_block(apc_msgs::RecogniseALG::Request &req,
                                                  apc_msgs::RecogniseALG::Response &resp) {
    if ( recogniser_done_ ) {
        ROS_INFO_ONCE("[target_srv_callback_block] target item request received");

        // setup flags
        recogniser_done_ = false;
        image_captured_ = false;

        // set bin configuration
        srv_bin_id_ = req.bin_id;
        srv_rm_object_indices_.clear();
        for ( int i = 0; i < (int)req.RemovedObjectIndices.size(); ++ i )
            srv_rm_object_indices_.push_back( (int)req.RemovedObjectIndices[i] );
        srv_object_name_ = req.ObjectName;
        srv_bin_contents_.clear();
        for ( int i = 0; i < (int)req.BinContents.size(); ++ i )
            srv_bin_contents_.push_back( req.BinContents[i] );

        // set sensor information
        SensorData * data = sensor_data_ptr_;
        xtion_rgb_model_.fromCameraInfo( req.xtion_rgb_info );
        xtion_depth_model_.fromCameraInfo( req.xtion_depth_info );
        camera_rgb_model_.fromCameraInfo( req.pg_rgb_info );
        data->xtion_rgb_ptr     = cv_bridge::toCvCopy( req.xtion_rgb_image, sensor_msgs::image_encodings::BGR8 );
        data->xtion_depth_ptr   = cv_bridge::toCvCopy( req.xtion_depth_image, sensor_msgs::image_encodings::TYPE_32FC1 );
        //! @todo: add pg flag to recogniser
//        data->camera_rgb_ptr    = cv_bridge::toCvCopy( req.pg_rgb_image, sensor_msgs::image_encodings::BGR8 );
        data->use_cloud = req.use_cloud;
        if ( data->use_cloud == true ) {
            data->xtion_cloud_ptr   = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>( ));
            pcl::fromROSMsg( req.cloud, *(data->xtion_cloud_ptr) );
        }

        {
            boost::mutex::scoped_lock lock(sensor_mutex_);
            sensor_empty_ = false;
        }
        sensor_cond_.notify_one();

        {
            boost::mutex::scoped_lock lock( recogniser_mutex_ );
            while ( !recogniser_done_ ) {
                recogniser_cond_.wait(lock);
            }
        }

        // assign response variables
        resp.found = true;

    }
    else {
        resp.found = false;
        resp.items_in_bin.clear();

    }
    return true;
}


/** main processing function */
void OfflineRecogniser::process() {
    while ( !exit_flag_ ) {
        ROS_INFO_ONCE( "[process] Process function" );
        {
            boost::mutex::scoped_lock lock(sensor_mutex_);
            while(sensor_empty_) {
                sensor_cond_.wait( lock );
            }
        }

        {
            SensorData * data = sensor_data_ptr_;
            if ( data->xtion_rgb_ptr->image.rows != 0 && data->xtion_rgb_ptr->image.cols != 0 &&
                 data->xtion_depth_ptr->image.rows != 0 && data->xtion_depth_ptr->image.cols != 0 /*&&
                 data->camera_rgb_ptr->image.rows != 0 && data->camera_rgb_ptr->image.cols != 0 */) {
                bin_objs_.items_in_bin.clear();
                ROS_INFO("Load mask images with index %s", srv_bin_id_.c_str());

                // kernel descriptor recogniser
                cv::Mat rgb_image = data->xtion_rgb_ptr->image.clone();
                cv::imshow( "rgb_image", rgb_image );
                cv::waitKey(3000);
                cv::Mat depth_image = data->xtion_depth_ptr->image.clone();
                /*
                switch (op_mode_) {
                case 1:
                {
                    // load mask image
                    cv::Mat xtion_rgb_mask = cv::imread( mask_dir_ + "/mask_xtion_rgb_" + srv_bin_id_ + ".png", CV_LOAD_IMAGE_GRAYSCALE );
                    cv::Mat empty_image = cv::imread( string(mask_dir_+"/"+srv_bin_id_+"_empty.png"), CV_LOAD_IMAGE_COLOR );
                    cv::Mat empty_depth_image = cv::imread( string(mask_dir_+"/"+srv_bin_id_+"_depth_empty.png"), CV_LOAD_IMAGE_ANYDEPTH );
                    kdr_.load_sensor_data( rgb_image, depth_image );
                    kdr_.load_info( empty_image, xtion_rgb_mask, empty_depth_image );
                    kdr_.set_env_configuration( srv_object_name_, srv_bin_contents_ );
                    vector<pair<string, vector<cv::Point> > > results;
                    kdr_.process( results );
                    for ( int i = 0; i < (int)results.size(); ++ i ) {
                        apc_msgs::Object obj;
                        obj.name = results[i].first;
                        obj.use_pose = false;
                        for ( int j = 0; j < results[i].second.size(); ++ j ) {
                            obj.convex_hull_x.push_back( results[i].second[j].x );
                            obj.convex_hull_y.push_back( results[i].second[j].y );
                        }
                        bin_objs_.bin_id = srv_bin_id_;
                        bin_objs_.items_in_bin.push_back( obj );
                    }
                }
                    break;
                case 2:
                {
                    ebr_.load_sensor_data( rgb_image );
                    ebr_.set_env_configuration( srv_object_name_, srv_bin_contents_ );
                    vector<pair<string, vector<cv::Point> > > results;
                    ebr_.process( results );
                    for ( int i = 0; i < (int)results.size(); ++ i ) {
                        apc_msgs::Object obj;
                        obj.name = results[i].first;
                        obj.use_pose = false;
                        for ( int j = 0; j < results[i].second.size(); ++ j ) {
                            obj.convex_hull_x.push_back( results[i].second[j].x );
                            obj.convex_hull_y.push_back( results[i].second[j].y );
                        }
                        bin_objs_.bin_id = srv_bin_id_;
                        bin_objs_.items_in_bin.push_back( obj );
                    }
                }
                    break;
                case 3:
                {
                    cv::Mat xtion_rgb_mask = cv::imread( mask_dir_ + "/mask_xtion_rgb_" + srv_bin_id_ + ".png", CV_LOAD_IMAGE_GRAYSCALE );
                    cv::Mat empty_image = cv::imread( string(mask_dir_+"/"+srv_bin_id_+"_empty.png"), CV_LOAD_IMAGE_COLOR );
                    cv::Mat empty_depth_image = cv::imread( string(mask_dir_+"/"+srv_bin_id_+"_depth_empty.png"), CV_LOAD_IMAGE_ANYDEPTH );

                    // kernel descriptor recogniser
                    kdr_.load_sensor_data( rgb_image, depth_image );
                    kdr_.load_info( empty_image, xtion_rgb_mask, empty_depth_image );
                    kdr_.set_env_configuration( srv_object_name_, srv_bin_contents_ );
                    vector<pair<string, vector<cv::Point> > > kd_results;
                    kdr_.process( kd_results );
                    // eblearn recogniser
                    ebr_.load_sensor_data( rgb_image );
                    ebr_.set_env_configuration( srv_object_name_, srv_bin_contents_ );
                    vector<pair<string, vector<cv::Point> > > eb_results;
                    ebr_.process(eb_results);

                    //! @todo: combination of two difference results
                    vector<pair<string, vector<cv::Point> > > results;
                    bin_objs_.bin_id = srv_bin_id_;
                    for ( int i = 0; i < (int)results.size(); ++ i ) {
                        apc_msgs::Object obj;
                        obj.name = results[i].first;
                        obj.use_pose = false;
                        for ( int j = 0; j < results[i].second.size(); ++ j ) {
                            obj.convex_hull_x.push_back( results[i].second[j].x );
                            obj.convex_hull_y.push_back( results[i].second[j].y );
                        }
                        bin_objs_.items_in_bin.push_back( obj );
                    }
                }
                    break;
                case 4:
                {
                    cv::Mat xtion_rgb_mask = cv::imread( mask_dir_ + "/mask_xtion_rgb_" + srv_bin_id_ + ".png", CV_LOAD_IMAGE_GRAYSCALE );
                    cv::Mat empty_image = cv::imread( string(mask_dir_+"/"+srv_bin_id_+"_empty.png"), CV_LOAD_IMAGE_COLOR );
                    cv::Mat empty_depth_image = cv::imread( string(mask_dir_+"/"+srv_bin_id_+"_depth_empty.png"), CV_LOAD_IMAGE_ANYDEPTH );

                    // kernel descriptor recogniser
                    kdr_.load_sensor_data( rgb_image, depth_image );
                    kdr_.load_info( empty_image, xtion_rgb_mask, empty_depth_image );
                    kdr_.set_env_configuration( srv_object_name_, srv_bin_contents_ );
                    vector<pair<string, vector<cv::Point> > > kd_results;
                    kdr_.process( kd_results );

                    // ebleran recogniser
                    ebr_.load_sensor_data( rgb_image );
                    ebr_.set_env_configuration( srv_object_name_, srv_bin_contents_ );
                    vector<pair<string, vector<cv::Point> > > eb_results;
                    ebr_.process(eb_results);

                    // generate comb_results from two results
                    vector<pair<string, vector<cv::Point> > > results;

                    bin_objs_.bin_id = srv_bin_id_;
                    for ( int i = 0; i < (int)results.size(); ++ i ) {
                        apc_msgs::Object obj;
                        obj.name = results[i].first;
                        obj.use_pose = false;
                        for ( int j = 0; j < results[i].second.size(); ++ j ) {
                            obj.convex_hull_x.push_back( results[i].second[j].x );
                            obj.convex_hull_y.push_back( results[i].second[j].y );
                        }
                        bin_objs_.items_in_bin.push_back( obj );
                    }

                    for ( int item_i = 0; item_i < (int)srv_bin_contents_.size(); ++  item_i ) {
                        string item_name = srv_bin_contents_[item_i];
                        RecogMethod method = methods_[item_name];
                        switch( method ) {
                        case RGB_RECOG:
                        {
                            RGBRecogniser rgb_reco( rgb_image );
                            rgb_reco.set_env_configuration( item_name, srv_bin_contents_ );
                            rgb_reco.set_camera_params(camera_rgb_model_.fx(), camera_rgb_model_.fy(), camera_rgb_model_.cx(), camera_rgb_model_.cy());
                            rgb_reco.load_models( xml_dir_ );
                            bool reco_success = rgb_reco.run( 15, 10, true );
                            if ( reco_success == true ) {
                                list<SP_Object> rgb_objs = rgb_reco.get_objects();

                                foreach( rgb_obj, rgb_objs ){
                                    // trying retrive the item from results
                                    for ( int res_i = 0; res_i < results.size(); ++ res_i ) {
                                        if ( results[res_i].first == rgb_obj->model_->name_ ) {
                                            apc_msgs::Object &obj = bin_objs_.items_in_bin[res_i];
                                            obj.use_pose = true;
                                            obj.pose.position.x = rgb_obj->pose_.t_.x();
                                            obj.pose.position.y = rgb_obj->pose_.t_.y();
                                            obj.pose.position.z = rgb_obj->pose_.t_.z();

                                            obj.pose.orientation.x = rgb_obj->pose_.q_.x();
                                            obj.pose.orientation.y = rgb_obj->pose_.q_.y();
                                            obj.pose.orientation.z = rgb_obj->pose_.q_.z();
                                            obj.pose.orientation.w = rgb_obj->pose_.q_.w();

                                            obj.mean_quality = rgb_obj->score_;
                                        }
                                    }
                                }
                            }
                        }
                            break;
                        case RGBD_RECOG:
                        {

                        }
                            break;
                        }
                    }
                }
                    break;
                default:
                    break;
                }

                */


                if ( srv_mode_ == 2 ) {
                    {
                        boost::mutex::scoped_lock lock(recogniser_mutex_);
                        recogniser_done_ = true;
                    }
                    recogniser_cond_.notify_one();
                }

                if ( srv_mode_ == 1 ) {
                    // client to call recogniser notification
                    apc_msgs::RecogStatus recog_status;
                    recog_status.request.recog = true;
                    if( recog_client_.call(recog_status) ) {
                        ROS_INFO( "return status: %s", recog_status.response.pub? "true" : "false" );
                    }
                    else {
                        ROS_ERROR( "Failed to call service %s", object_srv_name_.c_str() );
                    }


                    for ( int i = 0; i < (int)bin_objs_.items_in_bin.size(); ++ i )
                        cout << bin_objs_.items_in_bin[i].name << "; ";
                    cout << endl;

                    recog_pub_.publish( bin_objs_ );

                    srvc_mutex_.lock();
                    recogniser_done_ = true;
                    srvc_mutex_.unlock();

                    // reset sensor empty
                    sensor_mutex_.lock();
                    sensor_empty_ = true;
                    sensor_mutex_.unlock();
                }
            }
        }

        if ( !exit_flag_ ) {
            boost::unique_lock<boost::mutex> sensor_lock( sensor_mutex_ );
            sensor_empty_ = true;
        }
    }
}


/** load method configuration method */
void OfflineRecogniser::load_method_config( string filename ) {
    ROS_INFO( "Read method configuration file from %s", filename.c_str() );
    ifstream in( filename.c_str() );
    string line;
    getline( in, line );
    vector<string> seg_line;
    while ( getline(in, line) ) {
        boost::algorithm::split( seg_line, line, boost::algorithm::is_any_of(" ") );
        if ( seg_line[1] == "rgb" )
            methods_.insert( make_pair(seg_line[0], RGB_RECOG) );
        else if ( seg_line[1] == "rgbd" )
            methods_.insert( make_pair(seg_line[0], RGBD_RECOG) );
        ROS_INFO( "Object name %s and method %s ", seg_line[0].c_str(), seg_line[1].c_str());
        seg_line.clear();
    }
}


void OfflineRecogniser::start_monitor( void ) {
    // service target object
    ROS_INFO("start monitor with mode = %s", srv_mode_ == 1? "UTS_Mode": "ZJU_Mode");
    if ( srv_mode_ == 1 ) {
        ros::ServiceServer target_srv = nh_->advertiseService( target_srv_name_, &OfflineRecogniser::target_srv_callback, this );

        // subscribe to sensors
        xtion_rgb_sub_.subscribe(*nh_, xtion_rgb_topic_, 1 );
        xtion_rgb_info_sub_.subscribe( *nh_, xtion_rgb_info_topic_, 1 );
        xtion_depth_sub_.subscribe( *nh_, xtion_depth_topic_, 1 );
        xtion_depth_info_sub_.subscribe( *nh_, xtion_depth_info_topic_, 1 );
        camera_rgb_sub_.subscribe( *nh_, camera_rgb_topic_, 1 );
        camera_rgb_info_sub_.subscribe( *nh_, camera_rgb_info_topic_, 1 );

        if ( use_cloud_ == true ) {
            xtion_cloud_sub_.subscribe( *nh_, xtion_cloud_topic_, 1 );
            m_sensor_sync_.reset( new message_filters::Synchronizer<sensor_sync_policy>( sensor_sync_policy(100), xtion_rgb_sub_, xtion_rgb_info_sub_, xtion_depth_sub_, xtion_depth_info_sub_, camera_rgb_sub_, camera_rgb_info_sub_, xtion_cloud_sub_) );
            m_sensor_sync_->registerCallback( boost::bind( &OfflineRecogniser::sensor_callback, this, _1, _2, _3, _4, _5, _6, _7 ) );
        }
        else {
            m_no_cloud_sensor_sync_.reset( new message_filters::Synchronizer<no_cloud_sensor_sync_policy>( no_cloud_sensor_sync_policy(10), xtion_rgb_sub_, xtion_rgb_info_sub_, xtion_depth_sub_, xtion_depth_info_sub_, camera_rgb_sub_, camera_rgb_info_sub_  ) );
            m_no_cloud_sensor_sync_->registerCallback( boost::bind( &OfflineRecogniser::sensor_callback_no_cloud, this, _1, _2, _3, _4, _5, _6 ) );
        }
        process_thread_ = boost::thread(boost::bind(&OfflineRecogniser::process, this));

        ros::MultiThreadedSpinner spinner(6);
        ros::Rate loop(10);
        while ( ros::ok() ) {
            spinner.spin();
            loop.sleep();
        }
    }
    else if ( srv_mode_ == 2 ) {
        ros::ServiceServer target_srv = nh_->advertiseService( target_srv_name_, &OfflineRecogniser::target_srv_callback_block, this );
        process_thread_ = boost::thread( boost::bind(&OfflineRecogniser::process, this) );

        ros::MultiThreadedSpinner spinner(6);
        ros::Rate loop(10);
        while ( ros::ok() ) {
            spinner.spin();
            loop.sleep();
        }

    }
}


int main( int argc, char ** argv ) {
    ros::init( argc, argv, "offline_recogniser" );
    ros::NodeHandle nh("~");
    OfflineRecogniser reco( nh );
    reco.start_monitor();
    return 0;

}




