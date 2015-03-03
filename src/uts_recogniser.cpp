#include "include/uts_recogniser.h"

#include <boost/bind.hpp>
#include <unistd.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

// firewire rgb topics
const string g_camera_rgb_name  = "/camera/image_color";
const string g_camera_info      = "/camera/camera_info";

// xtion rgb topics
const string g_xtion_rgb_name   = "/camera/rgb/image_color";
const string g_xtion_rgb_info   = "/camera/rgb/camera_info";

// xtion depth topics
const string g_xtion_depth_name = "/camera/depth/image_raw";
const string g_xtion_depth_info = "/camera/depth/camera_info";

// xtion cloud topics
const string g_xtion_cloud_name = "/camera/depth/points";

// service server name
const string g_target_srv_name  = "/target_object_srv";

// json configuration file
const string g_json_filename    = "../data/amazon.json";

// object models, xml
const string g_models_dir       = "../data/amazon_models/";

// rgbd segmenter model
const string g_seg_model_dir    = "../data/seg_models/";
// method configuration
const string g_method_filename  = "../data/method.txt";

// constructor
UTSRecogniser::UTSRecogniser(ros::NodeHandle &nh)
    : exit_flag_(false)
    , sensor_empty_(true)
    , target_received_(false)
    , target_count_(0)
    , recogniser_done_(true)
    , image_captured_(true)
    , debug_(true){
    nh_ = & nh;

    // initialise sensor data pointer
    cindex_ = 0;
    imshow_data_ptr_ = 0;
    sensor_data_ptr_ = &d_buf_[0];

    // parse json files
    JSON json;
    json.from_file( g_json_filename );
    bin_contents_   = json.bin_contents_;
    work_order_     = json.work_order_;

    // load methods for all candidate objects, for amazon picking challenge
    // totally 27 objects, for uts dataset, totally 9 objects
    load_method_config( g_method_filename );

    // set recognition methods for all working order items
    for ( int i = 0; i < (int)work_order_.size(); ++ i ) {
        Item item;
        item.object_name = work_order_[i].second;
        if ( methods_[item.object_name] == "rgb" )
            item.method = RGB_RECOG;
        else if ( methods_[item.object_name] == "rgbd" )
            item.method = RGBD_RECOG;
        items_.push_back( item );
    }


    // resolve topic name
    xtion_rgb_topic_ = nh.resolveName( "/camera/rgb/image_color" );
    ROS_INFO( "subscribing to topic %s", xtion_rgb_topic_.c_str( ));
    xtion_rgb_info_topic_ = nh.resolveName("/camera/rgb/camera_info");
    ROS_INFO( "subscribing to topic %s", xtion_rgb_info_topic_.c_str( ));
    xtion_depth_topic_ = nh.resolveName("/camera/depth/image_raw");
    ROS_INFO( "subscribing to topic %s", xtion_depth_topic_.c_str() );
    xtion_depth_info_topic_ = nh.resolveName( "/camera/depth/camera_info" );
    ROS_INFO( "subscribing to topic %s", xtion_depth_info_topic_.c_str() );
    xtion_cloud_topic_ = nh.resolveName( "/camera/depth/points" );
    ROS_INFO( "subscribing to topic %s", xtion_cloud_topic_.c_str());
    camera_rgb_topic_ = nh.resolveName( "/camera/image_color" );
    ROS_INFO( "subscribing to topic %s", camera_rgb_topic_.c_str());
    camera_rgb_info_topic_ = nh.resolveName( "/camera/camera_info" );
    ROS_INFO( "subscribing to topic %s", camera_rgb_info_topic_.c_str());

    //Open a window to display the image
    cv::namedWindow(WINDOW_NAME);
}

// desctructor
UTSRecogniser::~UTSRecogniser() {
    exit_flag_ = true;
    sensor_cond_.notify_all();
    process_thread_.interrupt();
    process_thread_.join();

    //Remove the window to display the image
    cv::destroyWindow(WINDOW_NAME);
}


// sensor callback
void UTSRecogniser::sensor_callback( const sensor_msgs::ImageConstPtr & rgb_image_msg,
                                     const sensor_msgs::CameraInfoConstPtr & rgb_image_info,
                                     const sensor_msgs::ImageConstPtr & depth_image_msg,
                                     const sensor_msgs::CameraInfoConstPtr & depth_image_info,
                                     const sensor_msgs::PointCloud2ConstPtr & cloud_ptr_msg,
                                     const sensor_msgs::ImageConstPtr & camera_image_msg,
                                     const sensor_msgs::CameraInfoConstPtr & camera_image_info ) {
    ROS_INFO_ONCE( "[sensor_callback] Sensor information available" );

    bool lrecg, icap;

    srvc_mutex_.lock();
    lrecg = recogniser_done_;
    icap = image_captured_;
    srvc_mutex_.unlock();
//  std::cout <<"sensor callback" << "\n";
    if (!lrecg && !icap) {
        SensorData* data = sensor_data_ptr_;
        xtion_rgb_model_.fromCameraInfo( rgb_image_info );
        xtion_depth_model_.fromCameraInfo( depth_image_info );
        camera_rgb_model_.fromCameraInfo( camera_image_info );

        // set buffer info
        data->xtion_rgb_ptr     = cv_bridge::toCvCopy( rgb_image_msg, sensor_msgs::image_encodings::BGR8 );
        data->xtion_depth_ptr   = cv_bridge::toCvCopy( depth_image_msg, sensor_msgs::image_encodings::TYPE_32FC1 );
        data->xtion_cloud_ptr   = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>( ));
        pcl::fromROSMsg( *cloud_ptr_msg, *(data->xtion_cloud_ptr) );

        data->camera_rgb_ptr    = cv_bridge::toCvCopy( camera_image_msg, sensor_msgs::image_encodings::BGR8 );
        camera_rgb_model_.rectifyImage( data->camera_rgb_ptr->image, data->camera_rgb_ptr->image );

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

bool UTSRecogniser::target_srv_callback(uts_recogniser::TargetRequest::Request &req, uts_recogniser::TargetRequest::Response &resp) {
    if ( debug_ == true )
        ROS_INFO_ONCE("[target_srv_callback] target item request received");
//    std::cout << "target_srv_callback stage 1 :" << recogniser_done_ << "\n";
    srvc_mutex_.lock();
    if(recogniser_done_) {
        // add protection guard, check the input object index
        if ( req.ObjectIndex < work_order_.size() && req.ObjectIndex >= 0 ) {
//            std::cout << "target_srv_callback stage 2" << "\n";
            recogniser_done_ = false;
            image_captured_ = false;
            target_item_.target_name = req.ObjectName;
            target_item_.target_index = (int)req.ObjectIndex;
            for ( int i = 0; i < (int)req.RemovedObjectIndices.size(); ++ i )
                target_item_.removed_items_indices.push_back((int)req.RemovedObjectIndices[i]);
            reco_method_ = items_[target_item_.target_index].method;
            ROS_INFO( "Target name %s with index %d", target_item_.target_name.c_str(), (int)target_item_.target_index );

            target_received_ = true;
            target_count_ ++;
            srvc_mutex_.unlock();
        }
        else {
            srvc_mutex_.unlock();
            return false;
        }
    }
    else {
        srvc_mutex_.unlock();
    }
    return true;
}


/** main processing function */
void UTSRecogniser::process() {
    while ( !exit_flag_ ) {
        {
            boost::mutex::scoped_lock lock(sensor_mutex_);
            while(sensor_empty_) {
                sensor_cond_.wait( lock );
            }

        }

        SensorData * data = sensor_data_ptr_;
        if ( data->xtion_rgb_ptr->image.rows != 0 && data->xtion_rgb_ptr->image.cols != 0 &&
             data->xtion_depth_ptr->image.rows != 0 && data->xtion_depth_ptr->image.cols != 0 &&
             data->camera_rgb_ptr->image.rows != 0 && data->camera_rgb_ptr->image.cols != 0 ) {

            // convert depth image scale
            double min;
            double max;
            cv::minMaxIdx(data->xtion_depth_ptr->image, &min, &max);
            cv::convertScaleAbs(data->xtion_depth_ptr->image, data->xtion_depth_ptr->image, 255/max);

            // recognition
            /*
            switch ( reco_method_ ) {
            case RGB_RECOG:
            {
                RGBRecogniser recog( data->camera_rgb_ptr->image );
                recog.set_env_configuration( target_item_.target_index, work_order_, bin_contents_ );
                recog.set_camera_params( camera_rgb_model_.fx(), camera_rgb_model_.fy(), camera_rgb_model_.cx(), camera_rgb_model_.cy() );
                recog.load_models( g_models_dir );
                recog.run( true );
                break;
            }
            case RGBD_RECOG:
            {
                RGBDRecogniser recog( data->xtion_rgb_ptr->image, data->xtion_depth_ptr->image, data->xtion_cloud_ptr, g_seg_model_dir );
                recog.set_env_configuration( target_item_.target_index, work_order_, bin_contents_ );
                recog.set_camera_params( xtion_rgb_model_.fx(), xtion_rgb_model_.fy(), xtion_rgb_model_.cx(), xtion_rgb_model_.cy() );
                recog.load_models( g_models_dir );
                recog.run(true);
            }
            default:
                break;
            }
            */

            imshow_data_ptr_ = data;
            cindex_ = (++cindex_)%2;
            sensor_data_ptr_ = &d_buf_[cindex_];

            cv::imshow(WINDOW_NAME, data->camera_rgb_ptr->image);
            cv::waitKey(1);

            srvc_mutex_.lock();
            recogniser_done_ = true;
            srvc_mutex_.unlock();

            sensor_mutex_.lock();
            sensor_empty_ = true;
            sensor_mutex_.unlock();

        }

        if ( !exit_flag_ ) {
            boost::unique_lock<boost::mutex> sensor_lock( sensor_mutex_ );
            sensor_empty_ = true;
        }
        // call the robotic service to receive the pose/item

    }
}


/** load method configuration method */
void UTSRecogniser::load_method_config( string filename ) {
    ROS_INFO( "Read method configuration file from %s", filename.c_str() );
    ifstream in( filename.c_str() );
    string line;
    getline( in, line );
    vector<string> seg_line;
    while ( getline(in, line) ) {
        boost::algorithm::split( seg_line, line, boost::algorithm::is_any_of(" ") );
        methods_.insert( make_pair(seg_line[0], seg_line[1]) );
        ROS_INFO( "Object name %s and method %s ", seg_line[0].c_str(), seg_line[1].c_str());
        seg_line.clear();
    }
}


void UTSRecogniser::start_monitor( void ) {
    // service target object
    ros::ServiceServer target_srv = nh_->advertiseService( g_target_srv_name, &UTSRecogniser::target_srv_callback, this );


    // subscribe to sensors
    xtion_rgb_sub_.subscribe(*nh_, xtion_rgb_topic_, 1 );
    xtion_rgb_info_sub_.subscribe( *nh_, xtion_rgb_info_topic_, 1 );
    xtion_depth_sub_.subscribe( *nh_, xtion_depth_topic_, 1 );
    xtion_depth_info_sub_.subscribe( *nh_, xtion_depth_info_topic_, 1 );
    xtion_cloud_sub_.subscribe( *nh_, xtion_cloud_topic_, 1 );
    camera_rgb_sub_.subscribe( *nh_, camera_rgb_topic_, 1 );
    camera_rgb_info_sub_.subscribe( *nh_, camera_rgb_info_topic_, 1 );

    m_sensor_sync_.reset( new message_filters::Synchronizer<sensor_sync_policy>(
                                  sensor_sync_policy(100),
                                  xtion_rgb_sub_,
                                  xtion_rgb_info_sub_,
                                  xtion_depth_sub_,
                                  xtion_depth_info_sub_,
                                  xtion_cloud_sub_,
                                  camera_rgb_sub_,
                                  camera_rgb_info_sub_) );
    m_sensor_sync_->registerCallback( boost::bind( &UTSRecogniser::sensor_callback, this, _1, _2, _3, _4, _5, _6, _7 ) );

    process_thread_ = boost::thread(boost::bind(&UTSRecogniser::process, this));
    ros::MultiThreadedSpinner spinner(6);
    ros::Rate loop(1);
    while ( ros::ok() ) {
//      ros::spin();
        spinner.spin();
        loop.sleep();
    }
}





