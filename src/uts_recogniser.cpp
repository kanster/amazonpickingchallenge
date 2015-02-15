#include "include/uts_recogniser.h"

#include <boost/bind.hpp>
#include <unistd.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// global topic names
const string g_camera_rgb_name  = "/camera/image_color";
const string g_camera_info      = "/camera/camera_info";

const string g_xtion_rgb_name   = "/camera/rgb/image_color";
const string g_xtion_rgb_info   = "/camera/rgb/camera_info";

const string g_xtion_depth_name = "/camera/depth/image_raw";
const string g_xtion_depth_info = "/camera/depth/camera_info";

const string g_xtion_cloud_name = "/camera/depth/points";
const string g_target_name      = "/target_object";
const string g_target_srv_name  = "/target_object_srv";
const string g_json_filename    = "../data/uts.json";
const string g_models_dir       = "../data/object_models";


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
    // parse json files
    JSON json;
    json.from_file( g_json_filename );
    bin_contents_   = json.bin_contents_;
    work_order_     = json.work_order_;

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
}

// desctructor
UTSRecogniser::~UTSRecogniser() {
    exit_flag_ = true;
    sensor_cond_.notify_all();
    process_thread_.interrupt();
    process_thread_.join();
}


/** acquire read memory */

void UTSRecogniser::acquire_read_buffer(SensorData *&buf) {
    boost::unique_lock<boost::mutex> lock( sensor_mutex_ );
    while ( sensor_empty_ ) {
        if (debug_ == true)
            ROS_INFO_ONCE("[acquire_read_buffer] target: %d, sensor empty: %d", target_received_, sensor_empty_ );
        sensor_cond_.wait( lock );
    }
    buf = &sensor_data_;
}


/** acquire write memory */

bool UTSRecogniser::acquire_write_buffer(SensorData *&buf) {
    boost::system_time wait_until = boost::get_system_time() + boost::posix_time::milliseconds(100);
    while ( boost::get_system_time() < wait_until ) {
        if ( debug_ == true )
            ROS_INFO_ONCE("[acquire_write_buffer] target: %d, sensor empty: %d", target_received_, sensor_empty_ );
        boost::unique_lock<boost::mutex> lock( sensor_mutex_, boost::try_to_lock );
        if ( lock.owns_lock() ) {
            if ( sensor_empty_ ) {
                buf = & sensor_data_;
                return true;
            }
        }
        usleep(100);
    }
    return false;
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
    if (!lrecg && !icap)
    {
        if (debug_)
            ROS_INFO("No %d  target object with name %s", target_count_, target_item_.target_name.c_str());
        SensorData* data = &sensor_data_;
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
//  std::cout << "target_srv_callback stage 1 :" << recogniser_done_ << "\n";
    srvc_mutex_.lock();
    if(recogniser_done_)
    {
//      std::cout << "target_srv_callback stage 2" << "\n";
        recogniser_done_ = false;
        image_captured_ = false;
        target_item_.target_name = req.ObjectName;
        target_item_.target_index = req.ObjectIndex;
        target_received_ = true;
        target_count_ ++;
        srvc_mutex_.unlock();
    }
    else
    {
        srvc_mutex_.unlock();
    }
    return true;
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
                                  sensor_sync_policy(10),
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



/** main processing function */

void UTSRecogniser::process() {
    while ( !exit_flag_ ) {
        {
            boost::mutex::scoped_lock lock(sensor_mutex_);
            while(sensor_empty_)
            {
                sensor_cond_.wait( lock );
            }

        }

        SensorData * data = &sensor_data_;
        if ( data->xtion_rgb_ptr->image.rows != 0 && data->xtion_rgb_ptr->image.cols != 0 &&
             data->xtion_depth_ptr->image.rows != 0 && data->xtion_depth_ptr->image.cols != 0 &&
             data->camera_rgb_ptr->image.rows != 0 && data->camera_rgb_ptr->image.cols != 0 ) {

            // convert depth image scale
            double min;
            double max;
            cv::minMaxIdx(data->xtion_depth_ptr->image, &min, &max);
            cv::convertScaleAbs(data->xtion_depth_ptr->image, data->xtion_depth_ptr->image, 255/max);

            usleep(5000000);

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


/** obtain recognition method */

void UTSRecogniser::method_retrieval() {
    /** @todo table for method retrieval
      * table is predefined based object type
      */
    method_ = RGB_RECOG;
}

