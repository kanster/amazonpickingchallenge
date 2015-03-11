#include "include/offline_recogniser.h"

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

#include <pcl/console/parse.h>

// publish poses topic name
const string g_object_topic_name = "/object_poses";
const string g_object_srv_name   = "/recog_publish_srv";
// service server name
const string g_target_srv_name  = "/data_publish_srv";

// json configuration file
const string g_json_filename    = "./data/amazon.json";

// object models, xml
const string g_models_dir       = "./data/amazon_models";

// method configuration
const string g_method_filename  = "./data/method.txt";

const string g_rgb_win_name     = "rgb_image";
const string g_mask_win_name    = "mask_image";



// constructor
OfflineRecogniser::OfflineRecogniser(ros::NodeHandle &nh, string data_dir)
    : exit_flag_(false)
    , sensor_empty_(true)
    , target_received_(false)
    , target_count_(0)
    , recogniser_done_(true)
    , image_captured_(true)
    , debug_(true){
    nh_ = & nh;
    data_dir_ = data_dir;

    // cindex = 0;
    // imshow_data_ptr_ = 0;
    sensor_data_ptr_ = &sensor_data_;

    // parse json files
    JSON json;
    json.from_file( g_json_filename );
    bin_contents_   = json.bin_contents_;
    work_order_     = json.work_order_;

    // load methods for all candidate objects, for amazon picking challenge
    // totally 27 objects, for uts dataset, totally 9 objects
    load_method_config( g_method_filename );


    // resolve topic name
    xtion_rgb_topic_ = nh.resolveName( "/xtion/rgb/image" );
    ROS_INFO( "subscribing to topic %s", xtion_rgb_topic_.c_str( ));
    xtion_rgb_info_topic_ = nh.resolveName("/xtion/rgb/camera_info");
    ROS_INFO( "subscribing to topic %s", xtion_rgb_info_topic_.c_str( ));
    xtion_depth_topic_ = nh.resolveName("/xtion/depth/image");
    ROS_INFO( "subscribing to topic %s", xtion_depth_topic_.c_str() );
    xtion_depth_info_topic_ = nh.resolveName( "/xtion/depth/camera_info" );
    ROS_INFO( "subscribing to topic %s", xtion_depth_info_topic_.c_str() );
    xtion_cloud_topic_ = nh.resolveName( "/xtion/depth/points" );
    ROS_INFO( "subscribing to topic %s", xtion_cloud_topic_.c_str());
    camera_rgb_topic_ = nh.resolveName( "/camera/image" );
    ROS_INFO( "subscribing to topic %s", camera_rgb_topic_.c_str());
    camera_rgb_info_topic_ = nh.resolveName( "/camera/camera_info" );
    ROS_INFO( "subscribing to topic %s", camera_rgb_info_topic_.c_str());

    recog_pub_ = nh.advertise<apc_msgs::BinObjects>(g_object_topic_name, 100);
    recog_client_ = nh.serviceClient<apc_msgs::RecogStatus>( g_object_srv_name );
    //Open a window to display the image
//    cv::namedWindow(g_rgb_win_name);
//    cv::moveWindow( g_rgb_win_name, 0, 0 );
//    cv::namedWindow(g_mask_win_name);
//    cv::moveWindow( g_mask_win_name, 1280, 0 );
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

// sensor callback
void OfflineRecogniser::sensor_callback( const sensor_msgs::ImageConstPtr & rgb_image_msg,
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

bool OfflineRecogniser::target_srv_callback(apc_msgs::DataPublish::Request &req,
                                            apc_msgs::DataPublish::Response &resp) {
    if ( debug_ == true )
        ROS_INFO_ONCE("[target_srv_callback] target item request received");

    srvc_mutex_.lock();

    if ( recogniser_done_ ) {
        ROS_INFO_ONCE("[target_srv_callback] target item request received");
        // srv index and data on disk
        recogniser_done_ = false;
        image_captured_  = false;

        // set data index value
        data_index_ = req.DataIndex;

        ROS_INFO( "Target image index %d", data_index_ );
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



/** main processing function */
void OfflineRecogniser::process() {
    while ( !exit_flag_ ) {
        {
            boost::mutex::scoped_lock lock(sensor_mutex_);
            while(sensor_empty_) {
                sensor_cond_.wait( lock );
            }
        }


        {
            cout << "entering processing\n";
            SensorData * data = sensor_data_ptr_;


            if ( data->xtion_rgb_ptr->image.rows != 0 && data->xtion_rgb_ptr->image.cols != 0 &&
                 data->xtion_depth_ptr->image.rows != 0 && data->xtion_depth_ptr->image.cols != 0 &&
                 data->camera_rgb_ptr->image.rows != 0 && data->camera_rgb_ptr->image.cols != 0 ) {

//                // convert depth image scale
//                double min;
//                double max;
//                cv::minMaxIdx(data->xtion_depth_ptr->image, &min, &max);
//                cv::convertScaleAbs(data->xtion_depth_ptr->image, data->xtion_depth_ptr->image, 255/max);

                ROS_INFO("Load mask images with index %d", data_index_);
                string xtion_rgb_mask_path  = data_dir_ + "/xtion_rgb_mask_" + boost::lexical_cast<std::string>(data_index_) + ".png";
                string xtion_depth_mask_path= data_dir_ + "/xtion_depth_mask_" + boost::lexical_cast<std::string>(data_index_) + ".png";
                string camera_rgb_mask_path = data_dir_ + "/camera_rgb_mask_" + boost::lexical_cast<std::string>(data_index_) + ".png";
                // load mask image
                cv::Mat xtion_rgb_mask = cv::imread( xtion_rgb_mask_path, CV_LOAD_IMAGE_GRAYSCALE );
                cv::Mat xtion_depth_mask = cv::imread( xtion_depth_mask_path, CV_LOAD_IMAGE_GRAYSCALE );
                cv::Mat camera_rgb_mask = cv::imread( camera_rgb_mask_path, CV_LOAD_IMAGE_GRAYSCALE );


//                cv::imshow( g_mask_win_name, camera_rgb_mask );
//                cv::imshow(g_rgb_win_name, data->camera_rgb_ptr->image);
//                cv::waitKey(5);


                // foreach item in the bin
                // generate correspondece between bin id and collected data id
                string bin_id;  // better method by add constant to generate char
                switch (data_index_) {
                case 1:
                {
                    bin_id = "bin_A";
                    break;
                }
                case 2:
                {
                    bin_id = "bin_B";
                    break;
                }
                default:
                    break;
                }

                vector<string> items = bin_contents_[bin_id];
                apc_msgs::BinObjects bin_objs;
                for ( int i = 0; i < (int)items.size(); ++ i ) {
                    apc_msgs::Object obj;

                    string item_name = items[i];

                    ROS_INFO( "Start recognising item %s in bin %s", item_name.c_str(), bin_id.c_str() );

                    RecogMethod method = methods_[item_name];
                    switch (method) {
                    case RGB_RECOG:
                    {
                        cv::Mat bin_mask_image = cv::imread( camera_rgb_mask_path, CV_LOAD_IMAGE_GRAYSCALE );
                        RGBRecogniser recog( data->camera_rgb_ptr->image, bin_mask_image );
//                        recog.set_env_configuration( data_index_-1, work_order_, bin_contents_ );
                        recog.set_env_configuration( item_name, bin_contents_[bin_id] );
                        recog.set_camera_params( camera_rgb_model_.fx(), camera_rgb_model_.fy(), camera_rgb_model_.cx(), camera_rgb_model_.cy() );
                        recog.load_models( g_models_dir );
                        bool recog_success = recog.run(15, 10, true );
                        if ( recog_success == true ) {
                            list<SP_Object> recog_results = recog.get_objects();
                            foreach( object, recog_results ) {
                                // object name
                                obj.name = object->model_->name_;
                                // assign pose
                                obj.pose.position.x = object->pose_.t_.x();
                                obj.pose.position.y = object->pose_.t_.y();
                                obj.pose.position.z = object->pose_.t_.z();

                                obj.pose.orientation.x = object->pose_.q_.x();
                                obj.pose.orientation.y = object->pose_.q_.y();
                                obj.pose.orientation.z = object->pose_.q_.z();
                                obj.pose.orientation.w = object->pose_.q_.w();
                                // confidence score
                                obj.mean_quality = object->score_;
                                // used points no. is not decided

                                bin_objs.items_in_bin.push_back( obj );
                            }
                        }
                        break;
                    }
                    case RGBD_RECOG:
                    {
                        RGBDRecogniser recog( data->xtion_rgb_ptr->image, xtion_depth_mask, data->xtion_depth_ptr->image, data->xtion_cloud_ptr);
                        recog.set_env_configuration( item_name, bin_contents_[bin_id] );
                        recog.set_camera_params( xtion_rgb_model_.fx(), xtion_rgb_model_.fy(), xtion_rgb_model_.cx(), xtion_rgb_model_.cy() );
                        recog.load_models( g_models_dir );
                        bool recog_success = recog.run(true);
                        if ( recog_success == true ) {
                            list<SP_Object> recog_results = recog.get_objects();
                            foreach( object, recog_results ) {
                                obj.name = object->model_->name_;

                                obj.pose.position.x = object->pose_.t_.x();
                                obj.pose.position.y = object->pose_.t_.y();
                                obj.pose.position.z = object->pose_.t_.z();

                                obj.pose.orientation.x = object->pose_.q_.x();
                                obj.pose.orientation.y = object->pose_.q_.y();
                                obj.pose.orientation.z = object->pose_.q_.z();
                                obj.pose.orientation.w = object->pose_.q_.w();

                                obj.mean_quality = object->score_;

                                bin_objs.items_in_bin.push_back( obj );

                            }
                        }
                        break;
                    }

                    default:
                        break;

                    }
                }

                ROS_INFO( "Finish recognising for bin %s", bin_id.c_str() );


                recog_pub_.publish( bin_objs );

                // client to call recogniser notification
                apc_msgs::RecogStatus recog_status;
                recog_status.request.recog = true;
                ROS_INFO( "Sending recognition notification");
                if( recog_client_.call(recog_status) ) {
                    ROS_INFO( "return status: %s", recog_status.response.pub? "true" : "false" );
                }
                else {
                    ROS_ERROR( "Failed to call service %s", g_object_srv_name.c_str() );
                }

                srvc_mutex_.lock();
                recogniser_done_ = true;
                srvc_mutex_.unlock();
//                srvc_cond_.notify_one();


                sensor_mutex_.lock();
                sensor_empty_ = true;
                sensor_mutex_.unlock();



            }
        }

        if ( !exit_flag_ ) {
            boost::unique_lock<boost::mutex> sensor_lock( sensor_mutex_ );
            sensor_empty_ = true;
        }
        // call the robotic service to receive the pose/item

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
    ros::ServiceServer target_srv = nh_->advertiseService( g_target_srv_name, &OfflineRecogniser::target_srv_callback, this );

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
    m_sensor_sync_->registerCallback( boost::bind( &OfflineRecogniser::sensor_callback, this, _1, _2, _3, _4, _5, _6, _7 ) );

    process_thread_ = boost::thread(boost::bind(&OfflineRecogniser::process, this));
    ros::MultiThreadedSpinner spinner(4);
    ros::Rate loop(1);
    while ( ros::ok() ) {
        spinner.spin();
        loop.sleep();
    }
}

void print_usage( char * prog_name ) {
    std::cout << "\nUsage: " << prog_name << " [options]\n"
              << "Options:\n"
              << "-------------------------------------------\n"
              << "\t-dir <string>           data directory\n"
              << "\t-h                    this help\n"
              << "\n\n";
}


int main( int argc, char ** argv ) {
    if (pcl::console::find_argument (argc, argv, "-h") >= 0) {
        print_usage(argv[0]);
        return 0;
    }
    string dir;
    if ( pcl::console::parse(argc, argv, "-dir", dir) >= 0 ) {
        pcl::console::print_highlight( "Read mask image from %s\n", dir.c_str() );

        ros::init( argc, argv, "offline_recogniser" );
        ros::NodeHandle nh("~");
        OfflineRecogniser reco( nh, dir );
        reco.start_monitor();
        return 0;
    }
    return 0;
}




