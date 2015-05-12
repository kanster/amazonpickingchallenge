#include "include/helpfun/json_parser.hpp"

#include <apc_msgs/TargetRequest.h>

#include <apc_msgs/RecogniseALG.h>

#include <apc_msgs/DataPublish.h>
#include <apc_msgs/RecogStatus.h>
#include <apc_msgs/BinObjects.h>
#include <apc_msgs/Object.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>

using namespace std;


class DataPublisher{
private:
    ros::NodeHandle *nh_;

    //! published data on disk
    std::string dir_;
    int count_;

    //! srvs and topics names between ORS and request
    std::string target_srv_name_;
    std::string recog_srv_name_;
    std::string obj_topic_name_;

    //! publish topic names and publishers
    std::string xtion_rgb_topic_;
    std::string xtion_rgb_info_topic_;
    std::string xtion_depth_topic_;
    std::string xtion_depth_info_topic_;
    std::string xtion_cloud_topic_;
    std::string camera_rgb_topic_;
    std::string camera_rgb_info_topic_;

    ros::Publisher xtion_rgb_pub_;
    ros::Publisher xtion_rgb_info_pub_;
    ros::Publisher xtion_depth_pub_;
    ros::Publisher xtion_depth_info_pub_;
    ros::Publisher xtion_cloud_pub_;
    ros::Publisher camera_rgb_pub_;
    ros::Publisher camera_rgb_info_pub_;

    //! published sensor msgs
    sensor_msgs::Image xtion_rgb_msg_;
    sensor_msgs::CameraInfo xtion_rgb_info_msg_;
    sensor_msgs::Image xtion_depth_msg_;
    sensor_msgs::CameraInfo xtion_depth_info_msg_;
    sensor_msgs::PointCloud2 xtion_cloud_msg_;
    sensor_msgs::Image pg_rgb_msg_;
    sensor_msgs::CameraInfo pg_rgb_info_msg_;

    //! ORS caller
    ros::ServiceClient client_;

    //! ORS results server
    ros::ServiceServer recog_server_;

    //! ORS result topic subscriber
    ros::Subscriber obj_sub_;

    //! main publish thread
    boost::thread publish_thread_;

    //! use point cloud or not
    bool use_pointcloud_;

    // json file
    JSON json_;
    std::string json_filename_;
    map<string, vector<string> >  bin_contents_;
    vector<pair<string, string> > work_order_;

    //! srv mode
    int srv_mode_;

    //! online mode
    bool online_mode_;

    //! use pg or not
    bool use_pg_;

    //! callback function for online mode
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> without_pg_policy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> cloud_without_pg_policy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> with_pg_policy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::CameraInfo> cloud_with_pg_policy;
    boost::shared_ptr<message_filters::Synchronizer<without_pg_policy> > without_pg_sensor_sync_;
    boost::shared_ptr<message_filters::Synchronizer<cloud_without_pg_policy> > cloud_without_pg_sensor_sync_;
    boost::shared_ptr<message_filters::Synchronizer<with_pg_policy> > with_pg_sensor_sync_;
    boost::shared_ptr<message_filters::Synchronizer<cloud_with_pg_policy> > cloud_with_pg_sensor_sync_;


    string ori_xtion_rgb_topic_;
    string ori_xtion_rgb_info_topic_;
    string ori_xtion_depth_topic_;
    string ori_xtion_depth_info_topic_;
    string ori_xtion_cloud_topic_;
    string ori_pg_rgb_topic_;
    string ori_pg_rgb_info_topic_;

    message_filters::Subscriber<sensor_msgs::Image>         ori_xtion_rgb_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>    ori_xtion_rgb_info_sub_;
    message_filters::Subscriber<sensor_msgs::Image>         ori_xtion_depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>    ori_xtion_depth_info_sub_;
    message_filters::Subscriber<sensor_msgs::Image>         ori_pg_rgb_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>    ori_pg_rgb_info_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>   ori_xtion_cloud_sub_;



    //! sensor info data
    struct SensorData{
        cv::Mat xtion_rgb;
//        cv_bridge::CvImage xtion_rgb;
        cv::Mat xtion_depth;
        image_geometry::PinholeCameraModel xtion_rgb_model;
        image_geometry::PinholeCameraModel xtion_depth_model;
        bool use_pg;
        cv::Mat pg_rgb;
        image_geometry::PinholeCameraModel pg_rgb_model;
    };
    pcl::PointCloud<pcl::PointXYZRGB> xtion_cloud_;


    SensorData sensor_data_;
    SensorData * sensor_data_ptr_;
    bool sensor_empty_;
    boost::mutex sensor_mutex_;
    boost::condition_variable sensor_cond_;
    //! image for disp
    cv::Mat disp_image_;


private:
    void cloud_without_pg_callback(   const sensor_msgs::ImageConstPtr & xtion_rgb_msg,
                                      const sensor_msgs::CameraInfoConstPtr & xtion_rgb_info_msg,
                                      const sensor_msgs::PointCloud2ConstPtr & cloud_msg ) {

        ROS_INFO_ONCE( "Without point grey callback" );
        bool tmp_mask;

        {
            boost::mutex::scoped_lock lock( sensor_mutex_ );
            tmp_mask = sensor_empty_;
        }

        if(tmp_mask){
            pcl::fromROSMsg( *cloud_msg, xtion_cloud_ );

            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(xtion_rgb_msg, sensor_msgs::image_encodings::BGR8);
            }catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            sensor_data_.xtion_rgb = cv_ptr->image;


            // assign value for sensor data
            sensor_data_.xtion_rgb_model.fromCameraInfo( xtion_rgb_info_msg );
            for ( int y = 0; y < (int)xtion_cloud_.height; ++ y ) {
                for ( int x = 0; x < (int)xtion_cloud_.width; ++ x ) {
                    pcl::PointXYZRGB & pt = xtion_cloud_.points[y*xtion_cloud_.width+x];
                    sensor_data_.xtion_depth.at<unsigned short>(y,x) = static_cast<unsigned short>(0);
                    if ( !pcl_isinf(pt.x) && !pcl_isnan(pt.x) &&
                         !pcl_isinf(pt.y) && !pcl_isnan(pt.y) &&
                         !pcl_isinf(pt.z) && !pcl_isnan(pt.z) ) {
                        sensor_data_.xtion_depth.at<unsigned short>(y,x) = static_cast<unsigned short>(pt.z*1000.0);
                    }
                }
            }
            sensor_data_.use_pg = false;

            //disp
            disp_image_ = sensor_data_.xtion_rgb.clone();// cv_ptr->image.clone();

            cv::putText( disp_image_, "Press <s> to publish", cv::Point(20, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1 );
            cv::putText( disp_image_, "Put items: ", cv::Point(20, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1 );
            for ( int i = 0; i < (int)bin_contents_[work_order_[count_].first].size(); ++ i ) {
                cv::putText( disp_image_, bin_contents_[work_order_[count_].first][i], cv::Point(20, 70+i*20), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1 );
            }
            cv::imshow( "xtion_rgb_image", disp_image_ );
            char k = cv::waitKey(5);
            if ( k == 's' ) {
                {
                    boost::mutex::scoped_lock lock( sensor_mutex_ );
                    sensor_empty_ = false;
                }
                sensor_cond_.notify_one();
                ori_xtion_rgb_sub_.unsubscribe();
                ori_xtion_rgb_info_sub_.unsubscribe();
                ori_xtion_cloud_sub_.unsubscribe();
            }
            else if ( k == 'a' ||
                      k == 'b' ||
                      k == 'c' ||
                      k == 'd' ||
                      k == 'e' ||
                      k == 'f' ||
                      k == 'g' ||
                      k == 'h' ||
                      k == 'i' ||
                      k == 'j' ||
                      k == 'k' ||
                      k == 'l') {
                if ( k == 'a' )
                    count_ = 0;
                else if ( k == 'b' )
                    count_ = 1;
                else if ( k == 'c' )
                    count_ = 2;
                else if ( k == 'd' )
                    count_ = 3;
                else if ( k == 'e' )
                    count_ = 4;
                else if ( k == 'f' )
                    count_ = 5;
                else if ( k == 'g' )
                    count_ = 6;
                else if ( k == 'h' )
                    count_ = 7;
                else if ( k == 'i' )
                    count_ = 8;
                else if ( k == 'j' )
                    count_ = 9;
                else if ( k == 'k' )
                    count_ = 10;
                else if ( k == 'l' )
                    count_ = 11;
                {
                    boost::mutex::scoped_lock lock( sensor_mutex_ );
                    sensor_empty_ = false;
                }
                sensor_cond_.notify_one();
                ori_xtion_rgb_sub_.unsubscribe();
                ori_xtion_rgb_info_sub_.unsubscribe();
                ori_xtion_cloud_sub_.unsubscribe();
            }
        }
    }


    void cloud_with_pg_callback( const sensor_msgs::ImageConstPtr & xtion_rgb_msg,
                                 const sensor_msgs::CameraInfoConstPtr & xtion_rgb_info_msg,
                                 const sensor_msgs::PointCloud2ConstPtr & cloud_msg,
                                 const sensor_msgs::ImageConstPtr & pg_rgb_msg,
                                 const sensor_msgs::CameraInfoConstPtr & pg_rgb_info_msg ) {
        bool tmp_mask;
        {
            boost::mutex::scoped_lock lock( sensor_mutex_ );
            tmp_mask = sensor_empty_;
        }
        if(tmp_mask){
            pcl::fromROSMsg( *cloud_msg, xtion_cloud_ );

            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(xtion_rgb_msg, sensor_msgs::image_encodings::BGR8);
            }catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            sensor_data_.xtion_rgb = cv_ptr->image;

            cv_bridge::CvImagePtr pg_ptr;
            try {
                pg_ptr = cv_bridge::toCvCopy( pg_rgb_msg, sensor_msgs::image_encodings::BGR8 );
            } catch ( cv_bridge::Exception & e ) {
                ROS_ERROR( "cv_bridge exceptions: %s", e.what() );
                return;
            }
            sensor_data_.pg_rgb = pg_ptr->image;
            sensor_data_.use_pg = true;

            // assign value for sensor data
            sensor_data_.xtion_rgb_model.fromCameraInfo( xtion_rgb_info_msg );
            sensor_data_.pg_rgb_model.fromCameraInfo( pg_rgb_info_msg );


            for ( int y = 0; y < (int)xtion_cloud_.height; ++ y ) {
                for ( int x = 0; x < (int)xtion_cloud_.width; ++ x ) {
                    pcl::PointXYZRGB & pt = xtion_cloud_.points[y*xtion_cloud_.width+x];
                    sensor_data_.xtion_depth.at<unsigned short>(y,x) = static_cast<unsigned short>(0);
                    if ( !pcl_isinf(pt.x) && !pcl_isnan(pt.x) &&
                         !pcl_isinf(pt.y) && !pcl_isnan(pt.y) &&
                         !pcl_isinf(pt.z) && !pcl_isnan(pt.z) ) {
                        sensor_data_.xtion_depth.at<unsigned short>(y,x) = static_cast<unsigned short>(pt.z*1000.0);
                    }
                }
            }

            //disp
            disp_image_ = sensor_data_.xtion_rgb.clone();// cv_ptr->image.clone();

            cv::putText( disp_image_, "Press <s> to publish", cv::Point(20, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1 );
            cv::putText( disp_image_, "Put items: ", cv::Point(20, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1 );
            for ( int i = 0; i < (int)bin_contents_[work_order_[count_].first].size(); ++ i ) {
                cv::putText( disp_image_, bin_contents_[work_order_[count_].first][i], cv::Point(20, 70+i*20), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1 );
            }
            cv::imshow( "xtion_rgb_image", disp_image_ );
            cv::imshow( "pg_rgb_image", sensor_data_.pg_rgb );
            char k = cv::waitKey(5);
            if ( k == 's' ) {
                {
                    boost::mutex::scoped_lock lock( sensor_mutex_ );
                    sensor_empty_ = false;
                }
                sensor_cond_.notify_one();

                ori_xtion_rgb_sub_.unsubscribe();
                ori_xtion_rgb_info_sub_.unsubscribe();
                ori_xtion_cloud_sub_.unsubscribe();
                ori_pg_rgb_sub_.unsubscribe();
                ori_pg_rgb_info_sub_.unsubscribe();
            }
            else if ( k == 'a' ||
                      k == 'b' ||
                      k == 'c' ||
                      k == 'd' ||
                      k == 'e' ||
                      k == 'f' ||
                      k == 'g' ||
                      k == 'h' ||
                      k == 'i' ||
                      k == 'j' ||
                      k == 'k' ||
                      k == 'l') {
                if ( k == 'a' )
                    count_ = 0;
                else if ( k == 'b' )
                    count_ = 1;
                else if ( k == 'c' )
                    count_ = 2;
                else if ( k == 'd' )
                    count_ = 3;
                else if ( k == 'e' )
                    count_ = 4;
                else if ( k == 'f' )
                    count_ = 5;
                else if ( k == 'g' )
                    count_ = 6;
                else if ( k == 'h' )
                    count_ = 7;
                else if ( k == 'i' )
                    count_ = 8;
                else if ( k == 'j' )
                    count_ = 9;
                else if ( k == 'k' )
                    count_ = 10;
                else if ( k == 'l' )
                    count_ = 11;

                {
                    boost::mutex::scoped_lock lock( sensor_mutex_ );
                    sensor_empty_ = false;
                }
                sensor_cond_.notify_one();
                ori_xtion_rgb_sub_.unsubscribe();
                ori_xtion_rgb_info_sub_.unsubscribe();
                ori_xtion_cloud_sub_.unsubscribe();
                ori_pg_rgb_sub_.unsubscribe();
                ori_pg_rgb_info_sub_.unsubscribe();
            }
        }


    }



    void cam_model_to_msg( sensor_msgs::CameraInfo & msg, image_geometry::PinholeCameraModel & model ) {
        msg.binning_x = model.binningX();
        msg.binning_y = model.binningY();
        msg.D = model.distortionCoeffs();
        msg.P[0] = model.projectionMatrix().at<double>(0,0);
        msg.P[1] = model.projectionMatrix().at<double>(0,1);
        msg.P[2] = model.projectionMatrix().at<double>(0,2);
        msg.P[3] = model.projectionMatrix().at<double>(0,3);
        msg.P[4] = model.projectionMatrix().at<double>(1,0);
        msg.P[5] = model.projectionMatrix().at<double>(1,1);
        msg.P[6] = model.projectionMatrix().at<double>(1,2);
        msg.P[7] = model.projectionMatrix().at<double>(1,3);
        msg.P[8] = model.projectionMatrix().at<double>(2,0);
        msg.P[9] = model.projectionMatrix().at<double>(2,1);
        msg.P[10] = model.projectionMatrix().at<double>(2,2);
        msg.P[11] = model.projectionMatrix().at<double>(2,3);
    }


public:
    DataPublisher( ros::NodeHandle & nh )
        : sensor_empty_(true){
        nh_ = & nh;
        count_ = 0;

        // set size of cloud
        xtion_cloud_.width  = 640;
        xtion_cloud_.height = 480;
        xtion_cloud_.is_dense = true;
        xtion_cloud_.points.resize( 640*480 );

        // alloc sensor data
        sensor_data_.xtion_rgb = cv::Mat( 480, 640, CV_8UC3 );
        sensor_data_.xtion_depth = cv::Mat( 480, 640, CV_16UC1 );

        nh_->param<std::string>("json", json_filename_, "/tmp/apc/amazon.json");
        nh_->param<bool>("use_cloud", use_pointcloud_, false);
        nh_->param<bool>("online", online_mode_, true);
        nh_->param<bool>("use_pg", use_pg_, false);

//        if ( use_pg_ == true )
//            sensor_data_.pg_rgb = cv::Mat( 960, 1280, CV_8UC1 );

        ROS_INFO_ONCE( "json file from %s", json_filename_.c_str());
        json_.from_file( json_filename_ );
        bin_contents_   = json_.bin_contents_;
        work_order_     = json_.work_order_;

        nh_->param<int>("srv_mode", srv_mode_, 1);

        cv::namedWindow( "xtion_rgb_image" ); cv::moveWindow( "xtion_rgb_image", 0, 0 );

        if ( online_mode_ == true ) {
            ROS_INFO( "Use online mode" );
            // named window for call back visualization
            nh_->param<std::string>("ori_xtion_rgb_image", ori_xtion_rgb_topic_, "/camera/rgb/image_color");
            nh_->param<std::string>("ori_xtion_rgb_info", ori_xtion_rgb_info_topic_, "/camera/rgb/camera_info");
            nh_->param<std::string>("ori_xtion_cloud", ori_xtion_cloud_topic_, "/camera/depth_registered/points");
            ROS_INFO( "Subscribe from %s", ori_xtion_rgb_topic_.c_str() );
            ROS_INFO( "Subscribe from %s", ori_xtion_rgb_info_topic_.c_str() );
            ROS_INFO( "Subscribe from %s", ori_xtion_cloud_topic_.c_str() );

//            nh_->param<std::string>("ori_xtion_depth_image", ori_xtion_depth_topic_, "/camera/depth/image_raw");
//            nh_->param<std::string>("ori_xtion_depth_info", ori_xtion_depth_info_topic_, "/camera/depth/camera_info");
//            ROS_INFO( "Subscribe from %s", ori_xtion_depth_topic_.c_str() );
//            ROS_INFO( "Subscribe from %s", ori_xtion_depth_info_topic_.c_str() );


            // use point grey or not
            if ( use_pg_ == false ) {
                ROS_INFO( "Do NOT use point grey high resolution camera" );
                // subscribe to specific topics
                ori_xtion_rgb_sub_.subscribe( *nh_, ori_xtion_rgb_topic_, 1);
                ori_xtion_rgb_info_sub_.subscribe( *nh_, ori_xtion_rgb_info_topic_, 1);
                ori_xtion_cloud_sub_.subscribe( *nh_, ori_xtion_cloud_topic_, 1 );
                cloud_without_pg_sensor_sync_.reset( new message_filters::Synchronizer<cloud_without_pg_policy>( cloud_without_pg_policy(10), ori_xtion_rgb_sub_, ori_xtion_rgb_info_sub_, ori_xtion_cloud_sub_ ) );
                cloud_without_pg_sensor_sync_->registerCallback( boost::bind( &DataPublisher::cloud_without_pg_callback, this, _1, _2, _3 ) );

            }
            else if ( use_pg_ == true ) {
                cv::namedWindow( "pg_rgb_image" ); cv::moveWindow( "pg_rgb_image", 640, 0 );

                ROS_INFO( "Do use point grey high resolution camera" );
                nh_->param<std::string>("ori_pg_rgb_image", ori_pg_rgb_topic_, "/rgb_image");
                nh_->param<std::string>("ori_pg_rgb_info", ori_pg_rgb_info_topic_, "/camera_info");
                ROS_INFO( "Subscribe from %s", ori_pg_rgb_topic_.c_str() );
                ROS_INFO( "Subscribe from %s", ori_pg_rgb_info_topic_.c_str() );

                // subscribe to specific topics
                ori_xtion_rgb_sub_.subscribe( *nh_, ori_xtion_rgb_topic_, 1);
                ori_xtion_rgb_info_sub_.subscribe( *nh_, ori_xtion_rgb_info_topic_, 1);
                ori_xtion_cloud_sub_.subscribe( *nh_, ori_xtion_cloud_topic_, 1 );
                ori_pg_rgb_sub_.subscribe( *nh_, ori_pg_rgb_topic_, 1);
                ori_pg_rgb_info_sub_.subscribe( *nh_, ori_pg_rgb_info_topic_, 1);
                cloud_with_pg_sensor_sync_.reset( new message_filters::Synchronizer<cloud_with_pg_policy>(cloud_with_pg_policy(10), ori_xtion_rgb_sub_, ori_xtion_rgb_info_sub_, ori_xtion_cloud_sub_, ori_pg_rgb_sub_, ori_pg_rgb_info_sub_) );
                cloud_with_pg_sensor_sync_->registerCallback( boost::bind( &DataPublisher::cloud_with_pg_callback, this, _1, _2, _3, _4, _5 ) );
            }

            // srv mode
            if ( srv_mode_ == 1 ) {
                ROS_INFO( "UTS service mode, non block" );
                nh_->param<std::string>("object_topic_name", obj_topic_name_, "/object_poses");
                ROS_INFO("publish object recognition results to %s", obj_topic_name_.c_str());
                nh_->param<std::string>("object_srv_name", recog_srv_name_, "/recog_publish_srv");
                ROS_INFO("notification server %s to obtain results", recog_srv_name_.c_str());
                nh_->param<std::string>("target_srv_name", target_srv_name_, "/data_publish_srv");
                ROS_INFO("server %s to start recognition", target_srv_name_.c_str());

                recog_server_ = nh.advertiseService( recog_srv_name_, &DataPublisher::recog_srv_callback, this);
                obj_sub_ = nh.subscribe( obj_topic_name_, 1, &DataPublisher::recog_callback, this );

                client_ = nh.serviceClient<apc_msgs::TargetRequest>(target_srv_name_);

                nh_->param<std::string>("xtion_rgb_image", xtion_rgb_topic_, "/camera/lowres_rgb/image");
                nh_->param<std::string>("xtion_depth_image", xtion_depth_topic_, "/camera/depth/image");
                nh_->param<std::string>("xtion_rgb_info", xtion_rgb_info_topic_, "/camera/lowres_rgb/camera_info");
                nh_->param<std::string>("pg_rgb_image", camera_rgb_topic_, "/camera/highres_rgb/image");
                nh_->param<std::string>("pg_rgb_info", camera_rgb_info_topic_, "/camera/highres_rgb/camera_info");


                xtion_rgb_pub_ = nh.advertise<sensor_msgs::Image>(xtion_rgb_topic_, 1);
                xtion_rgb_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(xtion_rgb_info_topic_, 1);
                xtion_depth_pub_ = nh.advertise<sensor_msgs::Image>(xtion_depth_topic_, 1);
                camera_rgb_pub_ = nh.advertise<sensor_msgs::Image>(camera_rgb_topic_, 1);
                camera_rgb_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(camera_rgb_info_topic_, 1);

                if ( use_pointcloud_ == true ) {
                    nh_->param<std::string>("xtion_points", xtion_cloud_topic_, "/camera/points");
                    xtion_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(xtion_cloud_topic_, 1);
                }
            }
            else if ( srv_mode_ == 2 ) {
                ROS_INFO( "ZJU service mode, block" );
                nh_->param<std::string>("target_srv_name", target_srv_name_, "/data_publish_srv");
                client_ = nh.serviceClient<apc_msgs::RecogniseALG>(target_srv_name_);
            }
        }
        else if ( online_mode_ == false ) {
            nh_->param<std::string>("data_dir", dir_, "/tmp/apc/data/");
            ROS_INFO( "Use offline mode" );
            if ( srv_mode_ == 1 ) {
                nh_->param<std::string>("object_topic_name", obj_topic_name_, "/object_poses");
                ROS_INFO("publish object recognition results to %s", obj_topic_name_.c_str());
                nh_->param<std::string>("object_srv_name", recog_srv_name_, "/recog_publish_srv");
                ROS_INFO("notification server %s to obtain results", recog_srv_name_.c_str());
                nh_->param<std::string>("target_srv_name", target_srv_name_, "/data_publish_srv");
                ROS_INFO("server %s to start recognition", target_srv_name_.c_str());

                recog_server_ = nh.advertiseService( recog_srv_name_, &DataPublisher::recog_srv_callback, this);
                obj_sub_ = nh.subscribe( obj_topic_name_, 1, &DataPublisher::recog_callback, this );

                client_ = nh.serviceClient<apc_msgs::TargetRequest>(target_srv_name_);

                nh_->param<std::string>("xtion_rgb_image", xtion_rgb_topic_, "/camera/lowres_rgb/image");
                nh_->param<std::string>("xtion_depth_image", xtion_depth_topic_, "/camera/depth/image");
                nh_->param<std::string>("xtion_rgb_info", xtion_rgb_info_topic_, "/camera/lowres_rgb/camera_info");
                nh_->param<std::string>("pg_rgb_image", camera_rgb_topic_, "/camera/highres_rgb/image");
                nh_->param<std::string>("pg_rgb_info", camera_rgb_info_topic_, "/camera/highres_rgb/camera_info");


                xtion_rgb_pub_ = nh.advertise<sensor_msgs::Image>(xtion_rgb_topic_, 1);
                xtion_rgb_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(xtion_rgb_info_topic_, 1);
                xtion_depth_pub_ = nh.advertise<sensor_msgs::Image>(xtion_depth_topic_, 1);
                camera_rgb_pub_ = nh.advertise<sensor_msgs::Image>(camera_rgb_topic_, 1);
                camera_rgb_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(camera_rgb_info_topic_, 1);

                if ( use_pointcloud_ == true ) {
                    nh_->param<std::string>("xtion_points", xtion_cloud_topic_, "/camera/points");
                    xtion_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(xtion_cloud_topic_, 1);
                }
            }
            else if ( srv_mode_ == 2 ) {
                nh_->param<std::string>("target_srv_name", target_srv_name_, "/data_publish_srv");
                client_ = nh.serviceClient<apc_msgs::RecogniseALG>(target_srv_name_);
            }
        }
        publish_thread_ = boost::thread(boost::bind(&DataPublisher::publisher, this));
    }

    // object recognition results subscribe
    void recog_callback( const apc_msgs::BinObjectsConstPtr & objs_msg ) {
        std::cout << "Results from bin " << objs_msg->bin_id << "\n";
        for ( int i = 0; i < (int)objs_msg->items_in_bin.size(); ++ i ) {
            apc_msgs::Object obj = objs_msg->items_in_bin[i];
            std::cout << "    " << obj.name << ": position: [" << obj.pose.position.x << ", " << obj.pose.position.y << ", " << obj.pose.position.z << "]" << " and orientation [" << obj.pose.orientation.w << ", " << obj.pose.orientation.x << ", " << obj.pose.orientation.y << ", " << obj.pose.orientation.z  << "]\n";
        }
    }

    // service callback in unblock mode
    bool recog_srv_callback( apc_msgs::RecogStatus::Request & req,
                             apc_msgs::RecogStatus::Response & resp ) {
        ROS_INFO("[recog_srv_callback] recog compeletion request received");
        if ( req.recog == true )
            ROS_INFO( "Object recognition successed" );

        resp.pub = true;
        return true;
    }




    void publisher() {

        while ( nh_->ok() ) {

            for ( count_ = 0; count_ < (int)bin_contents_.size(); ++  count_ ) {
                cv_bridge::CvImage xtion_rgb_cv;
                cv_bridge::CvImage xtion_depth_cv;
                cv_bridge::CvImage pg_rgb_cv;

                sensor_msgs::Image xtion_rgb_msg;
                sensor_msgs::Image xtion_depth_msg;
                sensor_msgs::Image pg_rgb_msg;
                sensor_msgs::CameraInfo xtion_rgb_info_msg;
                sensor_msgs::CameraInfo pg_rgb_info_msg;


                string bin_id = work_order_[count_].first;

                if ( online_mode_ == true ) {
                    boost::mutex::scoped_lock lock(sensor_mutex_);
                    while(sensor_empty_) {
                        sensor_cond_.wait( lock );
                    }


                    xtion_rgb_cv.image = sensor_data_.xtion_rgb;
                    xtion_rgb_cv.encoding = sensor_msgs::image_encodings::BGR8;
                    xtion_rgb_cv.toImageMsg( xtion_rgb_msg );

                    // generate xtion depth msg
                    xtion_depth_cv.image  = sensor_data_.xtion_depth;
                    xtion_depth_cv.encoding = sensor_msgs::image_encodings::MONO16;
                    xtion_depth_cv.toImageMsg( xtion_depth_msg );


                    cam_model_to_msg( xtion_rgb_info_msg, sensor_data_.xtion_rgb_model );
                    xtion_rgb_info_msg.width = xtion_depth_msg.width;
                    xtion_rgb_info_msg.height = xtion_depth_msg.height;

                    // optional pg image
                    if ( use_pg_ == true ) {
                        // generate pg rgb msg
                        pg_rgb_cv.image = sensor_data_.pg_rgb;
                        pg_rgb_cv.encoding = sensor_msgs::image_encodings::BGR8;
                        pg_rgb_cv.toImageMsg( pg_rgb_msg );

                        // generate pg rgb camera info
                        cam_model_to_msg( pg_rgb_info_msg, sensor_data_.pg_rgb_model );
                        pg_rgb_info_msg.width = pg_rgb_msg.width;
                        pg_rgb_info_msg.height = pg_rgb_msg.height;
                    }
                }
                else
                {
                    // generate sensor information msgs
                    // generate filenames
                    std::string xtion_rgb_name = dir_ + "/xtion_rgb_" + boost::lexical_cast<std::string>(count_+1) + ".png";
                    std::string xtion_depth_name = dir_ + "/xtion_depth_" + boost::lexical_cast<std::string>(count_+1) + ".png";  // for depth images, .png
                    std::string xtion_rgb_info_name = dir_ + "/xtion_rgb_info_" + boost::lexical_cast<std::string>(count_+1) + ".yml";

                    // generate xtion rgb msg
                    xtion_rgb_cv.image = cv::imread( xtion_rgb_name, CV_LOAD_IMAGE_COLOR );
                    xtion_rgb_cv.encoding = sensor_msgs::image_encodings::BGR8;
                    xtion_rgb_cv.toImageMsg( xtion_rgb_msg );

                    // generate xtion depth msg
                    xtion_depth_cv.image  = cv::imread( xtion_depth_name, CV_LOAD_IMAGE_ANYDEPTH );
                    xtion_depth_cv.encoding = sensor_msgs::image_encodings::MONO16;
                    xtion_depth_cv.toImageMsg( xtion_depth_msg );

//                    cv::imshow( "rgb", xtion_rgb_cv.image );
//                    cv::imshow( "depth", xtion_depth_cv.image );
//                    cv::waitKey(0);

                    // generate xtion rgb camera info
                    int idx = 0;
                    cv::FileStorage xtion_rgb_fs(xtion_rgb_info_name, cv::FileStorage::READ);
                    cv::Mat xtion_rgb_proj, xtion_rgb_dist;
                    xtion_rgb_fs["Projection"] >> xtion_rgb_proj;
                    xtion_rgb_fs["Distortion"] >> xtion_rgb_dist;
                    boost::array<double, 12> xtion_rgb_boost_p;
                    for ( int y = 0; y < xtion_rgb_proj.rows; ++ y ) {
                        for ( int x = 0; x < xtion_rgb_proj.cols; ++ x ) {
                            xtion_rgb_boost_p[idx] = xtion_rgb_proj.at<double>(y,x);
                            idx ++;
                        }
                    }
                    idx = 0;
                    std::vector<double> xtion_rgb_boost_d;
                    for ( int y = 0; y < xtion_rgb_dist.rows; ++ y )
                        for ( int x = 0; x < xtion_rgb_dist.cols; ++ x )
                            xtion_rgb_boost_d.push_back( xtion_rgb_dist.at<double>(y,x) );
                    xtion_rgb_info_msg.height = xtion_rgb_cv.image.rows;
                    xtion_rgb_info_msg.width  = xtion_rgb_cv.image.cols;
                    xtion_rgb_info_msg.P = xtion_rgb_boost_p;
                    xtion_rgb_info_msg.D = xtion_rgb_boost_d;

                    // optional pg image
                    if ( use_pg_ == true ) {
                        std::string camera_rgb_name = dir_ + "/camera_rgb_" + boost::lexical_cast<std::string>(count_+1) + ".png";
                        std::string camera_rgb_info_name = dir_ + "/camera_rgb_info_" + boost::lexical_cast<std::string>(count_+1) + ".yml";

                        // generate pg rgb msg
                        pg_rgb_cv.image = cv::imread( camera_rgb_name, CV_LOAD_IMAGE_COLOR );
                        pg_rgb_cv.encoding = sensor_msgs::image_encodings::BGR8;
                        pg_rgb_cv.toImageMsg( pg_rgb_msg );

                        // generate pg rgb camera info
                        cv::FileStorage camera_rgb_fs(camera_rgb_info_name, cv::FileStorage::READ);
                        cv::Mat camera_rgb_proj, camera_rgb_dist;
                        camera_rgb_fs["Projection"] >> camera_rgb_proj;
                        camera_rgb_fs["Distortion"] >> camera_rgb_dist;
                        boost::array<double, 12> camera_rgb_boost_p;
                        for ( int y = 0; y < camera_rgb_proj.rows; ++ y ) {
                            for ( int x = 0; x < camera_rgb_proj.cols; ++ x ) {
                                camera_rgb_boost_p[idx] = camera_rgb_proj.at<double>(y,x);
                                idx ++;
                            }
                        }
                        idx = 0;
                        std::vector<double> camera_rgb_boost_d;
                        for ( int y = 0; y < camera_rgb_dist.rows; ++ y )
                            for ( int x = 0; x < camera_rgb_dist.cols; ++ x )
                                camera_rgb_boost_d.push_back( camera_rgb_dist.at<double>(y,x) );
                        pg_rgb_info_msg.height = pg_rgb_cv.image.rows;
                        pg_rgb_info_msg.width  = pg_rgb_cv.image.cols;
                        pg_rgb_info_msg.P = camera_rgb_boost_p;
                        pg_rgb_info_msg.D = camera_rgb_boost_d;
                    }
                }

                if ( srv_mode_ == 1 ) {
                    apc_msgs::TargetRequest target_req_srv;

                    // generate target request
                    target_req_srv.request.BinID = bin_id;
                    target_req_srv.request.ObjectName  = work_order_[count_].second;
                    vector<string> bin_content = bin_contents_[work_order_[count_].first];
                    for ( int i = 0; i < (int)bin_content.size(); ++ i ) {
                        target_req_srv.request.BinContents.push_back( bin_content[i] );
                    }
                    vector<int8_t> removed_indices;
                    target_req_srv.request.RemovedObjectIndices = removed_indices;
                    target_req_srv.request.use_cloud = use_pointcloud_;
                    target_req_srv.request.use_pg = use_pg_;
                    ROS_INFO( "request object name %s, sending ...",  target_req_srv.request.ObjectName.c_str() );
                   if ( client_.call( target_req_srv ) ) {
                        ROS_INFO( "return status: %s", target_req_srv.response.Found? "true" : "false" );
                    }
                    else {
                        ROS_ERROR( "Target object: %s, failed to call service target_object", target_req_srv.request.ObjectName.c_str() );
                    }

                    sleep(1);

                    xtion_rgb_pub_.publish( xtion_rgb_msg );
                    xtion_rgb_info_pub_.publish( xtion_rgb_info_msg );
                    xtion_depth_pub_.publish( xtion_depth_msg );
                    if ( use_pg_ == true ) {
                        camera_rgb_pub_.publish( pg_rgb_msg );
                        camera_rgb_info_pub_.publish( pg_rgb_info_msg );
                    }
                }
                else if (srv_mode_ == 2) {
                    apc_msgs::RecogniseALG target_req_srv;
                    target_req_srv.request.bin_id = bin_id;
                    target_req_srv.request.ObjectName  = work_order_[count_].second;
                    vector<string> bin_content = bin_contents_[work_order_[count_].first];
                    for ( int i = 0; i < (int)bin_content.size(); ++ i ) {
                        target_req_srv.request.BinContents.push_back( bin_content[i] );
                    }
                    vector<int8_t> removed_indices;
                    target_req_srv.request.RemovedObjectIndices = removed_indices;

                    target_req_srv.request.xtion_rgb_image = xtion_rgb_msg;
                    target_req_srv.request.xtion_depth_image = xtion_depth_msg;
                    target_req_srv.request.xtion_rgb_info = xtion_rgb_info_msg;
                    target_req_srv.request.use_pg = use_pg_;
                    if ( use_pg_ == true ) {
                        target_req_srv.request.pg_rgb_image = pg_rgb_msg;
                        target_req_srv.request.pg_rgb_info = pg_rgb_info_msg;
                    }

                    ROS_INFO( "request object name %s, sending ...", target_req_srv.request.ObjectName.c_str() );
                   if ( client_.call( target_req_srv ) ) {
                        ROS_INFO( "return status: %s", target_req_srv.response.found? "true" : "false" );
                    }
                    else {
                        ROS_ERROR( "Target object: %s, failed to call service target_object", target_req_srv.request.ObjectName.c_str() );
                    }
                }
                if ( online_mode_ == true ) {
                    ori_xtion_rgb_sub_.subscribe( *nh_, ori_xtion_rgb_topic_, 1);
                    ori_xtion_rgb_info_sub_.subscribe( *nh_, ori_xtion_rgb_info_topic_, 1);
                    ori_xtion_cloud_sub_.subscribe( *nh_, ori_xtion_cloud_topic_, 1 );
                    if ( use_pg_ == true ) {
                        ori_pg_rgb_sub_.subscribe( *nh_, ori_pg_rgb_topic_, 1);
                        ori_pg_rgb_info_sub_.subscribe( *nh_, ori_pg_rgb_info_topic_, 1);
                    }

                    if ( use_pg_ == false ) {
                        cloud_without_pg_sensor_sync_.reset( new message_filters::Synchronizer<cloud_without_pg_policy>( cloud_without_pg_policy(10), ori_xtion_rgb_sub_, ori_xtion_rgb_info_sub_, ori_xtion_cloud_sub_ ) );
                        cloud_without_pg_sensor_sync_->registerCallback( boost::bind( &DataPublisher::cloud_without_pg_callback, this, _1, _2, _3 ) );
                    }
                    else {
                        cloud_with_pg_sensor_sync_.reset( new message_filters::Synchronizer<cloud_with_pg_policy>(cloud_with_pg_policy(10), ori_xtion_rgb_sub_, ori_xtion_rgb_info_sub_, ori_xtion_cloud_sub_, ori_pg_rgb_sub_, ori_pg_rgb_info_sub_) );
                        cloud_with_pg_sensor_sync_->registerCallback( boost::bind( &DataPublisher::cloud_with_pg_callback, this, _1, _2, _3, _4, _5 ) );
                    }

                    sleep(1);

                    sensor_mutex_.lock();
                    sensor_empty_ = true;
                    sensor_mutex_.unlock();
                }
            }
        }
    }

    ~DataPublisher(){}
};



int ms_sleep( unsigned long ms ) {
    struct timespec req = {0};
    time_t sec = (int)(ms/1000);
    ms -= sec*1000;
    req.tv_sec  = sec;
    req.tv_nsec = ms*1000000L;
    while ( nanosleep( &req, &req ) == -1 )
        continue;
    return 1;
}


int main( int argc, char ** argv ) {
    ros::init( argc, argv, "data_publisher" );
    ros::NodeHandle nh("~");
    DataPublisher publisher( nh );
    ros::spin();
    return 1;
}


