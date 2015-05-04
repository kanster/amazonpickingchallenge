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

    //! recognition completed mutex and condition
    bool recog_completed_;
    boost::mutex recog_completed_mutex_;
    boost::condition_variable recog_completed_cond_;

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
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> with_pg_policy;
    boost::shared_ptr<message_filters::Synchronizer<without_pg_policy> > without_pg_sensor_sync_;
    boost::shared_ptr<message_filters::Synchronizer<with_pg_policy> > with_pg_sensor_sync_;

    string ori_xtion_rgb_topic_;
    string ori_xtion_rgb_info_topic_;
    string ori_xtion_depth_topic_;
    string ori_xtion_depth_info_topic_;
    string ori_pg_rgb_topic_;
    string ori_pg_rgb_info_topic_;

    message_filters::Subscriber<sensor_msgs::Image>         ori_xtion_rgb_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>    ori_xtion_rgb_info_sub_;
    message_filters::Subscriber<sensor_msgs::Image>         ori_xtion_depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>    ori_xtion_depth_info_sub_;
    message_filters::Subscriber<sensor_msgs::Image>         ori_pg_rgb_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo>    ori_pg_rgb_info_sub_;



    //! sensor info data
    struct SensorData{
//        cv::Mat xtion_rgb;
        cv_bridge::CvImagePtr xtion_rgb;
        cv_bridge::CvImagePtr xtion_depth;
        image_geometry::PinholeCameraModel xtion_rgb_model;
        image_geometry::PinholeCameraModel xtion_depth_model;
        bool use_pg;
        cv_bridge::CvImagePtr  pg_rgb;
        image_geometry::PinholeCameraModel pg_rgb_model;
    };
    SensorData sensor_data_;
    SensorData * sensor_data_ptr_;
    bool sensor_empty_;
    boost::mutex sensor_mutex_;
    boost::condition_variable sensor_cond_;
    //! image for disp
    cv::Mat disp_image_;

private:
//    void without_pg_callback( const sensor_msgs::ImageConstPtr & xtion_rgb_msg,
//                              const sensor_msgs::CameraInfoConstPtr & xtion_rgb_info_msg,
//                              const sensor_msgs::ImageConstPtr & xtion_depth_msg ) {
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
//        pcl::fromROSMsg( *cloud_msg, *cloud_ );

//        ROS_INFO_ONCE( "Without point grey callback" );
//        SensorData * data = &sensor_data_;
//        // assign value for sensor data
//        data->xtion_rgb = cv_bridge::toCvCopy(xtion_rgb_msg, sensor_msgs::image_encodings::BGR8);
//        data->xtion_rgb_model.fromCameraInfo( xtion_rgb_info_msg );
//        cv::Mat cloud_depth( cloud_->height, cloud_->width, CV_16UC1, cv::Scalar(0) );
//        cv::Mat disp_depth( cloud_->height, cloud_->width, CV_32FC1, cv::Scalar(0) );

//        for ( int y = 0; y < (int)cloud->height; ++ y ) {
//            for ( int x = 0; x < (int)cloud->width; ++ x ) {
//                pcl::PointXYZRGB & pt = cloud->points[y*cloud->width+x];
//                if ( !pcl_isinf(pt.x) && !pcl_isnan(pt.x) &&
//                     !pcl_isinf(pt.y) && !pcl_isnan(pt.y) &&
//                     !pcl_isinf(pt.z) && !pcl_isnan(pt.z) ) {
//                    cloud_depth_.at<unsigned short>(y,x) = static_cast<unsigned short>(pt.z*1000.0);
//                    disp_depth_.at<float>(y,x) = static_cast<float>(pt.z);
//                }
//            }
//        }

//    }


    void without_pg_callback( const sensor_msgs::ImageConstPtr & xtion_rgb_msg,
                              const sensor_msgs::CameraInfoConstPtr & xtion_rgb_info_msg,
                              const sensor_msgs::ImageConstPtr & xtion_depth_msg,
                              const sensor_msgs::CameraInfoConstPtr & xtion_depth_info_msg ) {
        ROS_INFO_ONCE( "Without point grey callback" );
        SensorData * data = &sensor_data_;
        data->xtion_rgb = cv_bridge::toCvCopy(xtion_rgb_msg, sensor_msgs::image_encodings::BGR8);
        data->xtion_depth = cv_bridge::toCvCopy( xtion_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1 );
        data->xtion_rgb_model.fromCameraInfo( xtion_rgb_info_msg );
        data->xtion_depth_model.fromCameraInfo( xtion_depth_info_msg );
        // set sensor data info
        data->use_pg = false;
        disp_image_ = data->xtion_rgb->image.clone();// cv_ptr->image.clone();
        cv::putText( disp_image_, "Press <s> to publish", cv::Point(20, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1 );
        cv::putText( disp_image_, "Put items: ", cv::Point(20, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1 );
        for ( int i = 0; i < (int)bin_contents_[work_order_[count_].first].size(); ++ i ) {
            cv::putText( disp_image_, bin_contents_[work_order_[count_].first][i], cv::Point(20, 70+i*20), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1 );
        }
        cv::imshow( "rgb_image", disp_image_ );
        char k = cv::waitKey(5);
        if ( k == 's' ) {
            {
                boost::mutex::scoped_lock lock( sensor_mutex_ );
                sensor_empty_ = false;
            }
            sensor_cond_.notify_one();

            {
                boost::mutex::scoped_lock lock( recog_completed_mutex_ );
                recog_completed_ = false;
            }
        }
        {
            boost::mutex::scoped_lock lock( recog_completed_mutex_ );
            while ( !recog_completed_ )
                recog_completed_cond_.wait( lock );
        }
    }


    void with_pg_callback( const sensor_msgs::ImageConstPtr & xtion_rgb_msg,
                           const sensor_msgs::CameraInfoConstPtr & xtion_rgb_info_msg,
                           const sensor_msgs::ImageConstPtr & xtion_depth_msg,
                           const sensor_msgs::CameraInfoConstPtr & xtion_depth_info_msg,
                           const sensor_msgs::ImageConstPtr & pg_rgb_msg,
                           const sensor_msgs::CameraInfoConstPtr & pg_rgb_info_msgs) {
        ROS_INFO( "With point grey callback" );
        SensorData * data = sensor_data_ptr_;
        data->xtion_rgb   = cv_bridge::toCvCopy( xtion_rgb_msg, sensor_msgs::image_encodings::BGR8 );
        data->xtion_depth = cv_bridge::toCvCopy( xtion_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1 );
        data->pg_rgb      = cv_bridge::toCvCopy( pg_rgb_msg, sensor_msgs::image_encodings::BGR8 );
        data->xtion_rgb_model.fromCameraInfo( xtion_rgb_info_msg );
        data->xtion_depth_model.fromCameraInfo( xtion_depth_info_msg );
        data->pg_rgb_model.fromCameraInfo(pg_rgb_info_msgs);
        data->use_pg = true;
        disp_image_ = data->xtion_rgb->image.clone();
        cv::putText( disp_image_, "Press <s> to publish", cv::Point(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.2, cv::Scalar(255, 0, 0), 0.1 );
        cv::imshow( "rgb_image", disp_image_ );
        char k = cv::waitKey(5);
        if ( k == 's' ) {
            {
                boost::mutex::scoped_lock lock( sensor_mutex_ );
                sensor_empty_ = false;
            }
            sensor_cond_.notify_one();
        }
    }

    void cam_model_to_msg( sensor_msgs::CameraInfo & msg, image_geometry::PinholeCameraModel & model ) {
        msg.binning_x = model.binningX();
        msg.binning_y = model.binningY();
        msg.D = model.distortionCoeffs();
        msg.P[0] = cv::Mat(model.projectionMatrix()).at<double>(0,0);
        msg.P[1] = cv::Mat(model.projectionMatrix()).at<double>(0,1);
        msg.P[2] = cv::Mat(model.projectionMatrix()).at<double>(0,2);
        msg.P[3] = cv::Mat(model.projectionMatrix()).at<double>(0,3);
        msg.P[4] = cv::Mat(model.projectionMatrix()).at<double>(1,0);
        msg.P[5] = cv::Mat(model.projectionMatrix()).at<double>(1,1);
        msg.P[6] = cv::Mat(model.projectionMatrix()).at<double>(1,2);
        msg.P[7] = cv::Mat(model.projectionMatrix()).at<double>(1,3);
        msg.P[8] = cv::Mat(model.projectionMatrix()).at<double>(2,0);
        msg.P[9] = cv::Mat(model.projectionMatrix()).at<double>(2,1);
        msg.P[10] = cv::Mat(model.projectionMatrix()).at<double>(2,2);
        msg.P[11] = cv::Mat(model.projectionMatrix()).at<double>(2,3);
    }

public:
    DataPublisher( ros::NodeHandle & nh )
        : sensor_empty_(true)
        , recog_completed_(true){
        nh_ = & nh;

        nh_->param<std::string>("json", json_filename_, "/tmp/apc/amazon.json");
        nh_->param<bool>("use_cloud", use_pointcloud_, false);
        nh_->param<bool>("online", online_mode_, true);
        nh_->param<bool>("use_pg", use_pg_, false);

        ROS_INFO_ONCE( "json file from %s", json_filename_.c_str());
        json_.from_file( json_filename_ );
        bin_contents_   = json_.bin_contents_;
        work_order_     = json_.work_order_;

        nh_->param<int>("srv_mode", srv_mode_, 1);

        if ( online_mode_ == true ) {
            ROS_INFO( "Use online mode" );
            // named window for call back visualization
            cv::namedWindow( "rgb_image" );
            nh_->param<std::string>("ori_xtion_rgb_image", ori_xtion_rgb_topic_, "/camera/rgb/image_color");
            nh_->param<std::string>("ori_xtion_rgb_info", ori_xtion_rgb_info_topic_, "/camera/rgb/camera_info");
            nh_->param<std::string>("ori_xtion_depth_image", ori_xtion_depth_topic_, "/camera/depth/image_raw");
            nh_->param<std::string>("ori_xtion_depth_info", ori_xtion_depth_info_topic_, "/camera/depth/camera_info");
            ROS_INFO( "Subscribe from %s", ori_xtion_rgb_topic_.c_str() );
            ROS_INFO( "Subscribe from %s", ori_xtion_rgb_info_topic_.c_str() );
            ROS_INFO( "Subscribe from %s", ori_xtion_depth_topic_.c_str() );
            ROS_INFO( "Subscribe from %s", ori_xtion_depth_info_topic_.c_str() );


            // use point grey or not
            if ( use_pg_ == false ) {
                ROS_INFO( "Do NOT use point grey high resolution camera" );
                // subscribe to specific topics
                ori_xtion_rgb_sub_.subscribe( *nh_, ori_xtion_rgb_topic_, 1);
                ori_xtion_rgb_info_sub_.subscribe( *nh_, ori_xtion_rgb_info_topic_, 1);
                ori_xtion_depth_sub_.subscribe( *nh_, ori_xtion_depth_topic_, 1);
                ori_xtion_depth_info_sub_.subscribe( *nh_, ori_xtion_depth_info_topic_, 1);
                without_pg_sensor_sync_.reset( new message_filters::Synchronizer<without_pg_policy>(without_pg_policy(10), ori_xtion_rgb_sub_, ori_xtion_rgb_info_sub_, ori_xtion_depth_sub_, ori_xtion_depth_info_sub_) );
                without_pg_sensor_sync_->registerCallback( boost::bind( &DataPublisher::without_pg_callback, this, _1, _2, _3, _4 ) );

            }
            else if ( use_pg_ == true ) {
                ROS_INFO( "Do use point grey high resolution camera" );
                nh_->param<std::string>("ori_pg_rgb_image", ori_pg_rgb_topic_, "/rgb_image");
                nh_->param<std::string>("ori_pg_rgb_info", ori_pg_rgb_info_topic_, "/camera_info");
                ROS_INFO( "Subscribe from %s", ori_pg_rgb_topic_.c_str() );
                ROS_INFO( "Subscribe from %s", ori_pg_rgb_info_topic_.c_str() );

                // subscribe to specific topics
                ori_xtion_rgb_sub_.subscribe( *nh_, ori_xtion_rgb_topic_, 1);
                ori_xtion_rgb_info_sub_.subscribe( *nh_, ori_xtion_rgb_info_topic_, 1);
                ori_xtion_depth_sub_.subscribe( *nh_, ori_xtion_depth_topic_, 1);
                ori_xtion_depth_info_sub_.subscribe( *nh_, ori_xtion_depth_info_topic_, 1);
                ori_pg_rgb_sub_.subscribe( *nh_, ori_pg_rgb_topic_, 1);
                ori_pg_rgb_info_sub_.subscribe( *nh_, ori_pg_rgb_info_topic_, 1);
                with_pg_sensor_sync_.reset( new message_filters::Synchronizer<with_pg_policy>(with_pg_policy(10), ori_xtion_rgb_sub_, ori_xtion_rgb_info_sub_, ori_xtion_depth_sub_, ori_xtion_depth_info_sub_, ori_pg_rgb_sub_, ori_pg_rgb_info_sub_) );
                with_pg_sensor_sync_->registerCallback( boost::bind( &DataPublisher::with_pg_callback, this, _1, _2, _3, _4, _5, _6 ) );
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

                recog_completed_ = true;
                recog_server_ = nh.advertiseService( recog_srv_name_, &DataPublisher::recog_srv_callback, this);
                obj_sub_ = nh.subscribe( obj_topic_name_, 1, &DataPublisher::recog_callback, this );

                client_ = nh.serviceClient<apc_msgs::TargetRequest>(target_srv_name_);

                nh_->param<std::string>("xtion_rgb_image", xtion_rgb_topic_, "/camera/lowres_rgb/image");
                nh_->param<std::string>("xtion_depth_image", xtion_depth_topic_, "/camera/depth/image");
                nh_->param<std::string>("xtion_rgb_info", xtion_rgb_info_topic_, "/camera/lowres_rgb/camera_info");
                nh_->param<std::string>("xtion_depth_info", xtion_depth_info_topic_, "/camera/depth/camera_info");
                nh_->param<std::string>("pg_rgb_image", camera_rgb_topic_, "/camera/highres_rgb/image");
                nh_->param<std::string>("pg_rgb_info", camera_rgb_info_topic_, "/camera/highres_rgb/camera_info");


                xtion_rgb_pub_ = nh.advertise<sensor_msgs::Image>(xtion_rgb_topic_, 1);
                xtion_rgb_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(xtion_rgb_info_topic_, 1);
                xtion_depth_pub_ = nh.advertise<sensor_msgs::Image>(xtion_depth_topic_, 1);
                xtion_depth_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(xtion_depth_info_topic_, 1);
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

                recog_completed_ = true;
                recog_server_ = nh.advertiseService( recog_srv_name_, &DataPublisher::recog_srv_callback, this);
                obj_sub_ = nh.subscribe( obj_topic_name_, 1, &DataPublisher::recog_callback, this );

                client_ = nh.serviceClient<apc_msgs::TargetRequest>(target_srv_name_);

                nh_->param<std::string>("xtion_rgb_image", xtion_rgb_topic_, "/camera/lowres_rgb/image");
                nh_->param<std::string>("xtion_depth_image", xtion_depth_topic_, "/camera/depth/image");
                nh_->param<std::string>("xtion_rgb_info", xtion_rgb_info_topic_, "/camera/lowres_rgb/camera_info");
                nh_->param<std::string>("xtion_depth_info", xtion_depth_info_topic_, "/camera/depth/camera_info");
                nh_->param<std::string>("pg_rgb_image", camera_rgb_topic_, "/camera/highres_rgb/image");
                nh_->param<std::string>("pg_rgb_info", camera_rgb_info_topic_, "/camera/highres_rgb/camera_info");


                xtion_rgb_pub_ = nh.advertise<sensor_msgs::Image>(xtion_rgb_topic_, 1);
                xtion_rgb_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(xtion_rgb_info_topic_, 1);
                xtion_depth_pub_ = nh.advertise<sensor_msgs::Image>(xtion_depth_topic_, 1);
                xtion_depth_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(xtion_depth_info_topic_, 1);
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
        ros::MultiThreadedSpinner spinner(4);
        ros::Rate loop(10);
        while ( ros::ok() ) {
            spinner.spin();
            loop.sleep();
        }
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
        /*
        {
            boost::mutex::scoped_lock lock( recog_completed_mutex_ );
            recog_completed_ = true;
        }
        recog_completed_cond_.notify_one();
        */
        resp.pub = true;
        return true;
    }




    void publisher() {
        while ( nh_->ok() ) {
            if ( online_mode_ == true ) {
                for ( count_ = 0; count_ < (int)bin_contents_.size(); ++  count_ ) {
                    {
                        boost::mutex::scoped_lock lock(sensor_mutex_);
                        while(sensor_empty_) {
                            sensor_cond_.wait( lock );
                        }
                    }

                    string bin_id = work_order_[count_].first;

                    // generate msgs
                    sensor_data_.xtion_rgb->toImageMsg( this->xtion_rgb_msg_ );
                    sensor_data_.xtion_depth->toImageMsg( this->xtion_depth_msg_ );
                    cam_model_to_msg( this->xtion_rgb_info_msg_, sensor_data_.xtion_rgb_model );
                    this->xtion_rgb_info_msg_.width = this->xtion_rgb_msg_.width;
                    this->xtion_rgb_info_msg_.height = this->xtion_rgb_msg_.height;
                    cam_model_to_msg( this->xtion_depth_info_msg_, sensor_data_.xtion_rgb_model );
                    this->xtion_depth_info_msg_.width = this->xtion_depth_msg_.width;
                    this->xtion_depth_info_msg_.height = this->xtion_depth_msg_.height;

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

                        // wait for recogniser done before sending request
                        /*
                        {
                            boost::mutex::scoped_lock lock(recog_completed_mutex_);
                            while ( !recog_completed_ ) {
                                recog_completed_cond_.wait(lock);
                            }
                        }
                        */

                        target_req_srv.request.use_cloud = use_pointcloud_;
                        target_req_srv.request.use_pg = this->use_pg_;
                        ROS_INFO( "request object name %s, sending ...",  target_req_srv.request.ObjectName.c_str() );
                       if ( client_.call( target_req_srv ) )
                            ROS_INFO( "return status: %s", target_req_srv.response.Found? "true" : "false" );
                        else
                            ROS_ERROR( "Target object: %s, failed to call service target_object", target_req_srv.request.ObjectName.c_str() );

                        sleep(1);

                        xtion_rgb_pub_.publish( this->xtion_rgb_msg_ );
                        xtion_rgb_info_pub_.publish( this->xtion_rgb_info_msg_ );
                        xtion_depth_pub_.publish( this->xtion_depth_msg_ );
                        xtion_depth_info_pub_.publish( this->xtion_depth_info_msg_ );
                        if ( this->use_pg_ = true ) {
                            sensor_data_.pg_rgb->toImageMsg( this->pg_rgb_msg_ );
                            camera_rgb_pub_.publish( this->pg_rgb_msg_ );
                            cam_model_to_msg( this->pg_rgb_info_msg_, sensor_data_.pg_rgb_model );
                            this->pg_rgb_info_msg_.width = this->pg_rgb_msg_.width;
                            this->pg_rgb_info_msg_.height = this->pg_rgb_msg_.height;
                            camera_rgb_info_pub_.publish( this->pg_rgb_info_msg_ );
                        }

                        // after publish set recogniser flag to false
                        recog_completed_mutex_.lock();
                        recog_completed_ = false;
                        recog_completed_mutex_.unlock();
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

                        // generate xtion rgb msg
                        target_req_srv.request.xtion_rgb_image = this->xtion_rgb_msg_;
                        target_req_srv.request.xtion_depth_image = this->xtion_depth_msg_;
                        target_req_srv.request.xtion_rgb_info = this->xtion_rgb_info_msg_;
                        target_req_srv.request.xtion_depth_info = this->xtion_depth_info_msg_;
                        target_req_srv.request.use_cloud = false;

                        if ( this->use_pg_ == true ) {
                            target_req_srv.request.use_cloud = true;
                            sensor_data_.pg_rgb->toImageMsg( pg_rgb_msg_ );
                            target_req_srv.request.pg_rgb_image = pg_rgb_msg_;
                            cam_model_to_msg( target_req_srv.request.pg_rgb_info, sensor_data_.pg_rgb_model );
                            target_req_srv.request.pg_rgb_info.width = target_req_srv.request.pg_rgb_image.width;
                            target_req_srv.request.pg_rgb_info.height = target_req_srv.request.pg_rgb_image.height;
                        }

                        ROS_INFO( "request object name %s, sending ...",  target_req_srv.request.ObjectName.c_str() );
                       if ( client_.call( target_req_srv ) ) {
                            ROS_INFO( "return status: %s", target_req_srv.response.found? "true" : "false" );
                        }
                        else {
                            ROS_ERROR( "Target object: %s, failed to call service target_object", target_req_srv.request.ObjectName.c_str() );
                        }

                        {
                            boost::mutex::scoped_lock lock( recog_completed_mutex_ );
                            recog_completed_ = true;
                        }
                        recog_completed_cond_.notify_one();

                        sensor_mutex_.lock();
                        sensor_empty_ = true;
                        sensor_mutex_.unlock();
                    }
                }
            }
            else if ( online_mode_ == false ) {
                for ( count_ = 0; count_ < (int)bin_contents_.size(); ++ count_ ) {
                    // generate bin id using ascii table
                    string bin_id = work_order_[count_].first;

                    // generate sensor information msgs
                    // generate filenames
                    std::string xtion_rgb_name = dir_ + "/xtion_rgb_" + boost::lexical_cast<std::string>(count_+1) + ".png";
                    std::string xtion_depth_name = dir_ + "/xtion_depth_" + boost::lexical_cast<std::string>(count_+1) + ".png";  // for depth images, .png
                    std::string xtion_rgb_info_name = dir_ + "/xtion_rgb_info_" + boost::lexical_cast<std::string>(count_+1) + ".yml";
                    std::string xtion_depth_info_name = dir_ + "/xtion_depth_info_" + boost::lexical_cast<std::string>(count_+1) + ".yml";

                    cv_bridge::CvImage xtion_rgb_cv;
                    cv_bridge::CvImage xtion_depth_cv;

                    // generate xtion rgb msg
                    xtion_rgb_cv.image = cv::imread( xtion_rgb_name, CV_LOAD_IMAGE_COLOR );
                    xtion_rgb_cv.encoding = sensor_msgs::image_encodings::BGR8;
                    xtion_rgb_cv.toImageMsg( xtion_rgb_msg_ );

                    // generate xtion depth msg
                    xtion_depth_cv.image  = cv::imread( xtion_depth_name, CV_LOAD_IMAGE_ANYDEPTH );
                    xtion_depth_cv.encoding = sensor_msgs::image_encodings::MONO16;
                    xtion_depth_cv.toImageMsg( xtion_depth_msg_ );

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
                    xtion_rgb_info_msg_.height = xtion_rgb_cv.image.rows;
                    xtion_rgb_info_msg_.width  = xtion_rgb_cv.image.cols;
                    xtion_rgb_info_msg_.P = xtion_rgb_boost_p;
                    xtion_rgb_info_msg_.D = xtion_rgb_boost_d;

                    // generate xtion depth camera info
                    cv::FileStorage xtion_depth_fs(xtion_depth_info_name, cv::FileStorage::READ);
                    cv::Mat xtion_depth_proj, xtion_depth_dist;
                    xtion_depth_fs["Projection"] >> xtion_depth_proj;
                    xtion_depth_fs["Distortion"] >> xtion_depth_dist;
                    boost::array<double, 12> xtion_depth_boost_p;
                    for ( int y = 0; y < xtion_depth_proj.rows; ++ y ) {
                        for ( int x = 0; x < xtion_depth_proj.cols; ++ x ) {
                            xtion_depth_boost_p[idx] = y*xtion_depth_proj.cols+x;
                            idx ++;
                        }
                    }
                    idx = 0;
                    std::vector<double> xtion_depth_boost_d;
                    for ( int y = 0; y < xtion_depth_dist.rows; ++ y )
                        for ( int x = 0; x < xtion_depth_dist.cols; ++ x )
                            xtion_depth_boost_d.push_back( xtion_depth_dist.at<double>(y,x) );
                    idx = 0;
                    xtion_depth_info_msg_.height = xtion_depth_cv.image.rows;
                    xtion_depth_info_msg_.width  = xtion_depth_cv.image.cols;
                    xtion_depth_info_msg_.P = xtion_depth_boost_p;
                    xtion_depth_info_msg_.D = xtion_depth_boost_d;



                    // optional point cloud
                    if( use_pointcloud_ == true ) {
                        std::string xtion_cloud_name = dir_ + "/xtion_cloud_" + boost::lexical_cast<std::string>(count_+1) + ".pcd";
                        // publish point cloud message
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
                        pcl::io::loadPCDFile( xtion_cloud_name, *cloud );
                        pcl::toROSMsg( *cloud, xtion_cloud_msg_ );
                    }

                    // optional pg image
                    if ( use_pg_ == true ) {
                        std::string camera_rgb_name = dir_ + "/camera_rgb_" + boost::lexical_cast<std::string>(count_+1) + ".png";
                        std::string camera_rgb_info_name = dir_ + "/camera_rgb_info_" + boost::lexical_cast<std::string>(count_+1) + ".yml";
                        cv_bridge::CvImage camera_rgb_cv;

                        // generate pg rgb msg
                        camera_rgb_cv.image = cv::imread( camera_rgb_name, CV_LOAD_IMAGE_COLOR );
                        camera_rgb_cv.encoding = sensor_msgs::image_encodings::BGR8;
                        camera_rgb_cv.toImageMsg( pg_rgb_msg_ );

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
                        pg_rgb_info_msg_.height = camera_rgb_cv.image.rows;
                        pg_rgb_info_msg_.width  = camera_rgb_cv.image.cols;
                        pg_rgb_info_msg_.P = camera_rgb_boost_p;
                        pg_rgb_info_msg_.D = camera_rgb_boost_d;
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

                        // wait for recogniser done before sending request
                        //
                        //{
                        //    boost::mutex::scoped_lock lock(recog_completed_mutex_);
                        //    while ( !recog_completed_ ) {
                        //        recog_completed_cond_.wait(lock);
                        //    }
                        //}


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

                        xtion_rgb_pub_.publish( xtion_rgb_msg_ );
                        xtion_rgb_info_pub_.publish( xtion_rgb_info_msg_ );
                        xtion_depth_pub_.publish( xtion_depth_msg_ );
                        xtion_depth_info_pub_.publish( xtion_depth_info_msg_ );
                        if ( use_pg_ == true ) {
                            camera_rgb_pub_.publish( pg_rgb_msg_ );
                            camera_rgb_info_pub_.publish( pg_rgb_info_msg_ );
                        }
                        if( use_pointcloud_ == true ) {
                            xtion_cloud_pub_.publish( xtion_cloud_msg_ );
                        }

                        // after publish set recogniser flag to false
                        recog_completed_mutex_.lock();
                        recog_completed_ = false;
                        recog_completed_mutex_.unlock();
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

                        target_req_srv.request.xtion_rgb_image = xtion_rgb_msg_;
                        target_req_srv.request.xtion_depth_image = xtion_depth_msg_;
                        target_req_srv.request.xtion_rgb_info = xtion_rgb_info_msg_;
                        target_req_srv.request.xtion_depth_info = xtion_depth_info_msg_;
                        target_req_srv.request.use_pg = use_pg_;
                        if ( use_pg_ == true ) {
                            target_req_srv.request.pg_rgb_image = pg_rgb_msg_;
                            target_req_srv.request.pg_rgb_info = pg_rgb_info_msg_;
                        }

                        if ( use_pointcloud_ == true ) {
                            target_req_srv.request.use_cloud = true;
                            target_req_srv.request.cloud = xtion_cloud_msg_;
                        }

                        ROS_INFO( "request object name %s, sending ...", target_req_srv.request.ObjectName.c_str() );
                       if ( client_.call( target_req_srv ) ) {
                            ROS_INFO( "return status: %s", target_req_srv.response.found? "true" : "false" );
                        }
                        else {
                            ROS_ERROR( "Target object: %s, failed to call service target_object", target_req_srv.request.ObjectName.c_str() );
                        }
                    }
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
    return 1;
}


