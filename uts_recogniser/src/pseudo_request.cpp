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

    // published data on disk
    std::string dir_;
    int count_;
    int n_frames_;

    // srvs and topics names between ORS and request
    std::string target_srv_name_;
    std::string recog_srv_name_;
    std::string obj_topic_name_;

    // publish topic names and publishers
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

    // published sensor msgs
    sensor_msgs::Image xtion_rgb_msg_;
    sensor_msgs::CameraInfo xtion_rgb_info_msg_;
    sensor_msgs::Image xtion_depth_msg_;
    sensor_msgs::CameraInfo xtion_depth_info_msg_;
    sensor_msgs::PointCloud2 xtion_cloud_msg_;
    sensor_msgs::Image camera_rgb_msg_;
    sensor_msgs::CameraInfo camera_rgb_info_msg_;

    // ORS caller
    ros::ServiceClient client_;

    // ORS results server
    ros::ServiceServer recog_server_;

    // ORS result topic subscriber
    ros::Subscriber obj_sub_;

    // main publish thread
    boost::thread publish_thread_;

    // use point cloud or not
    bool use_pointcloud_;

    // recognition completed mutex and condition
    bool recog_completed_;
    boost::mutex recog_completed_mutex_;
    boost::condition_variable recog_completed_cond_;

    // json file
    JSON json_;
    std::string json_filename_;

    // srv mode
    int srv_mode_;

public:
    DataPublisher( ros::NodeHandle & nh ) {
        nh_ = & nh;

        nh_->param<std::string>("json", json_filename_, "/tmp/apc/amazon.json");
        nh_->param<std::string>("data_dir", dir_, "/tmp/apc/data/");
        nh_->param<int>("n_frames", n_frames_, 1 );
        nh_->param<bool>("use_cloud", use_pointcloud_, false);

        ROS_INFO_ONCE( "json = %s and data from %s and n_frames = %d", json_filename_.c_str(), dir_.c_str(), n_frames_ );
        json_.from_file( json_filename_ );


        nh_->param<int>("srv_mode", srv_mode_, 1);
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

        publish_thread_ = boost::thread(boost::bind(&DataPublisher::info_publisher, this));


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
        {
            boost::mutex::scoped_lock lock( recog_completed_mutex_ );
            recog_completed_ = true;
        }
        recog_completed_cond_.notify_one();

        resp.pub = true;
        return true;
    }


    void info_publisher() {
        map<string, vector<string> >  bin_contents   = json_.bin_contents_;
        vector<pair<string, string> > work_order     = json_.work_order_;


        while ( true ) {
            while ( nh_->ok() ) {
                for ( count_ = 1; count_ <= n_frames_; ++ count_ ) {
                    // generate bin id using ascii table
                    char id = static_cast<char>(count_+64);
                    string bin_id = "bin_"+string(1, id);

                    // generate sensor information msgs
                    // generate filenames
                    std::string xtion_rgb_name = dir_ + "/xtion_rgb_" + boost::lexical_cast<std::string>(count_) + ".png";
                    std::string xtion_depth_name = dir_ + "/xtion_depth_" + boost::lexical_cast<std::string>(count_) + ".png";  // for depth images, .png
                    std::string xtion_rgb_info_name = dir_ + "/xtion_rgb_info_" + boost::lexical_cast<std::string>(count_) + ".yml";
                    std::string xtion_depth_info_name = dir_ + "/xtion_depth_info_" + boost::lexical_cast<std::string>(count_) + ".yml";
                    std::string camera_rgb_name = dir_ + "/camera_rgb_" + boost::lexical_cast<std::string>(count_) + ".png";
                    std::string camera_rgb_info_name = dir_ + "/camera_rgb_info_" + boost::lexical_cast<std::string>(count_) + ".yml";

                    cv_bridge::CvImage xtion_rgb_cv;
                    cv_bridge::CvImage xtion_depth_cv;
                    cv_bridge::CvImage camera_rgb_cv;

                    // generate xtion rgb msg
                    xtion_rgb_cv.image = cv::imread( xtion_rgb_name, CV_LOAD_IMAGE_COLOR );
                    xtion_rgb_cv.encoding = sensor_msgs::image_encodings::BGR8;
                    xtion_rgb_cv.toImageMsg( xtion_rgb_msg_ );

                    // generate xtion depth msg
                    xtion_depth_cv.image  = cv::imread( xtion_depth_name, CV_LOAD_IMAGE_ANYDEPTH );
                    xtion_depth_cv.encoding = sensor_msgs::image_encodings::MONO16;
                    xtion_depth_cv.toImageMsg( xtion_depth_msg_ );

                    // generate pg rgb msg
                    camera_rgb_cv.image = cv::imread( camera_rgb_name, CV_LOAD_IMAGE_COLOR );
                    camera_rgb_cv.encoding = sensor_msgs::image_encodings::BGR8;
                    camera_rgb_cv.toImageMsg( camera_rgb_msg_ );

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
                    camera_rgb_info_msg_.height = camera_rgb_cv.image.rows;
                    camera_rgb_info_msg_.width  = camera_rgb_cv.image.cols;
                    camera_rgb_info_msg_.P = camera_rgb_boost_p;
                    camera_rgb_info_msg_.D = camera_rgb_boost_d;

                    // optional point cloud
                    if( use_pointcloud_ == true ) {
                        std::string xtion_cloud_name = dir_ + "/xtion_cloud_" + boost::lexical_cast<std::string>(count_) + ".pcd";
                        // publish point cloud message
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
                        pcl::io::loadPCDFile( xtion_cloud_name, *cloud );
                        pcl::toROSMsg( *cloud, xtion_cloud_msg_ );
                    }


                    if ( srv_mode_ == 1 ) {
                        apc_msgs::TargetRequest target_req_srv;

                        // generate target request
                        target_req_srv.request.BinID = bin_id;
                        target_req_srv.request.ObjectIndex = count_;
                        target_req_srv.request.ObjectName  = work_order[count_-1].second;
                        vector<string> bin_content = bin_contents[work_order[count_-1].first];
                        for ( int i = 0; i < (int)bin_content.size(); ++ i ) {
                            target_req_srv.request.BinContents.push_back( bin_content[i] );
                        }
                        vector<int8_t> removed_indices;
                        target_req_srv.request.RemovedObjectIndices = removed_indices;

                        // wait for recogniser done before sending request
                        {
                            boost::mutex::scoped_lock lock(recog_completed_mutex_);
                            while ( !recog_completed_ ) {
                                recog_completed_cond_.wait(lock);
                            }
                        }

                        target_req_srv.request.use_cloud = use_pointcloud_;

                        ROS_INFO( "request object %d with name %s, sending ...", target_req_srv.request.ObjectIndex, target_req_srv.request.ObjectName.c_str() );
                       if ( client_.call( target_req_srv ) ) {
                            ROS_INFO( "return status: %s", target_req_srv.response.Found? "true" : "false" );
                        }
                        else {
                            ROS_ERROR( "Target object: %s, failed to call service target_object", target_req_srv.request.ObjectName.c_str() );
                        }

                        sleep(3);

                        xtion_rgb_pub_.publish( xtion_rgb_msg_ );
                        xtion_rgb_info_pub_.publish( xtion_rgb_info_msg_ );
                        xtion_depth_pub_.publish( xtion_depth_msg_ );
                        xtion_depth_info_pub_.publish( xtion_depth_info_msg_ );
                        camera_rgb_pub_.publish( camera_rgb_msg_ );
                        camera_rgb_info_pub_.publish( camera_rgb_info_msg_ );
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
                        target_req_srv.request.ObjectIndex = count_;
                        target_req_srv.request.ObjectName  = work_order[count_-1].second;
                        vector<string> bin_content = bin_contents[work_order[count_-1].first];
                        for ( int i = 0; i < (int)bin_content.size(); ++ i ) {
                            target_req_srv.request.BinContents.push_back( bin_content[i] );
                        }
                        vector<int8_t> removed_indices;
                        target_req_srv.request.RemovedObjectIndices = removed_indices;

                        target_req_srv.request.xtion_rgb_image = xtion_rgb_msg_;
                        target_req_srv.request.xtion_depth_image = xtion_depth_msg_;
                        target_req_srv.request.pg_rgb_image = camera_rgb_msg_;

                        target_req_srv.request.xtion_rgb_info = xtion_rgb_info_msg_;
                        target_req_srv.request.xtion_depth_info = xtion_depth_info_msg_;
                        target_req_srv.request.pg_rgb_info = camera_rgb_info_msg_;

                        if ( use_pointcloud_ == true ) {
                            target_req_srv.request.use_cloud = true;
                            target_req_srv.request.cloud = xtion_cloud_msg_;
                        }

                        ROS_INFO( "request object %d with name %s, sending ...", target_req_srv.request.ObjectIndex, target_req_srv.request.ObjectName.c_str() );
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


