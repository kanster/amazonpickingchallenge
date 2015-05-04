/** init uts recogniser
  * write empty bin sensor information and
  * mask image for each bin
  */

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
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

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


/** Multiple point mouse callback
  * used to generate mask image
  */
class MultiMouseCallback{
public:
    struct PointsImage {
        std::vector< cv::Point > pts;
        cv::Mat image;
    };

    PointsImage points_image;
    std::string window_name;

public:
    MultiMouseCallback() {}

    MultiMouseCallback( cv::Mat image ) {
        window_name = "MouseCallbackMulti::callback()";
        points_image.image = image;
    }

    void release() {
        points_image.image.release();
        cv::destroyWindow( window_name );
        points_image.pts.clear();
    }

    static void on_mouse( int event, int x, int y, int flag, void * ptr ) {
        cv::Mat * image = &(( PointsImage * )ptr)->image;
        MultiMouseCallback mouse_cb( *image );
        if ( event == CV_EVENT_LBUTTONDOWN ) {
            cv::Point pt;
            pt.x = x;   pt.y = y;
            cv::circle( mouse_cb.points_image.image, pt, 5, cv::Scalar(255, 255, 255), 5 );
            (( PointsImage * )ptr)->pts.push_back( pt );
        }
    }

    void callback() {
        cv::namedWindow( window_name, CV_WINDOW_NORMAL );
        cv::setMouseCallback( window_name, on_mouse, &points_image );
        char key = 0;
        while ( key != 27 ) {
            cv::imshow( window_name, points_image.image );
            key = cv::waitKey( 10 );
        }
    }
};


class UTSInit{
public:
    //! node handle
    ros::NodeHandle *nh_;

    //! publish topic names and publishers
    std::string xtion_rgb_topic_;
    std::string xtion_depth_topic_;
    std::string xtion_cloud_topic_;
    std::string pg_rgb_topic_;


    //! subscriber
    message_filters::Subscriber<sensor_msgs::Image>         xtion_rgb_sub_;
    message_filters::Subscriber<sensor_msgs::Image>         xtion_depth_sub_;
    ros::Subscriber xtion_cloud_sub_;
    message_filters::Subscriber<sensor_msgs::Image>         pg_rgb_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>   sync_xtion_cloud_sub_;


    //! config
    bool use_pg_;
    std::string save_dir_;
    int n_bin_;

    //! capture index
    int icap_;

    //! sync policy
    //! callback function for online mode
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> with_pg_policy;
    boost::shared_ptr<message_filters::Synchronizer<with_pg_policy> > with_pg_sensor_sync_;

    cv::Mat xtion_rgb_;
    cv::Mat xtion_depth_;
    cv::Mat pg_rgb_;

    cv::Mat disp_rgb_;
    cv::Mat disp_depth_;

    //! cloud msg
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    cv::Mat cloud_rgb_;
    cv::Mat cloud_depth_;


private:
    void without_pg_callback( const sensor_msgs::PointCloud2ConstPtr & cloud_msg ) {
        cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() );
        pcl::fromROSMsg( *cloud_msg, *cloud_ );

        // init rgb and depth images from cloud
        cloud_rgb_ = cv::Mat( cloud_->height, cloud_->width, CV_8UC3 );
        cloud_depth_ = cv::Mat( cloud_->height, cloud_->width, CV_16UC1, cv::Scalar(0) );
        disp_depth_ = cv::Mat( cloud_->height, cloud_->width, CV_32FC1, cv::Scalar(0) );

        for ( int y = 0; y < (int)cloud_->height; ++ y ) {
            for ( int x = 0; x < (int)cloud_->width; ++ x ) {
                pcl::PointXYZRGB & pt = cloud_->points[y*cloud_->width+x];
                cloud_rgb_.at<cv::Vec3b>(y,x)[0] = pt.b;
                cloud_rgb_.at<cv::Vec3b>(y,x)[1] = pt.g;
                cloud_rgb_.at<cv::Vec3b>(y,x)[2] = pt.r;
                if ( !pcl_isinf(pt.x) && !pcl_isnan(pt.x) &&
                     !pcl_isinf(pt.y) && !pcl_isnan(pt.y) &&
                     !pcl_isinf(pt.z) && !pcl_isnan(pt.z) ) {
                    cloud_depth_.at<unsigned short>(y,x) = static_cast<unsigned short>(pt.z*1000.0);
                    disp_depth_.at<float>(y,x) = static_cast<float>(pt.z);
                }
            }
        }

        // disp images
        disp_rgb_ = cloud_rgb_.clone();
        cv::putText( disp_rgb_, "Press <s> to save image", cv::Point(20, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1 );

        cv::imshow( "xtion_rgb_image", disp_rgb_ );
        cv::imshow( "xtion_depth_image", disp_depth_ );
        char k = cv::waitKey(5);
        if ( k == 's' ) {
            if ( icap_ <= n_bin_ ) {
                char id = static_cast<char>(icap_+64);
                std::string bin_id = "bin_"+std::string(1, id);
                ROS_INFO( "Save empty image and mask image for %s", bin_id.c_str() );
                std::string empty_rgb_path   = save_dir_+"/xtion_rgb_empty_"+bin_id+".png";
                std::string empty_depth_path = save_dir_+"/xtion_depth_empty_"+bin_id+".png";
                cv::imwrite( empty_rgb_path, cloud_rgb_ );
                cv::imwrite( empty_depth_path, cloud_depth_ );

                // draw mask image
                cv::Mat cloud_rgb_mask( cloud_rgb_.rows, cloud_rgb_.cols, CV_8UC1, cv::Scalar::all(0) );
                MultiMouseCallback cloud_rgb_cb( cloud_rgb_.clone() );
                cloud_rgb_cb.callback();
                cv::Point cloud_rgb_poly_pts[1][cloud_rgb_cb.points_image.pts.size()];
                for ( int i = 0; i < (int)cloud_rgb_cb.points_image.pts.size(); i ++ ) {
                    cloud_rgb_poly_pts[0][i].x = cloud_rgb_cb.points_image.pts[i].x;
                    cloud_rgb_poly_pts[0][i].y = cloud_rgb_cb.points_image.pts[i].y;
                }
                const cv::Point * cloud_rgb_st_pt[1] = { cloud_rgb_poly_pts[0] };
                int n_cloud_rgb_poly_pts[] = { cloud_rgb_cb.points_image.pts.size() };
                cv::fillPoly( cloud_rgb_mask, cloud_rgb_st_pt, n_cloud_rgb_poly_pts, 1, cv::Scalar::all(255), 8 );
                cloud_rgb_cb.release();

                cv::Mat cloud_depth_mask( disp_depth_.rows, disp_depth_.cols, CV_8UC1, cv::Scalar::all(0) );
                MultiMouseCallback cloud_depth_cb( disp_depth_.clone() );
                cloud_depth_cb.callback();
                cv::Point cloud_depth_poly_pts[1][cloud_depth_cb.points_image.pts.size()];
                for ( int i = 0; i < (int)cloud_depth_cb.points_image.pts.size(); i ++ ) {
                    cloud_depth_poly_pts[0][i].x = cloud_depth_cb.points_image.pts[i].x;
                    cloud_depth_poly_pts[0][i].y = cloud_depth_cb.points_image.pts[i].y;
                }
                const cv::Point * cloud_depth_st_pt[1] = { cloud_depth_poly_pts[0] };
                int n_cloud_depth_poly_pts[] = { cloud_depth_cb.points_image.pts.size() };
                cv::fillPoly( cloud_depth_mask, cloud_depth_st_pt, n_cloud_depth_poly_pts, 1, cv::Scalar::all(255), 8 );
                cloud_depth_cb.release();

                std::string mask_rgb_path   = save_dir_+"/xtion_rgb_mask_"+bin_id+".png";
                std::string mask_depth_path = save_dir_+"/xtion_depth_mask_"+bin_id+".png";
                cv::imwrite( mask_rgb_path, cloud_rgb_mask );
                cv::imwrite( mask_depth_path, cloud_depth_mask );

                ROS_INFO( "Finish save %s", bin_id.c_str() );
                icap_ ++;
            }
            else {
                ROS_INFO( "Enough information collected" );
            }
        }
    }

    /*
    void without_pg_callback( const sensor_msgs::ImageConstPtr & xtion_rgb_msg,
                              const sensor_msgs::ImageConstPtr & xtion_depth_msg ){
        xtion_rgb_   = cv_bridge::toCvCopy( xtion_rgb_msg, sensor_msgs::image_encodings::BGR8 )->image;
        xtion_depth_ = cv_bridge::toCvCopy( xtion_depth_msg, sensor_msgs::image_encodings::TYPE_16UC1 )->image;
        disp_rgb_ = xtion_rgb_.clone();
        cv::putText( disp_rgb_, "Press <s> to save image", cv::Point(20, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1 );


        double min, max;
        cv::minMaxIdx( xtion_depth_, &min, &max );
        cv::convertScaleAbs( xtion_depth_, disp_depth_, 255./max );
        cv::imshow( "xtion_rgb_image", disp_rgb_ );
        cv::imshow( "xtion_depth_image", disp_depth_ );
        char k = cv::waitKey(5);
        if ( k == 's' ) {
            if ( icap_ <= n_bin_ ) {
                char id = static_cast<char>(icap_+64);
                std::string bin_id = "bin_"+std::string(1, id);
                ROS_INFO( "Save empty image and mask image for %s", bin_id.c_str() );
                std::string empty_rgb_path   = save_dir_+"/xtion_rgb_empty_"+bin_id+".png";
                std::string empty_depth_path = save_dir_+"/xtion_depth_empty_"+bin_id+".png";
                cv::imwrite( empty_rgb_path, xtion_rgb_ );
                cv::imwrite( empty_depth_path, xtion_depth_ );

                // draw mask image
                cv::Mat xtion_rgb_mask( xtion_rgb_.rows, xtion_rgb_.cols, CV_8UC1, cv::Scalar::all(0) );
                MultiMouseCallback xtion_rgb_cb( xtion_rgb_.clone() );
                xtion_rgb_cb.callback();
                cv::Point xtion_rgb_poly_pts[1][xtion_rgb_cb.points_image.pts.size()];
                for ( int i = 0; i < (int)xtion_rgb_cb.points_image.pts.size(); i ++ ) {
                    xtion_rgb_poly_pts[0][i].x = xtion_rgb_cb.points_image.pts[i].x;
                    xtion_rgb_poly_pts[0][i].y = xtion_rgb_cb.points_image.pts[i].y;
                }
                const cv::Point * xtion_rgb_st_pt[1] = { xtion_rgb_poly_pts[0] };
                int n_xtion_rgb_poly_pts[] = { xtion_rgb_cb.points_image.pts.size() };
                cv::fillPoly( xtion_rgb_mask, xtion_rgb_st_pt, n_xtion_rgb_poly_pts, 1, cv::Scalar::all(255), 8 );
                xtion_rgb_cb.release();

                cv::Mat xtion_depth_mask( disp_depth_.rows, disp_depth_.cols, CV_8UC1, cv::Scalar::all(0) );
                MultiMouseCallback xtion_depth_cb( disp_depth_.clone() );
                xtion_depth_cb.callback();
                cv::Point xtion_depth_poly_pts[1][xtion_depth_cb.points_image.pts.size()];
                for ( int i = 0; i < (int)xtion_depth_cb.points_image.pts.size(); i ++ ) {
                    xtion_depth_poly_pts[0][i].x = xtion_depth_cb.points_image.pts[i].x;
                    xtion_depth_poly_pts[0][i].y = xtion_depth_cb.points_image.pts[i].y;
                }
                const cv::Point * xtion_depth_st_pt[1] = { xtion_depth_poly_pts[0] };
                int n_xtion_depth_poly_pts[] = { xtion_depth_cb.points_image.pts.size() };
                cv::fillPoly( xtion_depth_mask, xtion_depth_st_pt, n_xtion_depth_poly_pts, 1, cv::Scalar::all(255), 8 );
                xtion_depth_cb.release();

                std::string mask_rgb_path   = save_dir_+"/xtion_rgb_mask_"+bin_id+".png";
                std::string mask_depth_path = save_dir_+"/xtion_depth_mask_"+bin_id+".png";
                cv::imwrite( mask_rgb_path, xtion_rgb_mask );
                cv::imwrite( mask_depth_path, xtion_depth_mask );

                ROS_INFO( "Finish save %s", bin_id.c_str() );
                icap_ ++;
            }
            else {
                ROS_INFO( "Enough information collected" );
            }
        }
    }
    */

    void with_pg_callback( const sensor_msgs::PointCloud2ConstPtr & xtion_cloud_msg,
                           const sensor_msgs::ImageConstPtr & pg_rgb_msg ){
        cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() );
        pcl::fromROSMsg( *xtion_cloud_msg, *cloud_ );

        // init rgb and depth images from cloud
        cloud_rgb_ = cv::Mat( cloud_->height, cloud_->width, CV_8UC3 );
        cloud_depth_ = cv::Mat( cloud_->height, cloud_->width, CV_16UC1, cv::Scalar(0) );
        disp_depth_ = cv::Mat( cloud_->height, cloud_->width, CV_32FC1, cv::Scalar(0) );

        for ( int y = 0; y < (int)cloud_->height; ++ y ) {
            for ( int x = 0; x < (int)cloud_->width; ++ x ) {
                pcl::PointXYZRGB & pt = cloud_->points[y*cloud_->width+x];
                cloud_rgb_.at<cv::Vec3b>(y,x)[0] = pt.b;
                cloud_rgb_.at<cv::Vec3b>(y,x)[1] = pt.g;
                cloud_rgb_.at<cv::Vec3b>(y,x)[2] = pt.r;
                if ( !pcl_isinf(pt.x) && !pcl_isnan(pt.x) &&
                     !pcl_isinf(pt.y) && !pcl_isnan(pt.y) &&
                     !pcl_isinf(pt.z) && !pcl_isnan(pt.z) ) {
                    cloud_depth_.at<unsigned short>(y,x) = static_cast<unsigned short>(pt.z*1000.0);
                    disp_depth_.at<float>(y,x) = static_cast<float>(pt.z);
                }
            }
        }

        // disp images
        disp_rgb_ = cloud_rgb_.clone();
        cv::putText( disp_rgb_, "Press <s> to save image", cv::Point(20, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1 );

        pg_rgb_      = cv_bridge::toCvCopy( pg_rgb_msg, sensor_msgs::image_encodings::BGR8 )->image;

        cv::imshow( "xtion_rgb_image", disp_rgb_ );
        cv::imshow( "xtion_depth_image", disp_depth_ );
        cv::imshow( "pg_rgb_image", pg_rgb_ );
        char k = cv::waitKey(5);
        if ( k == 's' ) {
            if ( icap_ <= n_bin_ ) {
                char id = static_cast<char>(icap_+64);
                std::string bin_id = "bin_"+std::string(1, id);
                ROS_INFO( "Save empty image and mask image for %s", bin_id.c_str() );
                std::string empty_rgb_path   = save_dir_+"/xtion_rgb_empty_"+bin_id+".png";
                std::string empty_depth_path = save_dir_+"/xtion_depth_empty_"+bin_id+".png";
                cv::imwrite( empty_rgb_path, cloud_rgb_ );
                cv::imwrite( empty_depth_path, cloud_depth_ );

                // draw mask image
                cv::Mat cloud_rgb_mask( cloud_rgb_.rows, cloud_rgb_.cols, CV_8UC1, cv::Scalar::all(0) );
                MultiMouseCallback cloud_rgb_cb( cloud_rgb_.clone() );
                cloud_rgb_cb.callback();
                cv::Point cloud_rgb_poly_pts[1][cloud_rgb_cb.points_image.pts.size()];
                for ( int i = 0; i < (int)cloud_rgb_cb.points_image.pts.size(); i ++ ) {
                    cloud_rgb_poly_pts[0][i].x = cloud_rgb_cb.points_image.pts[i].x;
                    cloud_rgb_poly_pts[0][i].y = cloud_rgb_cb.points_image.pts[i].y;
                }
                const cv::Point * cloud_rgb_st_pt[1] = { cloud_rgb_poly_pts[0] };
                int n_cloud_rgb_poly_pts[] = { cloud_rgb_cb.points_image.pts.size() };
                cv::fillPoly( cloud_rgb_mask, cloud_rgb_st_pt, n_cloud_rgb_poly_pts, 1, cv::Scalar::all(255), 8 );
                cloud_rgb_cb.release();

                cv::Mat cloud_depth_mask( disp_depth_.rows, disp_depth_.cols, CV_8UC1, cv::Scalar::all(0) );
                MultiMouseCallback cloud_depth_cb( disp_depth_.clone() );
                cloud_depth_cb.callback();
                cv::Point cloud_depth_poly_pts[1][cloud_depth_cb.points_image.pts.size()];
                for ( int i = 0; i < (int)cloud_depth_cb.points_image.pts.size(); i ++ ) {
                    cloud_depth_poly_pts[0][i].x = cloud_depth_cb.points_image.pts[i].x;
                    cloud_depth_poly_pts[0][i].y = cloud_depth_cb.points_image.pts[i].y;
                }
                const cv::Point * cloud_depth_st_pt[1] = { cloud_depth_poly_pts[0] };
                int n_cloud_depth_poly_pts[] = { cloud_depth_cb.points_image.pts.size() };
                cv::fillPoly( cloud_depth_mask, cloud_depth_st_pt, n_cloud_depth_poly_pts, 1, cv::Scalar::all(255), 8 );
                cloud_depth_cb.release();

                std::string mask_rgb_path   = save_dir_+"/xtion_rgb_mask_"+bin_id+".png";
                std::string mask_depth_path = save_dir_+"/xtion_depth_mask_"+bin_id+".png";
                cv::imwrite( mask_rgb_path, cloud_rgb_mask );
                cv::imwrite( mask_depth_path, cloud_depth_mask );

                cv::Mat pg_rgb_mask( pg_rgb_.rows, pg_rgb_.cols, CV_8UC1, cv::Scalar::all(0) );
                MultiMouseCallback pg_rgb_cb( pg_rgb_.clone() );
                pg_rgb_cb.callback();
                cv::Point pg_rgb_poly_pts[1][pg_rgb_cb.points_image.pts.size()];
                for ( int i = 0; i < (int)pg_rgb_cb.points_image.pts.size(); i ++ ) {
                    pg_rgb_poly_pts[0][i].x = pg_rgb_cb.points_image.pts[i].x;
                    pg_rgb_poly_pts[0][i].y = pg_rgb_cb.points_image.pts[i].y;
                }
                const cv::Point * pg_rgb_st_pt[1] = { pg_rgb_poly_pts[0] };
                int n_pg_rgb_poly_pts[] = { pg_rgb_cb.points_image.pts.size() };
                cv::fillPoly( pg_rgb_mask, pg_rgb_st_pt, n_pg_rgb_poly_pts, 1, cv::Scalar::all(255), 8 );
                pg_rgb_cb.release();
                std::string pg_rgb_path = save_dir_+"/pg_rgb_mask_"+bin_id+".png";
                cv::imwrite( pg_rgb_path, pg_rgb_mask );


                cloud_rgb_mask.release();
                cloud_depth_mask.release();
                pg_rgb_mask.release();

                ROS_INFO( "Finish save %s", bin_id.c_str() );
                icap_ ++;
                ROS_INFO("Number %d captured", icap_);
            }
            else {
                ROS_INFO( "Enough information collected" );
            }
        }
    }

public:
    UTSInit( ros::NodeHandle & nh ){
        nh_ = & nh;
        nh_->param<std::string>( "save_dir", save_dir_, "/tmp/apc/" );
        nh_->param<bool>( "use_pg", use_pg_, false );
        nh_->param<int>( "n_bin", n_bin_, 1 );

        icap_ = 1;
        // create directory
        boost::filesystem::path dir( save_dir_.c_str() );
        if( boost::filesystem::create_directory(dir) )
            ROS_INFO( "Create directory %s", dir.c_str() );
        else
            ROS_INFO( "Directory %s exists", dir.c_str() );

        // subscribe topics
        cv::namedWindow( "xtion_rgb_image" );   cv::moveWindow( "xtion_rgb_image", 0, 0 );
        cv::namedWindow( "xtion_depth_image" ); cv::moveWindow( "xtion_depth_image", 0, 520 );
        nh_->param<std::string>("xtion_cloud", xtion_cloud_topic_, "/camera/depth_registered/points");
        ROS_INFO( "Subscribe from %s", xtion_cloud_topic_.c_str() );

        if ( use_pg_ == false ) {
            ROS_INFO( "Do NOT use point grey high resolution camera" );
            // subscribe to specific topics
            xtion_cloud_sub_ = nh_->subscribe( xtion_cloud_topic_, 1, &UTSInit::without_pg_callback, this );
        }
        else {
            cv::namedWindow( "pg_rgb_image" );  cv::moveWindow( "pg_rgb_image", 650, 0 );
            ROS_INFO( "Do use point grey high resolution camera" );
            nh_->param<std::string>("pg_rgb_image", pg_rgb_topic_, "/rgb_image");
            ROS_INFO( "Subscribe from %s", pg_rgb_topic_.c_str() );

            pg_rgb_sub_.subscribe( *nh_, pg_rgb_topic_, 1 );
            sync_xtion_cloud_sub_.subscribe( *nh_, xtion_cloud_topic_, 1 );
            with_pg_sensor_sync_.reset( new message_filters::Synchronizer<with_pg_policy>(with_pg_policy(10), sync_xtion_cloud_sub_, pg_rgb_sub_) );
            with_pg_sensor_sync_->registerCallback( boost::bind( &UTSInit::with_pg_callback, this, _1, _2 ) );
        }
        ros::Rate loop(1);
        while( ros::ok() ) {
            ros::spinOnce();
            loop.sleep();
        }
    }

};


int main( int argc, char ** argv ) {
    ros::init( argc, argv, "uts_init" );
    ros::NodeHandle nh("~");
    UTSInit init( nh );
    return 1;
}
