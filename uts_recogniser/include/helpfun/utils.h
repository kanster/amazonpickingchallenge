#ifndef UTILS_H
#define UTILS_H

//#include "siftfast/siftfast.h"
#include "include/ANN/ANN.h"

// system
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <string>
#include <algorithm>
#include <limits>

// opencv and pcl
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>

// eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZRGB PointType;

// alternative solution to boost foreach
#define foreach( i, c ) for( typeof((c).begin()) i##_hid=(c).begin(), *i##_hid2=((typeof((c).begin())*)1); i##_hid2 && i##_hid!=(c).end(); ++i##_hid) for( typeof( *(c).begin() ) &i=*i##_hid, *i##_hid3=(typeof( *(c).begin() )*)(i##_hid2=NULL); !i##_hid3 ; ++i##_hid3, ++i##_hid2)

#define eforeach( i, it, c ) for( typeof((c).begin()) it=(c).begin(), i##_hid = (c).begin(), *i##_hid2=((typeof((c).begin())*)1); i##_hid2 && it!=(c).end(); (it==i##_hid)?++it,++i##_hid:i##_hid=it) for( typeof(*(c).begin()) &i=*it, *i##_hid3=(typeof( *(c).begin() )*)(i##_hid2=NULL); !i##_hid3 ; ++i##_hid3, ++i##_hid2)

inline Vector3f string_to_vector3f( string str );
inline vector<float> string_to_vector( string str );

const string models_dir = "";


/** object model */
class Model {
private:

public:
    struct Feature{
        Vector3f coord3d;
        Vector3f color;
        vector<float> descrip;
    };
    typedef vector<Feature> Features;

    enum object_type{ RIGID_SMALL, RIGID_BIG, OTHERS };
    object_type type_;
    string name_;
    Features features_;
    Vector3f bounding_box_[2];

    // constructor
    Model();
    ~Model();
    // load models
    void load_model( string filename );

    // comparator
    bool operator < ( const Model & model ) const;
};
typedef boost::shared_ptr<Model> SP_Model;

/** estimated pose */
struct Pose {
    Quaternionf     q_;
    Translation3f   t_;
    Matrix3f        r_;

    Pose();
    Pose( Quaternionf q, Translation3f t );
    friend ostream & operator <<( ostream & out, const Pose & pose );
};

/** transformation matrix */
struct TransformMatrix {
    Matrix4f p_;
    void init( const Quaternionf & q, const Translation3f & t );
    void init( const Pose & pose );
    Vector3f transform( Vector3f orig );
    Vector3f inverse_transform( Vector3f orig );
    friend ostream & operator <<( ostream & out, const TransformMatrix & tm );
};

/** recognised object */
struct Object {
    SP_Model model_;
    Pose pose_;
    Matrix4f homo_;
    float score_;

    vector<cv::Point> get_object_hull( vector<cv::Point> pts );
    bool operator < ( const Object & o ) const;
    friend ostream & operator << ( ostream & out, const Object & obj );
};
typedef boost::shared_ptr<Object> SP_Object;
bool compare_sp_object(const SP_Object &spo_l, const SP_Object &spo_r);

/** detected features */
struct DetectedFeatureRGBD{
    Vector2f    img2d;
    Vector3f    obs3d;
    int         groupidx;   // cluster index
    vector<float> descrip;
};

struct MatchRGBD{
    Vector2f    img2d;
    Vector3f    obs3d;
    Vector3f    mdl3d;
};

/** detected features and matches for rgb data */
struct DetectedFeatureRGB{
    Vector2f    img2d;
    vector<float> descrip;
    float scale;
    float ori;

};

struct MatchRGB{
    Vector2f    img2d;
    int         imgid;
    Vector3f    mdl3d;
    int         mdlid;
    float       score;
    float       ratio;

};


//! detector parameters
struct DetectorParam{
    std::string type;

    DetectorParam() {
        type = "sift";
    }

    DetectorParam( std::string t ) {
        type = t;
    }
};


//! matcher parameters
struct MatcherParam{
    double quality;
    double ratio;
    int descrip_size;
    std::string type;
    MatcherParam(){
        quality = 5.0;
        ratio = 0.8;
        descrip_size = 128;
        type = "sift";
    }

    MatcherParam( double q, double r, int d, std::string t ) {
        quality = q;
        ratio = r;
        descrip_size = d;
        type = t;
    }
};


//! cluster parameters
struct ClusterParam{
    double radius;
    double merge;
    int minpts;
    int maxiters;

    ClusterParam() {
        radius = 200;
        merge = 20;
        minpts = 7;
        maxiters = 100;
    }

    ClusterParam( double r, double mg, int mp, int mi ) {
        radius = r;
        merge = mg;
        minpts = mp;
        maxiters = mi;
    }
};

//! levenburg marquat parameters
struct LevmarParam{
    int max_ransac;
    int max_lm;
    int max_objs_per_cluster;
    int n_pts_align;
    int min_pts_per_obj;
    double error_threshold;
    Vector4f camera_param;
    LevmarParam() {
        max_ransac = 600;
        max_lm = 200;
        max_objs_per_cluster = 4;
        n_pts_align = 5;
        min_pts_per_obj = 6;
        error_threshold = 10;
        camera_param << 500., 500., 319.5, 239.5;
    }

    LevmarParam( int mr, int ml, int mopc, int npa, int mppo, double et, float fx, float fy, float cx, float cy ) {
        max_ransac = mr;
        max_lm = ml;
        max_objs_per_cluster = mopc;
        n_pts_align = npa;
        min_pts_per_obj = mppo;
        error_threshold = et;
        camera_param << fx, fy, cx, cy;
    }
};

//! projection parameters
struct ProjParam{
    int min_pts;
    double feat_distance;
    double min_score;
    Vector4f camera_param;
    ProjParam() {
        min_pts = 5;
        feat_distance = 4096.;
        min_score = 2;
        camera_param << 500., 500., 319.5, 239.5;
    }

    ProjParam( int mp, double fd, double ms, float fx, float fy, float cx, float cy ) {
        min_pts = mp;
        feat_distance = fd;
        min_score = ms;
        camera_param << fx, fy, cx, cy;
    }
};

//! RGB parameters
struct RGBParam{
    DetectorParam dp;
    MatcherParam mp;
    ClusterParam cp;
    LevmarParam lmp1;
    LevmarParam lmp2;
    ProjParam pp1;
    ProjParam pp2;

    RGBParam(DetectorParam d, MatcherParam m, ClusterParam c, LevmarParam l1, LevmarParam l2, ProjParam p1, ProjParam p2 ){
        dp = d;
        mp = m;
        cp = c;
        lmp1 = l1;
        lmp2 = l2;
        pp1 = p1;
        pp2 = p2;
    }

    RGBParam(){
    }
};

struct SVDParam{
    double error;
    int minpts;
};

//! RGBD Parameters
struct RGBDParam{
    DetectorParam dp;
    MatcherParam mp;
    ClusterParam cp;
    SVDParam sp;
    Vector4f camera_p;

    RGBDParam( DetectorParam d, MatcherParam m, ClusterParam c, SVDParam s ) {
        dp = d;
        mp = m;
        cp = c;
        sp = s;
    }

    RGBDParam(){
    }
};


// functions
cv::Point2f project( const Pose &pose, const pcl::PointXYZ & pt3d, const Vector4f & params );

Vector2f project( const Pose & pose, const Vector3f & vec3d, const Vector4f & params );

// functions for each step, RGBD
/** sift feature extraction using libsiftfast
  * @param rgb image
  * @param point cloud
  * @return detected features
  */
void display_features( const cv::Mat & image, const vector<DetectedFeatureRGBD> & features, bool clustered_flag );

/** rgbd segmentation
  * @param rgb point cloud
  * @param detected features
  * @param cluster
  * @return labeled point cloud
  */
//pcl::PointCloud<pcl::PointXYZRGBL>::Ptr segment_point_cloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, vector<DetectedFeatureRGBD> & detected_features, vector< vector<int> > & clusters );
void display_labeled_cloud( pcl::PointCloud<pcl::PointXYZRGBL>::Ptr labeled_cloud, int label );

/** match descriptors
  * @param detected features
  * @param models
  * @param cluster matches indices
  * @return matches
  */
void display_matches(const cv::Mat &image, const vector<MatchRGBD> &matches );

// display rgbd matches
void display_clusters(const cv::Mat &image, const vector<MatchRGBD> &matches, vector<list<int> > &clusters);

/** pose estimation using svd
  * @param matches
  * @param clusters
  * @param models
  * @return objects
  */
const float g_pose_svd_error  = 0.005;
const int   g_pose_svd_minpts = 7;
//list<SP_Object> svd_pose_estimation_recog( const vector< vector<MatchRGBD> > & matches, const vector<SP_Model> & models, const vector< vector<int> > & clusters );
//pair<float, int> svd_pose_estimation_core( vector<MatchRGBD> & matches, Pose & pose );
//bool svd_pose_estimation_veri( vector<MatchRGBD> & matches, Pose & pose );
void display_pose(const cv::Mat &image, const list<SP_Object> &objects, const Vector4f & cp);

// functions for each step, RGB recogniser

/** ************************************************************************* */
// display mask image
void display_mask( const cv::Mat & mask_image );

//vector<DetectedFeatureRGB> detect_features_rgb( const cv::Mat & image );
void display_features( const cv::Mat & image, const vector< DetectedFeatureRGB > & features );


// match rgb features
void display_matches(const cv::Mat &image, const vector<MatchRGB> &matches );


// display cluster features
void display_clusters( const cv::Mat & image, const vector<MatchRGB> & matches, vector< list<int> > & clusters, cv::Point2i tl, string win_name = "cluster" );


// find apperance times in the bin contents
vector<pair<string, int> > duplicated_bin_contents(const vector<string> &bin_contents);


template<int N>
struct Pt {
    float p[N];


    Pt<N>& init(float *pt) { memcpy( p, pt, N*sizeof(float) ); return *this; }

    template<typename T> Pt<N>& init(T p0) { p[0]=p0; return *this; };
    template<typename T> Pt<N>& init(T p0, T p1) { p[0]=p0; p[1]=p1; return *this; };
    template<typename T> Pt<N>& init(T p0, T p1, T p2 ) { p[0]=p0; p[1]=p1; p[2]=p2; return *this; };
    template<typename T> Pt<N>& init(T p0, T p1, T p2, T p3 ) { p[0]=p0; p[1]=p1; p[2]=p2; p[3]=p3; return *this; };

    float& operator[] (int n) { return p[n]; }
    const float& operator[] (int n) const { return p[n]; }

    operator float *() { return p; }
    operator const float *() const { return p; }

    bool operator< ( const Pt<N> &pt ) const { for(int x=0; x<N; x++) if((*this)[x]!=pt[x]) return (*this)[x]<pt[x]; return false; }
    bool operator==( const Pt<N> &pt ) const { for(int x=0; x<N; x++) if((*this)[x]!=pt[x]) return false; return true; }

    Pt<N>& operator*= ( const Pt<N> &pt ) { for(int x=0; x<N; x++) (*this)[x]*=pt[x]; return *this; }
    Pt<N>& operator/= ( const Pt<N> &pt ) { for(int x=0; x<N; x++) (*this)[x]/=pt[x]; return *this; }
    Pt<N>& operator+= ( const Pt<N> &pt ) { for(int x=0; x<N; x++) (*this)[x]+=pt[x]; return *this; }
    Pt<N>& operator-= ( const Pt<N> &pt ) { for(int x=0; x<N; x++) (*this)[x]-=pt[x]; return *this; }

    template<typename T> Pt<N>& operator*= ( const T f ) { for(int x=0; x<N; x++) (*this)[x]*=f; return *this; }
    template<typename T> Pt<N>& operator/= ( const T f ) { for(int x=0; x<N; x++) (*this)[x]/=f; return *this; }
    template<typename T> Pt<N>& operator+= ( const T f ) { for(int x=0; x<N; x++) (*this)[x]+=f; return *this; }
    template<typename T> Pt<N>& operator-= ( const T f ) { for(int x=0; x<N; x++) (*this)[x]-=f; return *this; }

    template<typename T> Pt<N> operator*( const T &pt ) const {  Pt<N> r=*this; return r*=pt; }
    template<typename T> Pt<N> operator/( const T &pt ) const {  Pt<N> r=*this; return r/=pt; }
    template<typename T> Pt<N> operator+( const T &pt ) const {  Pt<N> r=*this; return r+=pt; }
    template<typename T> Pt<N> operator-( const T &pt ) const {  Pt<N> r=*this; return r-=pt; }

    float sqEuclDist( const Pt<N> &pt ) const {	float d, r=0; for(int x=0; x<N; r+=d*d, x++) d=pt[x]-(*this)[x]; return r; }
    float euclDist( const Pt<N> &pt ) const { return sqrt(sqEuclDist(pt)); }
    Pt<N>& norm() { float d=0; for(int x=0; x<N; x++) d+=(*this)[x]*(*this)[x]; d=1./sqrt(d); for(int x=0; x<N; x++) (*this)[x]*=d; return *this; }

    friend ostream& operator<< (ostream &out, const Pt<N> &pt) { out<<"["; for(int x=0; x<N; x++) out<<(x==0?"":" ")<<pt[x]; out<<"]"; return out; }
    friend istream& operator>> (istream &in, Pt<N> &pt) { for(int x=0; x<N; x++) in>>pt[x]; return in; }

    Pt<N> &min( Pt<N> &p2) { for(int x=0; x<N; x++) (*this)[x]=std::min((*this)[x], p2[x]); return *this;}
    Pt<N> &max( Pt<N> &p2) { for(int x=0; x<N; x++) (*this)[x]=std::max((*this)[x], p2[x]); return *this;}

};


list< Pt<2> > getConvexHull( vector< Pt<2> > &points );


vector<cv::Point> get_convex_hull( vector<cv::Point> & points );

#endif // UTILS_H
