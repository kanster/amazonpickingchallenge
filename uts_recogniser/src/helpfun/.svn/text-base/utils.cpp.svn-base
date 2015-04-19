#include "include/helpfun/utils.h"

// functions
inline Vector3f string_to_vector3f(string str) {
    Vector3f vec;
    vector<string> strs;
    boost::algorithm::split( strs, str, boost::algorithm::is_any_of(" ") );
    vec(0) = boost::lexical_cast<float>(strs[0]);
    vec(1) = boost::lexical_cast<float>(strs[1]);
    vec(2) = boost::lexical_cast<float>(strs[2]);
    strs.clear();
    return vec;
}

inline vector<float> string_to_vector(string str) {
    vector<float> vec;
    vector<string> strs;
    boost::algorithm::split( strs, str, boost::algorithm::is_any_of(" ") );
    for ( size_t i = 0; i < strs.size(); ++ i )
        vec.push_back( boost::lexical_cast<float>(strs[i]) );
    strs.clear();
    return vec;
}


/* ------------------------------------------------------ */
// Model
// constructor
Model::Model() {
}

// destructor
Model::~Model() {
    this->features_.clear();
}

void Model::load_model( string filename ) {
    boost::property_tree::ptree ptree;
    boost::property_tree::xml_parser::read_xml( filename, ptree );
    string str;
    // bounding_box_[0] = min bounding_box_[1] = max
    this->bounding_box_[0] << 1000., 1000., 1000.;
    this->bounding_box_[1] << -1000., -1000., -1000.;
    BOOST_FOREACH( boost::property_tree::ptree::value_type & v, ptree.get_child("Model") ) {
        if ( v.first == "<xmlattr>" )
            this->name_ = v.second.get<string>("name");
        if ( v.first == "Points" ) {
            BOOST_FOREACH( boost::property_tree::ptree::value_type const & u, v.second ) {
                if ( u.first == "Point" ) {
                    Model::Feature feat;
                    str = u.second.get("<xmlattr>.p3d", "");
                    feat.coord3d = string_to_vector3f( str );
                    str = u.second.get("<xmlattr>.color", "");
                    feat.color   = string_to_vector3f( str );
                    str = u.second.get("<xmlattr>.desc", "");
                    feat.descrip = string_to_vector( str );
                    this->features_.push_back( feat );
                    // find the bounding_box_
                    bounding_box_[0](0) = feat.coord3d(0) < bounding_box_[0](0)? feat.coord3d(0):bounding_box_[0](0);
                    bounding_box_[0](1) = feat.coord3d(1) < bounding_box_[0](1)? feat.coord3d(1):bounding_box_[0](1);
                    bounding_box_[0](2) = feat.coord3d(2) < bounding_box_[0](2)? feat.coord3d(2):bounding_box_[0](2);
                    bounding_box_[1](0) = feat.coord3d(0) > bounding_box_[1](0)? feat.coord3d(0):bounding_box_[1](0);
                    bounding_box_[1](1) = feat.coord3d(1) > bounding_box_[1](1)? feat.coord3d(1):bounding_box_[1](1);
                    bounding_box_[1](2) = feat.coord3d(2) > bounding_box_[1](2)? feat.coord3d(2):bounding_box_[1](2);
                }
            }
        }
    }
}

bool Model::operator < ( const Model & model ) const {
    return this->name_ < model.name_;
}

/* ------------------------------------------------------ */
// Pose
// constructor
Pose::Pose() {
}

Pose::Pose(Quaternionf q, Translation3f t) {
    this->q_ = q;
    this->t_ = t;
    this->r_ = q.toRotationMatrix();
}

// output
ostream & operator << ( ostream & out, const Pose & pose ) {
    out << "[" << pose.q_.w() << ", " << pose.q_.x() << ", " << pose.q_.y() << ", " << pose.q_.z() << "], [" << pose.t_.x() << ", " << pose.t_.y() << ", " << pose.t_.z() << "]";
    return out;
}

/* ------------------------------------------------------ */
// TranformMatrix
// constructor
void TransformMatrix::init(const Quaternionf &q, const Translation3f &t) {
    this->p_.topLeftCorner(3, 3) = q.toRotationMatrix();
    this->p_(0,3) = t.x(); this->p_(1,3) = t.y(); this->p_(2,3) = t.z();
    this->p_(3,0) = 0.0; this->p_(3,1) = 0.0; this->p_(3,2) = 0.0; this->p_(3,3) = 1.0;
}

void TransformMatrix::init(const Pose &pose) {
    init( pose.q_, pose.t_ );
}

// transform
Vector3f TransformMatrix::transform(Vector3f orig) {
    Vector4f orig4, dest4;
    orig4.topLeftCorner(3,1) = orig;
    orig4(3,0) = 1.0;
    dest4 = this->p_*orig4;
    Vector3f dest = dest4.topLeftCorner(3,1);
    return dest;
}

Vector3f TransformMatrix::inverse_transform(Vector3f orig) {
    Vector3f diff;
    diff = orig - this->p_.topRightCorner(3,1);
    Vector3f dest = (this->p_.topLeftCorner(3,3)).inverse()*diff;
    return dest;
}

// output
ostream & operator << ( ostream & out, const TransformMatrix & tm ) {
    out << tm.p_;
    return out;
}

/* ------------------------------------------------------ */
// Object
vector<cv::Point> Object::get_object_hull(vector<cv::Point> pts) {
    vector<cv::Point> hull_pts;
    cv::convexHull( pts, hull_pts );
    return hull_pts;
}

bool Object::operator <( const Object & o ) const {
    if ( this->model_->name_ != o.model_->name_ )
        return this->model_->name_ < o.model_->name_;
    else
        return this->score_ < o.score_;
}

ostream & operator << ( ostream & out, const Object & obj ) {
    pcl::console::print_highlight("Object ");
    cout << obj.model_->name_ << " with pose translation in local ";
    pcl::console::print_value("[ %f, %f, %f ]", obj.pose_.t_.x(), obj.pose_.t_.y(), obj.pose_.t_.z());
    return out;
}

bool compare_sp_object(const SP_Object &spo_l, const SP_Object &spo_r) {
    return spo_l->score_ > spo_r->score_;
}
/* ------------------------------------------------------ */
// project 3d points to 2d
cv::Point2f project(const Pose &pose, const pcl::PointXYZ &pt3d, const Vector4f &params) {
    TransformMatrix pose_tm;
    pose_tm.init( pose );
    Vector3f pt3d_env = pose_tm.transform( Vector3f(pt3d.x, pt3d.y, pt3d.z) );
    cv::Point2f pt2d;
    pt2d.x = (pt3d_env(0)/pt3d_env(2)*params(0)) + (int)params(2);
    pt2d.y = (pt3d_env(1)/pt3d_env(2)*params(1)) + (int)params(3);
    return pt2d;
}

Vector2f project(const Pose &pose, const Vector3f &vec3d, const Vector4f &params) {
    pcl::PointXYZ pt3d;
    pt3d.x = vec3d(0);
    pt3d.y = vec3d(1);
    pt3d.z = vec3d(2);
    Vector2f vec2d;
    cv::Point2f pt2d;
    pt2d = project( pose, pt3d, params );
    vec2d(0) = pt2d.x;
    vec2d(1) = pt2d.y;
    return vec2d;
}

// detect sift features using libsiftfast
void display_features(const cv::Mat & image, const vector<DetectedFeatureRGBD> & features, bool clustered_flag) {
    cv::Mat rgb_image = image.clone();
    if ( clustered_flag == false ) {
        for ( size_t i = 0; i < features.size(); i ++ )
            cv::circle( rgb_image, cv::Point(features[i].img2d(0), features[i].img2d(1)), 3, cv::Scalar(128, 0, 255), CV_FILLED );
    }
    else {
        vector<cv::Scalar> colors;
        for ( int i = 0; i < 255; i ++ )
            colors.push_back( cv::Scalar( rand()%255, rand()%255, rand()%255 ) );
        for ( size_t i = 0; i < features.size(); i ++ ) {
            cv::circle( rgb_image, cv::Point(features[i].img2d(0), features[i].img2d(1)), 4, colors[features[i].groupidx], CV_FILLED );
        }
    }

    cv::namedWindow( "feature", CV_WINDOW_NORMAL );
    cv::resizeWindow( "feature", 640, 480 );
    cv::moveWindow( "feature", 0, 0 );
    cv::imshow( "feature", rgb_image );
    cv::waitKey(10);
    rgb_image.release();
}

void display_labeled_cloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr labeled_cloud, const cv::Mat & image, int label) {
    cv::Mat rgb_image = image.clone();
    int max_label = 0;
    for ( size_t i = 0; i < labeled_cloud->points.size(); i++ )
        if ( (int)labeled_cloud->points[i].label > max_label )
            max_label = labeled_cloud->points[i].label;
    vector<cv::Scalar> colors;
    for ( int i = 0; i <= max_label; i ++ )
        colors.push_back( cv::Scalar(rand()%255, rand()%255, rand()%255 ) );
    if ( label == -1 ) {
        for ( int y = 0; y < rgb_image.rows; ++ y ) {
            for ( int x = 0; x < rgb_image.cols; ++ x ) {
                cv::Vec3b &cvp = rgb_image.at<cv::Vec3b>(y, x);
                int position = y*640+x;
                const pcl::PointXYZRGBL &pt = labeled_cloud->points[position];
                cvp[2] = colors[pt.label][2];
                cvp[1] = colors[pt.label][1];
                cvp[0] = colors[pt.label][0];
            }
        }
    }
    else {
        for ( int y = 0; y < rgb_image.rows; ++ y ) {
            for ( int x = 0; x < rgb_image.cols; ++ x ) {
                cv::Vec3b &cvp = rgb_image.at<cv::Vec3b>(y, x);
                int position = y*640+x;
                const pcl::PointXYZRGBL &pt = labeled_cloud->points[position];
                if ( (int)pt.label == label ) {
                    cvp[2] = colors[pt.label][2];
                    cvp[1] = colors[pt.label][1];
                    cvp[0] = colors[pt.label][0];
                }
            }
        }
    }
    cv::imshow( "labeled cloud", rgb_image );
    cv::waitKey(0);
    cv::destroyWindow( "labeled cloud" );
    rgb_image.release();
    colors.clear();
}

void display_matches(const cv::Mat &image, const vector<MatchRGBD> &matches ) {
    cv::Mat rgb_image = image.clone();
    cv::Scalar color( 255, 0, 0 );
    for ( size_t i = 0; i < matches.size(); ++ i ) {
        const MatchRGBD & m = matches[i];
        cv::circle( rgb_image, cv::Point( m.img2d(0), m.img2d(1) ), 3, color, 2 );
    }
    cv::namedWindow( "matches", CV_WINDOW_NORMAL );
    cv::resizeWindow( "matches", 640, 480 );
    cv::moveWindow( "matches", 0, 480);
    cv::imshow( "matches", rgb_image );
    cv::waitKey( 10 );
    rgb_image.release();
}

void display_clusters(const cv::Mat &image, const vector<MatchRGBD> &matches, vector<list<int> > &clusters) {
    cv::Mat rgb_image = image.clone();
    for ( size_t i = 0; i < clusters.size(); ++ i ) {
        cv::Scalar color( rand()%255, rand()%255, rand()%255 );
        for ( list<int>::iterator v = clusters[i].begin(); v != clusters[i].end(); ++ v ) {
            const MatchRGBD & m = matches[*v];
            cv::circle( rgb_image, cv::Point( m.img2d(0), m.img2d(1) ), 3, color, 2 );
        }
    }
    cv::namedWindow( "clusters", CV_WINDOW_NORMAL );
    cv::resizeWindow( "clusters", 640, 480 );
    cv::moveWindow( "clusters", 640, 0);
    cv::imshow( "clusters", rgb_image );

    cv::waitKey(10);
    rgb_image.release();
}

void display_pose(const cv::Mat &image, const list<SP_Object> &objects, const Vector4f & cp) {
    cv::Mat rgb_image = image.clone();
    vector<cv::Scalar> colors( objects.size() );
    for ( size_t i = 0; i < colors.size(); ++ i )
        colors[i] = cv::Scalar(rand()%255, rand()%255, rand()%255);
    int it = 0;
    foreach( obj, objects ) {
        pcl::PointXYZ bb[8];
        bb[0].x = obj->model_->bounding_box_[0](0);
        bb[0].z = obj->model_->bounding_box_[0](2);
        bb[0].y = obj->model_->bounding_box_[0](1);
        bb[1].x = obj->model_->bounding_box_[0](0);
        bb[1].z = obj->model_->bounding_box_[1](2);
        bb[1].y = obj->model_->bounding_box_[0](1);
        bb[2].x = obj->model_->bounding_box_[1](0);
        bb[2].z = obj->model_->bounding_box_[1](2);
        bb[2].y = obj->model_->bounding_box_[0](1);
        bb[3].x = obj->model_->bounding_box_[1](0);
        bb[3].z = obj->model_->bounding_box_[0](2);
        bb[3].y = obj->model_->bounding_box_[0](1);
        bb[4].x = obj->model_->bounding_box_[0](0);
        bb[4].z = obj->model_->bounding_box_[0](2);
        bb[4].y = obj->model_->bounding_box_[1](1);
        bb[5].x = obj->model_->bounding_box_[0](0);
        bb[5].z = obj->model_->bounding_box_[1](2);
        bb[5].y = obj->model_->bounding_box_[1](1);
        bb[6].x = obj->model_->bounding_box_[1](0);
        bb[6].z = obj->model_->bounding_box_[1](2);
        bb[6].y = obj->model_->bounding_box_[1](1);
        bb[7].x = obj->model_->bounding_box_[1](0);
        bb[7].z = obj->model_->bounding_box_[0](2);
        bb[7].y = obj->model_->bounding_box_[1](1);
        cv::Point pt[8];
        for ( int i = 0; i < 8; i ++ )
            pt[i] = project( obj->pose_, bb[i], cp );
        cv::Scalar & color = colors[it];
        cv::line( rgb_image, pt[0], pt[1], color, 2 );
        cv::line( rgb_image, pt[1], pt[2], color, 2 );
        cv::line( rgb_image, pt[2], pt[3], color, 2 );
        cv::line( rgb_image, pt[3], pt[0], color, 2 );
        cv::line( rgb_image, pt[4], pt[5], color, 2 );
        cv::line( rgb_image, pt[5], pt[6], color, 2 );
        cv::line( rgb_image, pt[6], pt[7], color, 2 );
        cv::line( rgb_image, pt[7], pt[4], color, 2 );
        cv::line( rgb_image, pt[0], pt[4], color, 2 );
        cv::line( rgb_image, pt[1], pt[5], color, 2 );
        cv::line( rgb_image, pt[2], pt[6], color, 2 );
        cv::line( rgb_image, pt[3], pt[7], color, 2 );
        it ++;
    }
    cv::namedWindow( "pose", CV_WINDOW_NORMAL );
    cv::resizeWindow( "pose", 640, 480 );
    cv::moveWindow( "pose", 640, 480 );
    cv::imshow( "pose", rgb_image );
    cv::waitKey(10);
    rgb_image.release();
}


// functions for each step, RGB recogniser
/** ************************************************************************* */
// display mask image
void display_mask(const cv::Mat &mask_image) {
    cv::namedWindow( "mask" );
    cv::imshow( "mask", mask_image );
    cv::waitKey(10);
};

// display rgb features
void display_features(const cv::Mat &image, const vector<DetectedFeatureRGB> &features) {
    cv::Mat rgb_image = image.clone();
    for ( size_t i = 0; i < features.size(); i ++ )
        cv::circle( rgb_image, cv::Point(features[i].img2d(0), features[i].img2d(1)), 3, cv::Scalar(128, 0, 255), 2 );
    cv::namedWindow( "feature", CV_WINDOW_NORMAL );
    cv::resizeWindow( "feature", 640, 480 );
    cv::moveWindow( "feature", 0, 0 );
    cv::imshow( "feature", rgb_image );
    cv::waitKey(10);
    rgb_image.release();
}

// display matches
void display_matches(const cv::Mat &image, const vector<MatchRGB> &matches) {
    cv::Mat rgb_image = image.clone();
    cv::Scalar color( 255, 0, 0 );
    for ( size_t i = 0; i < matches.size(); ++ i ) {
        const MatchRGB & m = matches[i];
        cv::circle( rgb_image, cv::Point( m.img2d(0), m.img2d(1) ), 3, color, 2 );
    }
    cv::namedWindow( "matches", CV_WINDOW_NORMAL );
    cv::resizeWindow( "matches", 640, 480 );
    cv::moveWindow( "matches", 0, 480 );
    cv::imshow( "matches", rgb_image );
    cv::waitKey( 10 );
    rgb_image.release();
}

// display clusters
void display_clusters(const cv::Mat &image, const vector<MatchRGB> &matches, vector<list<int> > &clusters, cv::Point2i tl, string win_name) {
    cv::Mat rgb_image = image.clone();
    for ( size_t i = 0; i < clusters.size(); ++ i ) {
        cv::Scalar color( rand()%255, rand()%255, rand()%255 );
        for ( list<int>::iterator v = clusters[i].begin(); v != clusters[i].end(); ++ v ) {
            const MatchRGB & m = matches[*v];
            cv::circle( rgb_image, cv::Point( m.img2d(0), m.img2d(1) ), 3, color, 2 );
        }
    }
    cv::namedWindow( win_name, CV_WINDOW_NORMAL );
    cv::resizeWindow( win_name, 640, 480 );
    cv::moveWindow( win_name, tl.x, tl.y );
    cv::imshow( win_name, rgb_image );
    cv::waitKey(10);
    rgb_image.release();
}


// deprecate item and numbers
vector<pair<string, int> > deprecate_bin_contents(const vector<string> &bin_contents) {
    map<string, int> items_map;
    for ( int i = 0; i < (int)bin_contents.size(); ++ i ) {
        if ( items_map.find( bin_contents[i] ) == items_map.end() )
            items_map[bin_contents[i]] = 1;
        else
            items_map[bin_contents[i]] = items_map[bin_contents[i]]+1;
    }
    vector<pair<string, int> > items_n;
    for ( map<string, int>::iterator it = items_map.begin(); it != items_map.end(); ++ it ) {
        items_n.push_back( make_pair(it->first, it->second) );
    }
    return items_n;
}








