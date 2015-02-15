#include "include/utils.h"

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
            cv::circle( rgb_image, cv::Point(features[i].img2d(0), features[i].img2d(1)), 3, cv::Scalar(128, 0, 255), 2 );
    }
    else {
        vector<cv::Scalar> colors;
        for ( int i = 0; i < 255; i ++ )
            colors.push_back( cv::Scalar( rand()%255, rand()%255, rand()%255 ) );
        for ( size_t i = 0; i < features.size(); i ++ ) {
            cv::circle( rgb_image, cv::Point(features[i].img2d(0), features[i].img2d(1)), 4, colors[features[i].groupidx], CV_FILLED );
        }
    }

    cv::namedWindow( "feature" );
    cv::imshow( "feature", rgb_image );
    cv::waitKey(0);
    rgb_image.release();
}

// segment rgb point cloud
/*
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr segment_point_cloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, vector<DetectedFeatureRGBD> & detected_features, vector< vector<int> > & clusters ) {
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr ori_labeled_cloud( new pcl::PointCloud<pcl::PointXYZRGBL> );
    segment::SegmenterLight seg("../externals/model/");
    seg.setFast(false);
    seg.setDetail(0);
    ori_labeled_cloud = seg.processPointCloud(cloud);

    // obtain the group numbers
    int max_label = 0;
    for ( size_t i = 0; i < ori_labeled_cloud->points.size(); ++ i )
        if ( (int)ori_labeled_cloud->points[i].label > max_label )
            max_label = ori_labeled_cloud->points[i].label;

    // find how many features in a given group
    vector<int> labels_counter(max_label+1, 0);
    vector<int> useless_labels; // the labels do not have enough number of features
    vector<int> useful_labels;  // the labels contain enough number of features
    for ( size_t i = 0; i < detected_features.size(); ++ i ) {
        int idx = (int)(detected_features[i].img2d(1))*640+(int)(detected_features[i].img2d(0));
        labels_counter[ori_labeled_cloud->points[idx].label] ++;
    }
    for ( size_t i = 0; i < labels_counter.size(); ++ i ) {
        if ( labels_counter[i] <= 10 )  // if the feature number is less than 10, set it as useless
            useless_labels.push_back( i );
        else
            useful_labels.push_back(i);
    }

    // generate new labeled point cloud
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr labeled_cloud( new pcl::PointCloud<pcl::PointXYZRGBL> );
    for ( size_t i = 0; i < ori_labeled_cloud->points.size(); ++ i ) {
        const pcl::PointXYZRGBL & pt = ori_labeled_cloud->points[i];
        pcl::PointXYZRGBL npt = pt;
        if ( find( useless_labels.begin(), useless_labels.end(), pt.label ) != useless_labels.end() )
            npt.label = 0;
        else {
            int pos = find(useful_labels.begin(), useful_labels.end(), pt.label)-useful_labels.begin()+1;
            npt.label = pos;
        }
        labeled_cloud->points.push_back( npt );
    }
    labeled_cloud->width = 640;
    labeled_cloud->height = 480;

    // generate clusters
    clusters.resize( useful_labels.size()+1 );
    for ( size_t i = 0; i < detected_features.size(); ++ i ) {
        int idx = (int)(detected_features[i].img2d(1))*640+(int)(detected_features[i].img2d(0));
        detected_features[i].groupidx = labeled_cloud->points[idx].label;
        clusters[labeled_cloud->points[idx].label].push_back(i);
    }

    labels_counter.clear();
    useless_labels.clear();
    useful_labels.clear();

    return labeled_cloud;
}
*/
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

// match sift features
/*
vector< vector<MatchRGBD> > match_features_rgbd( vector<DetectedFeatureRGBD> & detected_features, vector<SP_Model> & models, vector< vector<int> > & match_clusters ) {
    // build kdtree
    unsigned int model_feat_number = 0;
    for ( size_t i = 0; i < models.size(); ++ i ) {
        model_feat_number += models[i]->features_.size();
    }

    vector<int> corresp_model_idx( model_feat_number );
    vector<int> corresp_feat_idx( model_feat_number );
    vector< Vector3f * > corresp_model_feat( model_feat_number );
    ANNpointArray model_descrips = annAllocPts( model_feat_number, 128 );
    int idx = 0;
    for ( size_t i_model = 0; i_model < models.size(); ++ i_model ) {
        SP_Model m = models[i_model];
        for ( size_t i_feat = 0; i_feat < m->features_.size(); ++ i_feat ) {
            corresp_model_idx[idx]  = i_model;
            corresp_model_feat[idx] = &m->features_[i_feat].coord3d;
            corresp_feat_idx[idx]   = i_feat;
            normaliser( m->features_[i_feat].descrip );
            for ( size_t i_desc = 0; i_desc < 128; ++ i_desc )
                model_descrips[idx][i_desc] = m->features_[i_feat].descrip[i_desc];
            idx ++;
        }
    }
    ANNkd_tree * kdtree = new ANNkd_tree( model_descrips, model_feat_number, 128 );

    // init  clusters and matches
    vector< vector<MatchRGBD> > matches;
    matches.resize( models.size() );
    ANNpoint pt = annAllocPt( 128 );
    ANNidxArray nx  = new ANNidx[2];
    ANNdistArray ds = new ANNdist[2];
    for ( size_t i = 0; i < detected_features.size(); ++ i ) {
        normaliser( detected_features[i].descrip );
        for ( size_t j = 0; j < 128; ++ j )
            pt[j] = detected_features[i].descrip[j];
        #pragma omp critical(ANN)
        kdtree->annkSearch( pt, 2, nx, ds, 5 );
        if ( ds[0]/ds[1] < 0.8 ) {
            MatchRGBD match;
            match.img2d = detected_features[i].img2d;
            match.obs3d = detected_features[i].obs3d;
            match.mdl3d = models[corresp_model_idx[nx[0]]]->features_[corresp_feat_idx[nx[0]]].coord3d;
            match.mdlidx = make_pair<int, int>( corresp_model_idx[nx[0]], corresp_feat_idx[nx[0]] );
            match.cluidx = detected_features[i].groupidx;
            match.obsidx = i;
            matches[corresp_model_idx[nx[0]]].push_back( match );
            match_clusters[detected_features[i].groupidx].push_back( corresp_model_idx[nx[0]] );
        }
        else {
            match_clusters[detected_features[i].groupidx].push_back(-1);
        }
    }

    delete kdtree;
    return matches;
}
*/

void display_matches(const cv::Mat &image, const vector<vector<MatchRGBD> > &matches ) {
    cv::Mat rgb_image = image.clone();
    vector<cv::Scalar> colors( matches.size() );
    for ( size_t i = 0; i < colors.size(); ++ i )
        colors[i] = cv::Scalar(rand()%255, rand()%255, rand()%255);
    for ( size_t i = 0; i < matches.size(); ++ i ) {
        for ( size_t j = 0; j < matches[i].size(); ++ j ) {
            const MatchRGBD & m = matches[i][j];
            cv::circle( rgb_image, cv::Point(m.img2d(0), m.img2d(1)), 4, colors[i], CV_FILLED );
        }
    }
    cv::namedWindow( "matches" );
    cv::imshow( "matches", rgb_image );
    cv::waitKey(0);
    cv::destroyWindow("matches");
    rgb_image.release();
    colors.clear();

}

/*
// pose estimation core
pair<float, int> svd_pose_estimation_core( vector<MatchRGBD> & matches, Pose & pose) {
    // initialisation
    MatrixXf mdl_pts( matches.size(), 3 ), obs_pts( matches.size(), 3 );
    Vector3f mdl_ave(0.0, 0.0, 0.0), obs_ave(0.0, 0.0, 0.0);
    for ( size_t i = 0; i < matches.size(); ++ i ) {
        const MatchRGBD & m = matches[i];
        mdl_pts.row(i) = m.mdl3d.transpose();
        obs_pts.row(i) = m.obs3d.transpose();
        mdl_ave = mdl_ave + (mdl_pts.row(i)).transpose();
        obs_ave = obs_ave + (obs_pts.row(i)).transpose();
    }
    mdl_ave = mdl_ave/matches.size();
    obs_ave = obs_ave/matches.size();
    for ( size_t y = 0; y < matches.size(); ++ y ) {
        mdl_pts.row(y) = mdl_pts.row(y)-mdl_ave.transpose();
        obs_pts.row(y) = obs_pts.row(y)-obs_ave.transpose();
    }

    Matrix3f sum_product = Matrix3f::Zero();
    for ( size_t i = 0; i < matches.size(); ++ i )
    {
        Vector3f mdl_vec = mdl_pts.row(i);
        Vector3f obs_vec = obs_pts.row(i);
        sum_product = sum_product + mdl_vec*(obs_vec.transpose());
    }

    // svd decomposition
    Matrix3f rotation;
    JacobiSVD<MatrixXf> svd( sum_product, ComputeThinU|ComputeThinV );
    MatrixXf svdu = svd.matrixU();
    MatrixXf svdv = svd.matrixV();
    MatrixXf tmp = svdv*(svdu.transpose());
    if ( abs(tmp.determinant()-1) < 0.1 )
        rotation = svdv*(svdu.transpose());
    else
    {
        for ( int i = 0; i < 3; i ++ )
            svdv(i, 2) *= -1.0;
        rotation = svdv*(svdu.transpose());
    }
    Vector3f translation = obs_ave - rotation*mdl_ave;
    pose.r_ = rotation;
    pose.t_ = Translation3f( translation );
    pose.q_ = Quaternionf( rotation );
    float max_error = 0.0;
    int max_idx  = -1;
    TransformMatrix tm;
    tm.init( pose );
    for ( size_t i = 0; i < matches.size(); ++ i ) {
        const MatchRGBD & m = matches[i];
        Vector3f tpt = tm.transform( m.mdl3d );
        float err = (tpt-m.obs3d).norm();
        if ( err > max_error ) {
            max_error = err;
            max_idx   = i;
        }
    }
    return make_pair<float, int>(max_error, max_idx);
}

bool svd_pose_estimation_veri( vector<MatchRGBD> & matches, Pose & pose) {
    Pose init_pose;
    pair<float, int> max_error = svd_pose_estimation_core( matches, init_pose );
    while ( (float)max_error.first > g_pose_svd_error && (int)max_error.second > g_pose_svd_minpts  ) {
        matches.erase( matches.begin() + max_error.second );
        max_error = svd_pose_estimation_core( matches, init_pose );
    }
    if ( max_error.first > g_pose_svd_error && max_error.second > g_pose_svd_minpts ) {
        pose = init_pose;
        return true;
    }
    else
        return false;
}

list<SP_Object> svd_pose_estimation_recog(const vector<vector<MatchRGBD> > &matches, const vector<SP_Model> &models, const vector<vector<int> > &clusters) {
    list<SP_Object> objects;
    for ( size_t i = 0; i < matches.size(); ++ i ) {
        vector< vector<MatchRGBD> > clustered_matches( clusters.size() );
        for ( size_t j = 0; j < matches[i].size(); ++ j )
            clustered_matches[matches[i][j].cluidx].push_back( matches[i][j] );
        for ( size_t j = 0; j < clustered_matches.size(); ++ j ) {
            if ( (int)clustered_matches[j].size() > g_pose_svd_minpts ) {
                // graph filter
                MatrixXf relation_graph( clustered_matches[j].size(), clustered_matches[j].size() );
                vector<int> ranked_n( relation_graph.rows(), 0 );
                for ( int y = 0; y < relation_graph.rows(); ++ y ) {
                    for ( int x = 0; x < relation_graph.cols(); ++ x ) {
                        if ( x != y ) {
                            Vector3f obs_diff = clustered_matches[j][y].obs3d - clustered_matches[j][x].obs3d;
                            Vector3f mdl_diff = clustered_matches[j][y].mdl3d - clustered_matches[j][x].mdl3d;
                            float diff = obs_diff.norm() - mdl_diff.norm();
                            if ( abs(diff) > g_pose_svd_error ) {
                                relation_graph(y, x) = 0.0;
                                relation_graph(x, y) = 0.0;
                            }
                            else {
                                relation_graph(y, x) = 1.0;
                                relation_graph(x, y) = 1.0;
                            }
                        }
                        else
                            relation_graph(y, x) = 0.0;
                    }
                    for ( int x = 0; x < relation_graph.cols(); ++ x )
                        if ( relation_graph(y, x) > 0.5 )
                            ranked_n[y] ++;
                }
                vector<MatchRGBD> filtered_matches;
                int max_node_index = distance( ranked_n.begin(), max_element(ranked_n.begin(), ranked_n.end()) );
                for ( int x = 0; x < relation_graph.cols(); ++ x )
                    if ( relation_graph(max_node_index, x) > 0.5 )
                        filtered_matches.push_back( clustered_matches[j][x] );
                filtered_matches.push_back( matches[j][max_node_index] );

                // pose estimation
                if ( (int)filtered_matches.size() > g_pose_svd_minpts ) {
                    Pose pose;
                    if ( svd_pose_estimation_veri( filtered_matches, pose ) ) {
                        SP_Object obj( new Object );
                        obj->model_ = models[i];
                        obj->pose_  = pose;
                        obj->score_ = filtered_matches.size();
                        Matrix4f homo;
                        homo.topLeftCorner(3, 3)    = pose.r_;
                        homo.topRightCorner(3, 1)   = pose.t_.vector().transpose();
                        homo(3,0) = 0.0; homo(3,1) = 0.0; homo(3,2) = 0.0; homo(3,3) = 1.0;
                        obj->homo_  = homo;
                        objects.push_back( obj );
                    }
                }

                // release memory
                ranked_n.clear();
                filtered_matches.clear();
            }
        }
        clustered_matches.clear();
    }
    return objects;
}
*/

void display_pose(const cv::Mat &image, const list<SP_Object> &objects, const Vector4f & cp) {
    /** @todo: compute bounding_box */
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
    cv::namedWindow( "pose" );
    cv::imshow( "pose", rgb_image );
    cv::waitKey(10);
    rgb_image.release();
}


// functions for each step, RGB recogniser
/** ************************************************************************* */
// display rgb features
void display_features(const cv::Mat &image, const vector<DetectedFeatureRGB> &features) {
    cv::Mat rgb_image = image.clone();
    for ( size_t i = 0; i < features.size(); i ++ )
        cv::circle( rgb_image, cv::Point(features[i].img2d(0), features[i].img2d(1)), 3, cv::Scalar(128, 0, 255), 2 );
    cv::namedWindow( "feature" );
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
    cv::imshow( "matches", rgb_image );
    cv::waitKey( 10 );
    rgb_image.release();
}

// display clusters
void display_clusters(const cv::Mat &image, const vector<MatchRGB> &matches, vector<list<int> > &clusters) {
    cv::Mat rgb_image = image.clone();
    for ( size_t i = 0; i < clusters.size(); ++ i ) {
        cv::Scalar color( rand()%255, rand()%255, rand()%255 );
        for ( list<int>::iterator v = clusters[i].begin(); v != clusters[i].end(); ++ v ) {
            const MatchRGB & m = matches[*v];
            cv::circle( rgb_image, cv::Point( m.img2d(0), m.img2d(1) ), 3, color, 2 );
        }
    }
    cv::imshow( "clusters", rgb_image );
    cv::waitKey(10);
    rgb_image.release();
}











