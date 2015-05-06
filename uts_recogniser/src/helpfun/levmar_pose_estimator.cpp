#include "include/helpfun/levmar_pose_estimator.h"
#include <deque>
#include <boost/bind.hpp>

Vector4f g_levmar_camera_param = Vector4f::Zero();

//! constructor
LevmarPoseEstimator::LevmarPoseEstimator(LevmarParam lmp) {
    init_params( lmp.max_ransac, lmp.max_lm, lmp.max_objs_per_cluster, lmp.n_pts_align, lmp.min_pts_per_obj, (float)lmp.error_threshold, lmp.camera_param );
}

//! init
void LevmarPoseEstimator::init_params(int max_ransac_tests, int max_lm_tests, int max_objects_per_cluster, int n_pts_align, int min_n_pts_object, float error_threshold, Vector4f levmar_camera_param) {
    max_ransac_tests_   = max_ransac_tests;
    max_lm_tests_       = max_lm_tests;
    max_objects_per_cluster_ = max_objects_per_cluster;
    n_pts_align_         = n_pts_align;
    min_n_pts_object_    = min_n_pts_object;
    error_threshold_     = error_threshold;
    g_levmar_camera_param(0) = levmar_camera_param(0);
    g_levmar_camera_param(1) = levmar_camera_param(1);
    g_levmar_camera_param(2) = levmar_camera_param(2);
    g_levmar_camera_param(3) = levmar_camera_param(3);
}

// generate non-repeated randome samples from clustered matches
bool LevmarPoseEstimator::random_samples(vector<LmData *> &samples, const vector<LmData *> &cluster, unsigned int n_samples) {
    map< pair<float, float>, int > used;
    deque< pair<float, LmData *> > random_samples;
    foreach( match, cluster )
        random_samples.push_back( make_pair((float)rand(), match) );
    sort( random_samples.begin(), random_samples.end() );

    while ( used.size() < n_samples && !random_samples.empty() ) {
        if ( !used[ make_pair( random_samples.front().second->coord2d(0), random_samples.front().second->coord2d(1) ) ] ++ )
            samples.push_back( random_samples.front().second );
        random_samples.pop_front();
    }
    return used.size() == n_samples;
}

// levmar functions for optimisation
void LevmarPoseEstimator::lm_func_quat(float *lm_pose, float *pts2d, int n_pose, int n_pts2d, void *data) {
    vector<LmData *> & lm_data = *(vector<LmData *> *)data;
    Pose pose;
    pose.q_ = Quaternionf(lm_pose[3], lm_pose[0], lm_pose[1], lm_pose[2]);
    pose.q_.normalize();
    pose.t_ = Translation3f(lm_pose[4], lm_pose[5], lm_pose[6]);
    pose.r_ = pose.q_.toRotationMatrix();

    TransformMatrix tm;
    tm.init( pose );

    for ( int i = 0; i < n_pts2d/2; ++ i ) {
        Vector3f p3d = tm.transform( lm_data[i]->coord3d );
        Vector2f p2d;
        p2d(0) = p3d(0)/p3d(2)*g_levmar_camera_param(0) + g_levmar_camera_param(2);
        p2d(1) = p3d(1)/p3d(2)*g_levmar_camera_param(1) + g_levmar_camera_param(3);
        if ( p3d(2) < 0 ) {
            pts2d[2*i]      = -p3d(2)+10;
            pts2d[2*i+1]    = -p3d(2)+10;
        }
        else {
            pts2d[2*i]      = p2d(0)-lm_data[i]->coord2d(0);
            pts2d[2*i+1]    = p2d(1)-lm_data[i]->coord2d(1);
            pts2d[2*i]  *= pts2d[2*i];
            pts2d[2*i+1]*= pts2d[2*i+1];
        }
    }
}

float LevmarPoseEstimator::optimize_camera(Pose &pose, const vector<LmData *> &samples, const int max_lm_tests) {
    float cam_pose_lm[7] = {pose.q_.x(), pose.q_.y(), pose.q_.z(), pose.q_.w(), pose.t_.x(), pose.t_.y(), pose.t_.z()};
    vector<float> pts2d( samples.size()*2, 0 );
    float info[LM_INFO_SZ];
    int ret_value = slevmar_dif( lm_func_quat, cam_pose_lm, &pts2d[0], 7, samples.size()*2, max_lm_tests, NULL, info, NULL, NULL, (void *)&samples );
    if ( ret_value < 0 ) {
        return ret_value;
    }
    pose.q_ = Quaternionf(cam_pose_lm[3], cam_pose_lm[0], cam_pose_lm[1], cam_pose_lm[2]);
    pose.q_.normalize();

    pose.t_ = Translation3f(cam_pose_lm[4], cam_pose_lm[5], cam_pose_lm[6]);
    pose.r_ = pose.q_.toRotationMatrix();
    return info[1];
}

vector<pair<int, float> > LevmarPoseEstimator::test_allpoints(vector<LmData *> &consistent_corresp, const Pose &pose, const vector<LmData *> &test_points, const float error_threshold) {
    vector<pair<int, float> > errors;
    consistent_corresp.clear();
    Vector2f vec2d;
    int idx = 0;
    foreach( corresp, test_points ) {
        vec2d = project( pose, corresp->coord3d, g_levmar_camera_param );
        vec2d -= corresp->coord2d;
        float projection_error = vec2d.norm();
        errors.push_back( make_pair(idx, projection_error) );
        if ( projection_error < error_threshold )
            consistent_corresp.push_back( corresp );
        idx ++;
    }
    return errors;
}

void LevmarPoseEstimator::init_pose(Pose &pose, const vector<LmData *> &samples) {
//    pose.q_ = Quaternionf( 0.577555, 0.358377, -0.24973, -0.68966 );
    pose.q_ = Quaternionf((rand()&255)/256., (rand()&255)/256., (rand()&255)/256., (rand()&255)/256.);
    pose.t_ = Translation3f(0., 0., 70.);
//    pose.t_ = Translation3f( 0., 0., 1.0);
    pose.r_ = pose.q_.toRotationMatrix();
}

bool LevmarPoseEstimator::ransac(Pose &pose, const vector<LmData *> &cluster) {
    vector<LmData *> samples;
    for ( int nIters = 0; nIters < max_ransac_tests_; ++ nIters) {
        samples.clear();
        if( !random_samples(samples, cluster, n_pts_align_) ) {
            return false;
        }

        init_pose( pose, samples );
        int lm_iterations = optimize_camera( pose, samples, max_lm_tests_ );
        if( lm_iterations == -1 ) {
            continue;
        }

        vector<LmData *> consistent;
        test_allpoints( consistent, pose, cluster, error_threshold_ );
        if ( (int)consistent.size() > min_n_pts_object_ ) {
            optimize_camera( pose, consistent, max_lm_tests_ );
            return true;
        }
    }
    return false;
}


bool LevmarPoseEstimator::ratsac(Pose &pose, const vector<LmData *> &ori_cluster) {
    vector<LmData *> cluster;
    for ( int i = 0; i < (int)ori_cluster.size(); ++ i )
        cluster.push_back( ori_cluster[i] );
    // sort clusters based on the ratio
    int j;
    LmData * tmp_lmdata;
    for ( int i = 0; i < (int)cluster.size(); ++ i ) {
        j = i;
        while ( j > 0 && cluster[j]->ratio < cluster[j-1]->ratio ) {
            tmp_lmdata = cluster[j];
            cluster[j] = cluster[j-1];
            cluster[j-1] = tmp_lmdata;
            j --;
        }
    }
    vector<LmData *> samples;
    while ( cluster.size() >= n_pts_align_ ) {
        // generate samples
        samples.clear();
        int n_cluster = cluster.size();
        for ( int i = n_cluster-1; i > n_cluster-n_pts_align_-1; -- i )
            samples.push_back( cluster[i] );

        init_pose( pose, samples );
        int lm_iterations = optimize_camera( pose, samples, max_lm_tests_ );
        if( lm_iterations == -1 ) {
            continue;
        }
        vector<LmData *> consistent;
        vector<pair<int, float> > errors = test_allpoints( consistent, pose, cluster, error_threshold_ );
        // sort errors
        sort(errors.begin(), errors.end(),  boost::bind(&std::pair<int, float>::second, _1) < boost::bind(&std::pair<int, float>::second, _2));

        if ( (int)consistent.size() > cluster.size()*0.5 ) {
//            for ( int i = 0; i < (int)errors.size(); ++ i )
//                cout << errors[i].first << ", " << errors[i].second << "\n";
            vector<LmData *> pose_matches;
            int n_pose_matches = min_n_pts_object_ > consistent.size()? consistent.size(): min_n_pts_object_;
//            cout << "\n=========\n";
            int idx = 0;
            while ( errors[idx].second < error_threshold_ ) {
                pose_matches.push_back( cluster[errors[idx].first] );
//                cout << pose_matches.back()->score << ", " << pose_matches.back()->ratio << " with error: " << errors[idx].second << endl;
                idx ++;
            }
            /*
            for ( int i = 0; i < n_pose_matches; ++ i ) {
                pose_matches.push_back( cluster[errors[i].first] );
                cout << pose_matches.back()->score << ", " << pose_matches.back()->ratio << " with error: " << errors[i].second << endl;
            }
            */
//            cout << "=========\n";

            optimize_camera( pose, pose_matches, max_lm_tests_ );
            return true;
        }

        cluster.pop_back();
    }
//    getchar();
    return false;

}


void LevmarPoseEstimator::preprocess_all_matches(vector<LmData> &opt_data, const vector<MatchRGB> &matches ) {
    opt_data.resize( matches.size() );
    for ( size_t i = 0; i < matches.size(); ++ i ) {
        opt_data[i].coord2d = matches[i].img2d;
        opt_data[i].coord3d = matches[i].mdl3d;
        opt_data[i].score = matches[i].score;
        opt_data[i].ratio = matches[i].ratio;
    }
}

void LevmarPoseEstimator::process(const vector<MatchRGB> &matches, SP_Model model, const vector<list<int> > &clusters,  list<SP_Object> & objects) {
    vector<LmData> lm_data;
    preprocess_all_matches( lm_data, matches );


    vector<int> tasks;
    tasks.reserve(1000);
    for ( int i_cluster = 0; i_cluster < (int)clusters.size(); ++ i_cluster )
        for ( int i_obj = 0; i_obj < max_objects_per_cluster_; ++ i_obj )
            tasks.push_back( i_cluster );
    #pragma omp parallel for
    for ( int task = 0; task < (int)tasks.size(); ++ task ) {
        vector<LmData *> cl;
        foreach( point, clusters[tasks[task]] )
            cl.push_back( &lm_data[point] );
        Pose pose;
//        bool found = ratsac( pose, cl );
        bool found = ransac( pose, cl );
        if ( found > 0 )
        #pragma omp critical(POSE)
        {
            SP_Object obj(new Object);
            obj->pose_ = pose;
            obj->model_= model;
            obj->homo_ = Matrix4f::Identity();
            obj->homo_.topLeftCorner(3,3) = pose.r_;
            obj->homo_.topRightCorner(3,1) = pose.t_.vector();
            objects.push_back( obj );
        }
    }
}































