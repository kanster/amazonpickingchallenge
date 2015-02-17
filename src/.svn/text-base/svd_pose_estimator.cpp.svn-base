#include "include/svd_pose_estimator.h"


// constructor with parametrisation
SVDPoseEstimator::SVDPoseEstimator(float error, int minpts) {
    error_ = error;
    minpts_ = minpts;
}

// svd pose estimation solver
pair<float, int> SVDPoseEstimator::solver(vector<MatchRGBD> & matches, Pose & pose) {
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

// verification
bool SVDPoseEstimator::verifier(vector<MatchRGBD> &matches, Pose &pose) {
    Pose init_pose;
    pair<float, int> max_error = solver( matches, init_pose );
//    cout << "max error: " << max_error.first << ", " << max_error.second << "\n";
    int iter = 0;
    while ( (float)max_error.first > error_ ) {
        matches.erase( matches.begin() + max_error.second );
        max_error = solver( matches, init_pose );
        iter ++;
//        cout << iter << "th iteration: max error [" << max_error.first << ", " << max_error.second << "]\n";
    }
    if ( max_error.first < error_ ) {
//        cout << "FINAL -> max error: " << max_error.first << ", " << max_error.second << "\n";
        pose = init_pose;
        return true;
    }
    else
        return false;
}


// process
list<SP_Object> SVDPoseEstimator::process( const vector<MatchRGBD> & matches, SP_Model model, const vector< list<int> > & clusters ) {
    list<SP_Object> objects;
    vector< vector<MatchRGBD> > clustered_matches( clusters.size() );
    for ( int i = 0; i < (int)clusters.size(); ++ i ) {
        foreach( it, clusters[i] ) {
            clustered_matches[i].push_back( matches[it] );
        }
    }


    for ( size_t i = 0; i < clustered_matches.size(); ++ i ) {
        if ( (int)clustered_matches[i].size() > minpts_ ) {
            // graph filter
            MatrixXi relation_graph( clustered_matches[i].size(), clustered_matches[i].size() );
            vector<int> ranked_n( relation_graph.rows(), 0 );
            for ( int y = 0; y < relation_graph.rows(); ++ y ) {
                for ( int x = 0; x < relation_graph.cols(); ++ x ) {
                    if ( x != y ) {
                        Vector3f obs_diff = clustered_matches[i][y].obs3d - clustered_matches[i][x].obs3d;
                        Vector3f mdl_diff = clustered_matches[i][y].mdl3d - clustered_matches[i][x].mdl3d;
                        float diff = obs_diff.norm() - mdl_diff.norm();
                        if ( abs(diff) > error_ ) {
                            relation_graph(y, x) = 0;
                            relation_graph(x, y) = 0;
                        }
                        else {
                            relation_graph(y, x) = 1;
                            relation_graph(x, y) = 1;
                        }
                    }
                    else
                        relation_graph(y, x) = 0;
                }
                for ( int x = 0; x < relation_graph.cols(); ++ x )
                    if ( relation_graph(y, x) > 0.5 )
                        ranked_n[y] ++;
            }

            // filter consistent matches
            vector<MatchRGBD> filtered_matches;
            int max_node_index = distance( ranked_n.begin(), max_element(ranked_n.begin(), ranked_n.end()) );
            for ( int x = 0; x < relation_graph.cols(); ++ x )
                if ( relation_graph(max_node_index, x) > 0.5 )
                    filtered_matches.push_back( clustered_matches[i][x] );
            filtered_matches.push_back( clustered_matches[i][max_node_index] );


            // pose estimation core
            if ( (int)filtered_matches.size() > minpts_ ) {
                Pose pose;
                if ( verifier( filtered_matches, pose ) ) {
                    SP_Object obj( new Object );
                    obj->pose_  = pose;
                    obj->model_ = model;
                    obj->score_ = filtered_matches.size();
                    obj->homo_ = Matrix4f::Identity();
                    obj->homo_.topLeftCorner(3, 3)    = pose.r_;
                    obj->homo_.topRightCorner(3, 1)   = pose.t_.vector();
                    objects.push_back( obj );
                }
            }
            ranked_n.clear();
            filtered_matches.clear();
        }
    }

    /** @todo combine objects with similiar poses */


    return objects;
}
