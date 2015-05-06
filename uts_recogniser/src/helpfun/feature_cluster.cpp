#include "include/helpfun/feature_cluster.h"

//! canopy constructor
template<typename T, int N> FeatureCluster::Canopy<T, N>::Canopy( Matrix<float, N, 1> & p, T & t, int id ) {
    center = p;
    bound_points.push_back( t );
    bound_points_size = 1;
    canopyid = id;
}

//! Feature cluster constructor
FeatureCluster::FeatureCluster(ClusterParam cp) {
    radius_ = (float)cp.radius;
    merge_ = (float)cp.merge;
    min_pts_ = cp.minpts;
    max_iterations_ = cp.maxiters;
}

FeatureCluster::FeatureCluster(float radius, float merge, int min_pts, int max_iterations) {
    radius_ = radius;
    merge_  = merge;
    min_pts_    = min_pts;
    max_iterations_ = max_iterations;
}

// mean shift method
template<typename T, int N> void FeatureCluster::mean_shift( vector< list<T> > & clusters, vector< pair<Eigen::Matrix<float, N, 1>, T> > & points ) {
    vector< FeatureCluster::Canopy<T, N> > can;
    can.reserve( points.size() );
    list<int> canopies_remaining;
    for ( size_t i = 0; i < points.size();  ++ i ) {
        can.push_back( Canopy<T, N>(points[i].first, points[i].second, i) );
        can.back().merges = & can.back();
        canopies_remaining.push_back( i );
    }

    bool done = false;
    for ( int it = 0; !done && it < max_iterations_; it ++ ) {
        done = true;
        foreach ( cId, canopies_remaining ) {
            can[cId].touch_pts_aggregate = can[cId].center*can[cId].bound_points_size;
            int touchpts_n = can[cId].bound_points_size;
            foreach ( othercId, canopies_remaining ) {
                if ( cId == othercId )
                    continue;
                float dist = (can[cId].center-can[othercId].center).norm();
                if ( dist < radius_ ) {
                    touchpts_n += can[othercId].bound_points_size;
                    can[cId].touch_pts_aggregate = can[cId].touch_pts_aggregate+can[othercId].center*can[othercId].bound_points_size;
                }
            }
        }
        foreach ( cId, canopies_remaining ) {
            foreach ( othercId, canopies_remaining ) {
                if ( cId == othercId )
                    break;
                float dist = (can[cId].center-can[othercId].center).norm();
                if ( dist < merge_ ) {
                    can[othercId].merges->merges = & can[cId];
                    can[othercId].merges = & can[cId];
                }
            }
        }
        eforeach ( cId, cId_it, canopies_remaining ) {
            if ( can[cId].merges != &can[cId] ) {
                can[cId].merges->center = can[cId].merges->center*can[cId].merges->bound_points.size()+can[cId].center*can[cId].bound_points_size;
                can[cId].merges->bound_points.splice( can[cId].merges->bound_points.end(), can[cId].bound_points);
                can[cId].merges->bound_points_size += can[cId].bound_points_size;
                can[cId].merges->center = can[cId].merges->center/can[cId].merges->bound_points_size;
                cId_it = canopies_remaining.erase( cId_it );
                done = false;
            }
        }
    }

    foreach ( cId, canopies_remaining ) {
        if ( can[cId].bound_points_size < min_pts_ )
            continue;
        clusters.resize( clusters.size() + 1 );
        clusters.back().splice( clusters.back().end(), can[cId].bound_points );
    }
}

// process
vector< list<int> > FeatureCluster::process(const vector<MatchRGB> & matches) {
    vector< list<int> > clusters;
    vector< pair<Eigen::Matrix<float, 2, 1>, int > > points_per_group;
    for ( size_t i = 0; i < matches.size(); ++ i ) {
        Eigen::Matrix<float, 2, 1> tmp;
        tmp << matches[i].img2d(0), matches[i].img2d(1);
        points_per_group.push_back( make_pair(tmp, i) );
    }
    mean_shift( clusters, points_per_group );
    points_per_group.clear();

    return clusters;
}


// process
vector< list<int> > FeatureCluster::process(const vector<MatchRGBD> & matches) {
    vector< list<int> > clusters;
    vector< pair<Eigen::Matrix<float, 3, 1>, int > > points_per_group;
    for ( size_t i = 0; i < matches.size(); ++ i ) {
        Eigen::Matrix<float, 3, 1> tmp;
        tmp << matches[i].obs3d(0), matches[i].obs3d(1), matches[i].obs3d(2);
        points_per_group.push_back( make_pair(tmp, i) );
    }
    mean_shift( clusters, points_per_group );
    points_per_group.clear();

    return clusters;
}




