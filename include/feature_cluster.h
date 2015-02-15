#ifndef FEATURE_CLUSTER_H
#define FEATURE_CLUSTER_H

#include "include/utils.h"

class FeatureCluster{
private:
    float   radius_;
    float   merge_;
    int     min_pts_;
    int     max_iterations_;

    template<typename T, int N> struct Canopy {
        Eigen::Matrix<float, N, 1> center;
        Eigen::Matrix<float, N, 1> touch_pts_aggregate;
        list<T> bound_points;
        int bound_points_size;
        int canopyid;
        Canopy<T, N> *merges;
        Canopy<T, N>(Matrix<float, N, 1> & p, T & t, int id);
    };

    template<typename T, int N> void mean_shift( vector< list<T> > & clusters, vector< pair<Eigen::Matrix<float, N, 1>, T> > & points);

public:
    // constructor with parametrisation
    FeatureCluster( float radius, float merge, int min_pts, int max_iterations );

    vector< list<int> > process( const vector<MatchRGB> & matches );
};

#endif
