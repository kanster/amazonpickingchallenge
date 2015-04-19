#include "include/helpfun/projection_filter.h"

void ProjectionFilter::init_params(int min_points, float feature_distance, float min_score, Vector4f camera_params) {
    min_points_ = min_points;
    feature_distance_ = feature_distance;
    min_score_ = min_score;
    camera_params_ = camera_params;
}

void ProjectionFilter::process( SP_Model & model, vector<MatchRGB> &matches, vector<list<int> > & clusters, list<SP_Object> &objects) {
    // using object index instead recognised object
    // map< pair<coordx, coordy>, pair<best score, object index> >
    map< pair<float, float>, pair<float, int> > best_points;
    // map< object inddex, cluster >
    map< int, list<int> > new_clusters;

    // go through each object
    Vector2f vec2d;
    int index = 0;
    foreach( object, objects ) {
        list<int> & new_cluster = new_clusters[index];
        float score = 0;
        int count = 0;
        for ( size_t i = 0; i < matches.size(); ++ i ) {
            vec2d = project( object->pose_, matches[i].mdl3d, camera_params_ );
            vec2d -= matches[i].img2d;
            float projection_error = vec2d(0)*vec2d(0)+vec2d(1)*vec2d(1);
            if ( projection_error < feature_distance_ ) {
                count ++;
                new_cluster.push_back( i );
                score += 1./(projection_error+1.);
            }
        }
        cout << "Consistent matches no: " << count << "\n";
        // assign score to 2d image point
        object->score_ = score;
        foreach ( match, new_cluster ) {
            pair<float, int> & point = best_points[make_pair( matches[match].img2d(0), matches[match].img2d(1) )];
            if ( point.first < score ) {
                point.first = score;
                point.second = index;
            }
        }
        index ++;
    }

    new_clusters.clear();
    for ( size_t i = 0; i < matches.size(); ++ i ) {
        pair<float, int> & point = best_points[make_pair( matches[i].img2d(0), matches[i].img2d(1) )];
        if ( point.second != NULL )
            new_clusters[point.second].push_back( i );
    }

    clusters.clear();
    index = 0;
    eforeach( object, object_it, objects ) {
        if ( (int)new_clusters[index].size() < min_points_ || object->score_ < min_score_ ) {
            object_it = objects.erase( object_it );
        }
        else {
            clusters.push_back( new_clusters[index] );
        }
        index ++;
    }
}

