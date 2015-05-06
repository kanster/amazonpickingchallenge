#include "include/helpfun/projection_filter.h"


//! constructor
ProjectionFilter::ProjectionFilter(ProjParam pp) {
    init_params( pp.min_pts, (float)pp.feat_distance,(float) pp.min_score, pp.camera_param );
}

//! init parameters
void ProjectionFilter::init_params(int min_points, float feature_distance, float min_score, Vector4f camera_params) {
    min_points_ = min_points;
    feature_distance_ = feature_distance;
    min_score_ = min_score;
    camera_params_ = camera_params;
}

void ProjectionFilter::process( SP_Model & model, vector<MatchRGB> &matches, vector<list<int> > & clusters, list<SP_Object> &objects) {
    // using object index instead recognised object
    // map< pair<coordx, coordy>, pair<best score, object index> >
    map< pair<float, float>, pair<float, Object*> > best_points;
    // map< object inddex, cluster >
    map< Object *, list<int> > new_clusters;

    // go through each object
    Vector2f vec2d;
    foreach ( object, objects ) {
        list<int> & new_cluster = new_clusters[object.get()];
        float score = 0.;
        for ( int mi = 0; mi < (int)matches.size(); ++ mi ) {
            vec2d = project( object->pose_, matches[mi].mdl3d, camera_params_ );
            vec2d -= matches[mi].img2d;
            float projection_error = vec2d(0)*vec2d(0)+vec2d(1)*vec2d(1);
            if ( projection_error < feature_distance_ ) {
                new_cluster.push_back( mi );
                score += 1./(projection_error+1.0);
            }
        }
        object->score_ = score;
        foreach ( match, new_cluster ) {
            pair<float, Object* > & point = best_points[make_pair(matches[match].img2d(0), matches[match].img2d(1))];
            if ( point.first < score ) {
                point.first = score;
                point.second = object.get();
            }
        }
    }

    new_clusters.clear();
    for ( int mi = 0; mi < matches.size(); ++ mi ) {
        pair<float, Object *> &point = best_points[make_pair( matches[mi].img2d(0), matches[mi].img2d(1))];
        if ( point.second != NULL ) {
            new_clusters[point.second].push_back(mi);
        }
    }

    clusters.clear();
    eforeach( object, object_it, objects ) {
        if ( (int)new_clusters[object.get()].size() < min_points_ || object->score_ < min_score_ ) {
            object_it = objects.erase( object_it );
        }
        else {
            clusters.push_back( new_clusters[object.get()] );
        }
    }

    /*
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
    cout << "best points size: " << best_points.size() << endl;

    new_clusters.clear();
    for ( size_t i = 0; i < matches.size(); ++ i ) {
        pair<float, int> & point = best_points[make_pair( matches[i].img2d(0), matches[i].img2d(1) )];
        if ( point.second != NULL )
            new_clusters[point.second].push_back( i );
    }
    cout << "new clusters size: " << new_clusters.size() << endl;

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

    for ( int i = 0; i < (int)clusters.size(); ++ i ) {
        cout << "cluster " << i << " of size " << clusters[i].size() << "\n";
    }
    */
}

