#ifndef FILTER_PROJECTION_H
#define FILTER_PROJECTION_H

#include "include/helpfun/utils.h"

class ProjectionFilter{
private:
    int min_points_;
    float feature_distance_;
    float min_score_;
    Vector4f camera_params_;

public:
    //! constructor
    ProjectionFilter( ProjParam pp );

    //! init parameters
    void init_params( int min_points, float feature_distance, float min_score, Vector4f camera_params );
    void process( SP_Model & model, vector<MatchRGB> &matches, vector<list<int> > & clusters, list<SP_Object> &objects );
};

#endif // FILTER_PROJECTION_H
