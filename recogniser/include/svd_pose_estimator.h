#ifndef SVD_POSE_ESTIMATOR_H
#define SVD_POSE_ESTIMATOR_H

#include "include/utils.h"

class SVDPoseEstimator{
private:
    float error_;
    int minpts_;
    // svd main solver
    pair<float, int> solver( vector<MatchRGBD> & matches, Pose & pose );
    // pose verification
    bool verifier( vector<MatchRGBD> & matches, Pose & pose );


public:
    SVDPoseEstimator( float error, int minpts );

    list<SP_Object> process( const vector<MatchRGBD> & matches, SP_Model model, const vector< list<int> > & clusters );
};

#endif
