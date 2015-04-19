#ifndef FEATURE_MATCHER_H
#define FEATURE_MATCHER_H

#include "include/helpfun/utils.h"

template<typename T> void normaliser( vector<T> & vec );


class FeatureMatcher{
private:
    float quality_;
    float ratio_;
    int descriptor_size_;
    string descriptor_type_;

    int model_size_;
    vector<int> corresp_model_idx_;
    vector<Vector3f *> corresp_model_feat_;
    vector<int> corresp_feat_idx_;


    ANNkd_tree * kdtree_;

public:
    // constructor with parametrisation
    FeatureMatcher( float quality, float ratio, int descriptor_size, string descriptor_type );

    // load models
    void load_models( vector<SP_Model> & models );

    // process matching
    vector<MatchRGB> process( vector<DetectedFeatureRGB> &features, int object_in_bin );

    // process matching rgbd
    vector<MatchRGBD> process(vector<DetectedFeatureRGBD> &features, int object_in_bin);
};

#endif
