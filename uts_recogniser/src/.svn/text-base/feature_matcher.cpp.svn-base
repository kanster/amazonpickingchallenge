#include "include/feature_matcher.h"


template<typename T> void normaliser( vector<T> & vec ) {
    float norm = 0.0;
    for ( size_t i = 0; i < vec.size(); ++ i )
        norm += vec[i]*vec[i];
    norm = sqrt(norm);
    for ( size_t i = 0; i < vec.size(); ++ i )
        vec[i] /= norm;
}

// constructor with parameterisation
FeatureMatcher::FeatureMatcher(float quality, float ratio, int descriptor_size, string descriptor_type) {
    quality_    = quality;
    ratio_      = ratio;
    descriptor_size_    = descriptor_size;
    descriptor_type_    = descriptor_type;
}


// load models and build kdtree
void FeatureMatcher::load_models( vector<SP_Model> &models) {
    unsigned int n_model_features = 0;
    foreach( model, models )
        n_model_features += model->features_.size();

    model_size_ = models.size();
    corresp_model_idx_.resize( n_model_features );
    corresp_model_feat_.resize( n_model_features );
    corresp_feat_idx_.resize( n_model_features );

    ANNpointArray model_descrips = annAllocPts( n_model_features, 128 );
    int idx = 0;
    for ( size_t i_model = 0; i_model < models.size(); ++ i_model ) {
        const SP_Model & model = models[i_model];
        for ( size_t i_feat = 0; i_feat < model->features_.size(); ++ i_feat ) {
            corresp_model_idx_[idx] = i_model;
            corresp_model_feat_[idx] = &model->features_[i_feat].coord3d;
            corresp_feat_idx_[idx] = i_feat;
            normaliser(model->features_[i_feat].descrip);
            for ( int i_desc = 0; i_desc < descriptor_size_; ++ i_desc )
                model_descrips[idx][i_desc] = model->features_[i_feat].descrip[i_desc];
            idx ++;
        }
    }

    kdtree_ = new ANNkd_tree( model_descrips, n_model_features, 128 );
}

// process rgb matches
vector<MatchRGB> FeatureMatcher::process(vector<DetectedFeatureRGB> &features, int object_in_bin) {
    vector<MatchRGB> matches;
    ANNpoint pt     = annAllocPt( descriptor_size_ );
    ANNidxArray nx  = new ANNidx[2];
    ANNdistArray ds = new ANNdist[2];
    for ( size_t i = 0; i < features.size(); ++ i ) {
        DetectedFeatureRGB & f = features[i];
        normaliser( f.descrip );
        for ( int j = 0; j < descriptor_size_; ++ j )
            pt[j] = f.descrip[j];
        #pragma omp critical(ANN)
        kdtree_->annkSearch( pt, 2, nx, ds, quality_ );
        if ( ds[0]/ds[1] < ratio_ ) {
            int model_idx = corresp_model_idx_[nx[0]];
            if ( matches.capacity() < 1000 )
                matches.reserve(1000);

            if ( model_idx == object_in_bin ) {
                matches.resize( matches.size() + 1 );
                MatchRGB &m = matches.back();
                m.img2d     = features[i].img2d;
                m.mdl3d     = *corresp_model_feat_[nx[0]];
            }
        }
    }

    delete kdtree_;
    corresp_model_idx_.clear();
    corresp_model_feat_.clear();

    return matches;
}

// process rgbd matches
vector<MatchRGBD> FeatureMatcher::process(vector<DetectedFeatureRGBD> &features, int object_in_bin) {
    vector<MatchRGBD> matches;
    ANNpoint pt     = annAllocPt( descriptor_size_ );
    ANNidxArray nx  = new ANNidx[2];
    ANNdistArray ds = new ANNdist[2];
    for ( size_t i = 0; i < features.size(); ++ i ) {
        DetectedFeatureRGBD & f = features[i];
        normaliser( f.descrip );
        for ( int j = 0; j < descriptor_size_; ++ j )
            pt[j] = f.descrip[j];
        #pragma omp critical(ANN)
        kdtree_->annkSearch( pt, 2, nx, ds, quality_ );
        if ( ds[0]/ds[1] < ratio_ ) {
            int model_idx = corresp_model_idx_[nx[0]];
            if ( matches.capacity() < 1000 )
                matches.reserve(1000);

            if ( model_idx == object_in_bin ) {
                matches.resize( matches.size() + 1 );
                MatchRGBD &m = matches.back();
                m.img2d     = features[i].img2d;
                m.obs3d     = features[i].obs3d;
                m.mdl3d     = *corresp_model_feat_[nx[0]];
            }
        }
    }

    delete kdtree_;
    corresp_model_idx_.clear();
    corresp_model_feat_.clear();

    return matches;
}











































