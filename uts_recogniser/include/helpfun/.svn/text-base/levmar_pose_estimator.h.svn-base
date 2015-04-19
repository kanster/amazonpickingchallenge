#ifndef LEVMAR_POSE_ESTIMATION_H
#define LEVMAR_POSE_ESTIMATION_H

#include "include/helpfun/utils.h"
#include "include/levmar/lm.h"


// levmar for pose estimation
class LevmarPoseEstimator {
private:
    int max_ransac_tests_;          // e.g. 500
    int max_lm_tests_;              // e.g. 500
    int max_objects_per_cluster_;   // e.g. 4
    int n_pts_align_;               // e.g. 5
    int min_n_pts_object_;          // e.g. 8
    float error_threshold_;         // e.g. 5

    struct LmData{
        cv::Mat image;
        Vector2f coord2d;
        Vector3f coord3d;

        float score;
        float ratio;
    };


    // This function populates "samples" with nSamples references to object correspondences
    bool random_samples( vector<LmData *> & samples, const vector<LmData *> & cluster, unsigned int n_samples );

    static void lm_func_quat(float *lm_pose, float *pts2d, int n_pose, int n_pts2d, void *data);

    float optimize_camera( Pose & pose, const vector<LmData *> & samples, const int max_lm_tests );

    vector<pair<int, float> > test_allpoints( vector<LmData *> &consistent_corresp, const Pose &pose, const vector<LmData *> &test_points, const float error_threshold );

    void init_pose( Pose &pose, const vector<LmData *> &samples );

    bool ransac( Pose &pose, const vector<LmData *> & cluster );

    bool ratsac( Pose & pose,const vector<LmData *> & ori_cluster );

    void preprocess_all_matches( vector< LmData > &opt_data, const vector< MatchRGB > &matches ) ;

public:
    // init params
    void init_params( int max_ransac_tests, int max_lm_tests, int max_objects_per_cluster, int n_pts_align, int min_n_pts_object, float error_threshold, Vector4f levmar_camera_param );


    void process( const vector<MatchRGB> & matches, SP_Model model,const vector< list<int> > & clusters, list<SP_Object> & objects );
};


#endif // LEVMAR_POSE_ESTIMATION_H
