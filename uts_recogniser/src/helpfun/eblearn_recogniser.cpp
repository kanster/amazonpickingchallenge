/** eblearn recogniser */

#include "include/helpfun/eblearn_recogniser.h"
#include "include/helpfun/utils.h"

template <typename T> void EBRecogniser::init_detector(ebl::detector<T> & detect, ebl::configuration & conf, std::string & outdir, bool silent) {
    // multi-scaling parameters
    double maxs = conf.try_get_double("max_scale", 2.0);
    double mins = conf.try_get_double("min_scale", 1.0);
    ebl::t_scaling scaling_type = (ebl::t_scaling)conf.try_get_uint("scaling_type", ebl::SCALES_STEP);
    double scaling = conf.try_get_double("scaling", 1.4);
    std::vector<ebl::midxdim> scales;
    switch (scaling_type) {
    case ebl::MANUAL:
        if (!conf.exists("scales"))
            eblerror("expected \"scales\" variable to be defined in manual mode");
        scales = ebl::string_to_midxdimvector(conf.get_cstring("scales"));
        detect.set_resolutions(scales);
        break ;
    case ebl::ORIGINAL:
        detect.set_scaling_original();
        break;
    case ebl::SCALES_STEP:
        detect.set_resolutions(scaling, maxs, mins);
        break;
    case ebl::SCALES_STEP_UP:
        detect.set_resolutions(scaling, maxs, mins);
        detect.set_scaling_type(scaling_type);
        break;
    default:
        detect.set_scaling_type(scaling_type);
    }


    // remove pads from target scales if requested
    if (conf.exists_true("scaling_remove_pad"))
        detect.set_scaling_rpad(true);
    // optimize memory usage by using only 2 buffers for entire flow
    ebl::state<T> input(1, 1, 1), output(1, 1, 1);
    if (!conf.exists_false("mem_optimization"))
        detect.set_mem_optimization(input, output, true);
    // TODO: always keep inputs, otherwise detection doesnt work. fix this.
    // 				   conf.exists_true("save_detections") ||
    // 				   (display && !mindisplay));
    // zero padding
    float hzpad = conf.try_get_float("hzpad", 0);
    float wzpad = conf.try_get_float("wzpad", 0);
    detect.set_zpads(hzpad, wzpad);


    if (conf.exists("input_min")) // limit inputs size
        detect.set_min_resolution(conf.get_uint("input_min"));
    if (conf.exists("input_max")) // limit inputs size
        detect.set_max_resolution(conf.get_uint("input_max"));
    if (silent)
        detect.set_silent();
    if (conf.exists_bool("save_detections")) {
        std::string detdir = outdir;
        detdir += "detections";
        uint nsave = conf.try_get_uint("save_max_per_frame", 0);
        bool diverse = conf.exists_true("save_diverse");
        detdir = detect.set_save(detdir, nsave, diverse);
    }
    else {
    }
    detect.set_scaler_mode(conf.exists_true("scaler_mode"));
    if (conf.exists("bbox_decision"))
        detect.set_bbox_decision(conf.get_uint("bbox_decision"));
    if (conf.exists("bbox_scalings")) {
        ebl::mfidxdim scalings = ebl::string_to_fidxdimvector(conf.get_cstring("bbox_scalings"));
        detect.set_bbox_scalings(scalings);
    }


    // nms configuration //////////////////////////////////////////////////////
    ebl::t_nms nms_type = (ebl::t_nms)conf.try_get_uint("nms", 0);
    float pre_threshold = conf.try_get_float("pre_threshold", 0.0);
    float post_threshold = conf.try_get_float("post_threshold", 0.0);
    float pre_hfact = conf.try_get_float("pre_hfact", 1.0);
    float pre_wfact = conf.try_get_float("pre_wfact", 1.0);
    float post_hfact = conf.try_get_float("post_hfact", 1.0);
    float post_wfact = conf.try_get_float("post_wfact", 1.0);
    float woverh = conf.try_get_float("woverh", 1.0);
    float max_overlap = conf.try_get_float("max_overlap", 0.0);
    float max_hcenter_dist = conf.try_get_float("max_hcenter_dist", 0.0);
    float max_wcenter_dist = conf.try_get_float("max_wcenter_dist", 0.0);
    float vote_max_overlap = conf.try_get_float("vote_max_overlap", 0.0);
    float vote_mhd = conf.try_get_float("vote_max_hcenter_dist", 0.0);
    float vote_mwd = conf.try_get_float("vote_max_wcenter_dist", 0.0);
    detect.set_nms(nms_type, pre_threshold, post_threshold, pre_hfact,
                   pre_wfact, post_hfact, post_wfact, woverh, max_overlap,
                   max_hcenter_dist, max_wcenter_dist, vote_max_overlap,
                   vote_mhd, vote_mwd);
    if (conf.exists("raw_thresholds")) {
        std::string srt = conf.get_string("raw_thresholds");
        std::vector<float> rt = ebl::string_to_floatvector(srt.c_str());
        detect.set_raw_thresholds(rt);
    }
    if (conf.exists("outputs_threshold"))
        detect.set_outputs_threshold(conf.get_double("outputs_threshold"), conf.try_get_double("outputs_threshold_val", -1));
    ///////////////////////////////////////////////////////////////////////////
    if (conf.exists("netdims")) {
        ebl::idxdim d = ebl::string_to_idxdim(conf.get_string("netdims"));
        detect.set_netdim(d);
    }
    if (conf.exists("smoothing")) {
        ebl::idxdim ker;
        if (conf.exists("smoothing_kernel"))
            ker = ebl::string_to_idxdim(conf.get_string("smoothing_kernel"));
        detect.set_smoothing(conf.get_uint("smoothing"), conf.try_get_double("smoothing_sigma", 1), &ker, conf.try_get_double("smoothing_sigma_scale", 1));
    }
    if (conf.exists("background_name"))
        detect.set_bgclass(conf.get_cstring("background_name"));
    if (conf.exists_true("bbox_ignore_outsiders"))
        detect.set_ignore_outsiders();
    if (conf.exists("corners_inference"))
        detect.set_corners_inference(conf.get_uint("corners_inference"));
    if (conf.exists("input_gain"))
        detect.set_input_gain(conf.get_double("input_gain"));
    if (conf.exists_true("dump_outputs")) {
        std::string fname;
        fname << outdir << "/dump/detect_out";
        detect.set_outputs_dumping(fname.c_str());
    }
}


void EBRecogniser::set_frame() {
    if ( !rgb_image_.empty() ) {
        frame_ = ebl::idx<float>( rgb_image_.rows, rgb_image_.cols, 3 );
        int b, g, r;
        for ( int y = 0; y < rgb_image_.rows; ++ y ) {
            for ( int x = 0; x < rgb_image_.cols; ++ x ) {
                b = rgb_image_.at<cv::Vec3b>(y,x)[0];
                g = rgb_image_.at<cv::Vec3b>(y,x)[1];
                r = rgb_image_.at<cv::Vec3b>(y,x)[2];
                frame_.set( static_cast<float>(r), y, x, 0 );
                frame_.set( static_cast<float>(g), y, x, 1 );
                frame_.set( static_cast<float>(b), y, x, 2 );
            }
        }
    }
}

EBRecogniser::EBRecogniser(){}


void EBRecogniser::load_sensor_data(cv::Mat rgb_image, cv::Mat depth_image) {
    rgb_image_ = rgb_image;
    depth_image_ = depth_image;
    set_frame();
}


void EBRecogniser::load_sensor_data(cv::Mat rgb_image) {
    rgb_image_ = rgb_image;
    set_frame();
}


void EBRecogniser::load_info(cv::Mat empty_image, cv::Mat mask_image, cv::Mat empty_depth_image) {
    empty_image_ = empty_image;
    mask_image_  = mask_image;
    empty_depth_image_ = empty_depth_image;
}


void EBRecogniser::load_info(cv::Mat empty_image, cv::Mat mask_image) {
    empty_image_ = empty_image;
    mask_image_  = mask_image;
}


void EBRecogniser::set_env_configuration(std::string target_item, std::vector<std::string> items) {
    target_item_ = target_item;
    for ( int i = 0; i < (int)items.size(); ++ i )
        items_.push_back( items[i] );
}


void EBRecogniser::set_conf_dir(std::string conf_dir) {
    conf_dir_ = conf_dir;
}


std::vector< std::pair<std::string, std::vector<cv::Point> > > EBRecogniser::process( bool use_rgb ) {
    std::vector< std::pair<std::string, std::vector<cv::Point> > > recog_results;

    std::vector< std::pair<std::string, int> > objects_n = duplicated_bin_contents( items_ );

    // init detector for each item
    for ( int i = 0; i < (int)objects_n.size(); ++ i ) {
        std::string item_name = objects_n[i].first;
        std::string conf_path = conf_dir_+item_name+".conf";
        ebl::configuration conf( conf_path, true, true, false );
        ebl::parameter<float> theparam;
        theparam.set_forward_only();
        ebl::idx<ebl::ubyte> classes(1,1);
        ebl::load_matrix<ebl::ubyte>( classes, conf.get_cstring( "classes" ) );
        std::vector<std::string> sclasses = ebl::ubyteidx_to_stringvector(classes);
        ebl::answer_module<float> *ans = ebl::create_answer<float, float, float>(conf, classes.dim(0));
        uint noutputs = ans->get_nfeatures();
        ebl::intg thick = -1;
        ebl::module_1_1<float> * net = ebl::create_network<float>(theparam, conf, thick, noutputs, "arch", 0);

        // loading weights
        if ( conf.exists("weights") ) {
            std::vector<std::string> w = ebl::string_to_stringvector(conf.get_string("weights"));
            theparam.load_x(w);
            if ( conf.exists("weights_permutation") ) {
                std::string sblocks = conf.get_string("weights_blocks");
                std::string spermut = conf.get_string("weights_permutation");
                std::vector<ebl::intg> blocks = ebl::string_to_intgvector(sblocks.c_str());
                std::vector<uint> permut = ebl::string_to_uintvector(spermut.c_str());
                theparam.permute_x(blocks, permut);
            }
        }
        else {
            if (conf.exists_true("manual_load")) {
                ebl::manually_load_network(*((ebl::layers<float>*)net), conf);
            }
            else { // random weights
                int seed = ebl::dynamic_init_drand();
                ebl::forget_param_linear fgp(1, 0.5, seed);
                net->forget(fgp);
            }
        }

        pdetect_ = new ebl::detector<float>(*net, sclasses, ans);
        std::string outdir = "./";
        bool silent = false;
        init_detector( *pdetect_, conf, outdir, silent );

        ebl::bboxes & bbs = pdetect_->fprop(frame_);
        int ret_nbb = bbs.size() > objects_n[i].second? objects_n[i].second: bbs.size(); // return n of bbs
        for ( int j = 0; j < ret_nbb; ++ j ) {
            ebl::bbox bb = bbs[j];
            std::vector<cv::Point> pts;
            pts.push_back( cv::Point(bb.w0, bb.h0) );
            pts.push_back( cv::Point(bb.w0+bb.width, bb.h0) );
            pts.push_back( cv::Point(bb.w0+bb.width, bb.h0+bb.height) );
            pts.push_back( cv::Point(bb.w0, bb.h0+bb.height) );
            recog_results.push_back( std::make_pair( item_name, pts ) );
        }
        if (ans) delete ans;
        if (net) delete net;
    }

    return recog_results;
}































