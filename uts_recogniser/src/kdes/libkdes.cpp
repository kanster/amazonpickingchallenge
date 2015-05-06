/*
 * libkdes.cpp
 * Description: Kernel Descriptor
 * Copyright (c) 2013, Centre for Intelligent Mechatronics Systems, University of Technology, Sydney, Australia.
 * All rights reserved.
 *
 * This software was developed as a part of Amazon Picking Challenge
 *
 * Author: Kanzhi Wu
 * Date: 23/03/2015
 *
 */

#include "include/kdes/libkdes.h"

KernelDescManager::KernelDescManager(string model_name_,
                                     string model_file_,
                                     string param_file_,
                                     string models_folder_,
                                     unsigned int MODEL_TYPE_,
                                     bool use_output_,
                                     int MAX_IMAGE_SIZE_) :
    model_name(model_name_),
    model_file(model_file_),
    param_file(param_file_),
    models_folder(models_folder_),
    MODEL_TYPE(MODEL_TYPE_),
    use_output(use_output_),
    MAX_IMAGE_SIZE(MAX_IMAGE_SIZE_) {
    model_var = model_file;
    this->model_dir = models_folder + model_file +string(".mat");
    if ( use_output == true )
        cout << "Model Dir: " << this->model_dir.c_str() << " Model file: " << model_file <<  endl;
    this->param_dir = models_folder + param_file + string(".mat");
    this->model_kdes = load_mat("Model file",model_dir.c_str());
    this->kdes_params = load_mat("Param file", param_dir.c_str());
    this->model_kdes_treeroot = model_var + string("->svm->classname");
    this->model_list = get_charlist(this->model_kdes, this->model_kdes_treeroot.c_str());
    //PrintModelList();
    if ( use_output == true )
        if (this->model_list == NULL)
            printf("WARNING: the model list is nulli\n");
        else
            printf("This is a kernel descriptor demo for %d-class object recognition\n",(int)this->model_list->size());

    get_matrix(this->svmW, this->model_kdes,(model_var + string("->svm->w")).c_str());

    this->svmW.transposeInPlace();

}

string KernelDescManager::GetObjectName(VectorXf& scores) {
    // cout << scores.transpose() << endl;
    MatrixXf res = scores;

    vector<mypair> vec;
    for(int i=0; i<res.rows(); i++) {
        vec.push_back( mypair( res(i,0), myindex(i,0) ) );
    }
    sort( vec.begin(), vec.end(), comparator );
    vector<mypair>::iterator itr=vec.begin();


    string str_result =(* this->model_list)[itr->second.first];
    if ( use_output == true ) {
        cout << "OBJECT IS: " << setw(12) << (* this->model_list)[itr->second.first] << "    ";
        for(int k=1; k<min(OBJECTS_TO_PRINT,(int)res.rows()); k++) {
            itr++;
            cout << "[" << setw(12) << (* this->model_list)[itr->second.first] << "] ";
        }
        itr=vec.begin();
        for(int k=0; k<min(OBJECTS_TO_PRINT,(int)res.rows()); k++) {
            cout << " " << setw(12) << (float)itr->first << " ";
            itr++;
        }
        cout << "\r";
    }
    return str_result;
}


int KernelDescManager::Classify(VectorXf& scores, const MatrixXf& imfea) {
    // cout << "Classify 1\n";
    MatrixXf svmMinValue;
    MatrixXf svmMaxValue;
    get_matrix(svmMaxValue, this->model_kdes,(model_var + string("->svm->maxvalue")).c_str());//"modelgkdes->svm->maxvalue");
    get_matrix(svmMinValue, this->model_kdes,(model_var + string("->svm->minvalue")).c_str());//"modelgkdes->svm->minvalue");
    MatrixXf imfea_s = scaletest_power( imfea, svmMinValue, svmMaxValue);

    MatrixXf svmBias;
    get_matrix(svmBias, this->model_kdes, (model_var+string("->svm->bias")).c_str());

    // cout << "Classify 2\n";
    /*
    ofstream outW("w.txt");
    for ( int y = 0; y < svmW.rows(); ++ y ) {
        for ( int x = 0; x < svmW.cols(); ++ x ) {
            outW << svmW(y, x) << " ";
        }
        outW << endl;
    }
    outW.close();
    */

//    cout << "imfea: " << imfea_s.rows() << ", " << imfea_s.cols() << endl;
//    cout << "w: " << this->svmW.rows() << ", " << this->svmW.cols() << endl;
    VectorXf res = ( imfea_s.transpose()*(this->svmW) ).transpose()+svmBias.transpose();
//    cout << "res: " << res.rows() << ", " << res.cols() << endl;
    scores = res;
    // cout << "Classify 3\n";
    MatrixXf::Index idx;
    float maxscore = scores.maxCoeff(&idx);
    // cout << "Classify 4\n";
    return idx;
}


string KernelDescManager::GetObjectName(VectorXf& scores, vector<string>& top_objects, vector<double>& top_scores) {
    MatrixXf res = scores;

    vector<mypair> vec;
    for(int i=0; i<res.rows(); i++) {
        vec.push_back( mypair( res(i,0), myindex(i,0) ) );
    }
    sort( vec.begin(), vec.end(), comparator );

    top_objects.clear();
    top_scores.clear();
    vector<mypair>::iterator itr=vec.begin();
    vector<string>::iterator obj=top_objects.begin();
    string str_result =(* this->model_list)[itr->second.first];

    for(int i=0; i<OBJECTS_TO_PRINT; i++, itr++, obj++) {
        string s=(* this->model_list)[itr->second.first];
        top_objects.push_back(s);
        top_scores.push_back((double)itr->first);
    }

    return str_result;
}


bool KernelDescManager::Process(MatrixXf&imfea, IplImage* image, const VectorXf& top_left) {
    this->top_left=top_left;
    return Process(imfea,image);
}

bool KernelDescManager::Process(MatrixXf&imfea, IplImage* image) {

    //========================================================================
    if ( !(MODEL_TYPE==1 || MODEL_TYPE==2) ) {
        printf("MODEL_TYPE %d unsupported.\n",MODEL_TYPE);
        return false;
    }
    // do the first object in the list
    // convert to floats
    IplImage* img_init = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_32F, image->nChannels);
    if (!img_init)
        printf("Image is unavailable!\n");
    //cvConvertScale(image, img_init,1.0/255, 0);
    switch (MODEL_TYPE) {
    case 1 :
        assert( image->nChannels==1 );  // must be grayscale
        cvConvertScale(image, img_init,1.0/1000, 0);
        break;
    case 2 :
        cvConvertScale(image, img_init,1.0/255, 0);
        break;
    default :

        break;
    }




    //========================================================================

    IplImage* img;

    const double EPS_RATIO=0.0001;
    int max_imsize=(int)get_value<float>(this->model_kdes, (model_var+"->kdes->max_imsize").c_str() );
    int min_imsize=(int)get_value<float>(this->model_kdes, (model_var+"->kdes->min_imsize").c_str() );

    double ratio, ratio_f, ratio_max, ratio_min;
    ratio_f=1.0;
    if (MAX_IMAGE_SIZE>0) {
        ratio_f = max( ratio_f, max( (double)img_init->width/this->MAX_IMAGE_SIZE, (double)img_init->height/this->MAX_IMAGE_SIZE ) );
    }
    ratio_max = max( max( (double)img_init->width/max_imsize, (double)img_init->height/max_imsize ), 1.0 );
    ratio_min = min( min( (double)img_init->width/min_imsize, (double)img_init->height/min_imsize ), 1.0 );
    if (ratio_min<1.0-EPS_RATIO) {
        ratio=ratio_min;
    }
    else {
        ratio=max(ratio_f,ratio_max);
    }
    if (ratio>1.0-EPS_RATIO || ratio<1.0-EPS_RATIO) {
        int method=CV_INTER_CUBIC;
        if (MODEL_TYPE==4 || MODEL_TYPE==5) method=CV_INTER_NN;   // nearest neighbor for depth image
        img = cvCreateImage( cvSize(ceil((img_init->width)/ratio),ceil((img_init->height)/ratio)), IPL_DEPTH_32F, img_init->nChannels);
        cvResize( img_init, img, method );
        cvReleaseImage(&img_init);
    }
    else {
        img=img_init;
    }




    //========================================================================
    int img_w=img->width, img_h=img->height;
    //void GKDESDense(IplImage* im, matvarplus_t* kdes_params, float grid_space, float patch_size, float low_contrast) {
    if ( use_output == true )
        cout << "Start KDES computation..." << "(" << img_w << " " << img_h << ")" << endl;

    MatrixXf feaArr, feaMag, fgrid_y, fgrid_x;
    if (MODEL_TYPE==0 || MODEL_TYPE==3) {
        //cvCvtColor(im, im_temp, CV_RGB2GRAY);
        cout << "before GKDESDense\n";
        GKDESDense(feaArr, feaMag, fgrid_y, fgrid_x, img, this->kdes_params, get_value<float>(this->model_kdes, "modelgkdes->kdes->grid_space"),
        get_value<float>(this->model_kdes, "modelgkdes->kdes->patch_size"),
        get_value<float>(this->model_kdes,"modelgkdes->kdes->low_contrast"));
        cout << "before GKDESDense\n";
    }
    if (MODEL_TYPE==2) {
//        struct timeval t1;
//        gettimeofday(&t1, NULL);
//        long int ms1 = t1.tv_sec * 1000 + t1.tv_usec / 1000; //get current timestamp in milliseconds
        RGBKDESDense(feaArr, feaMag, fgrid_y, fgrid_x, img, this->kdes_params, get_value<float>(this->model_kdes, "modelrgbkdes->kdes->grid_space"), get_value<float>(this->model_kdes, "modelrgbkdes->kdes->patch_size"));
//        struct timeval t2;
//        gettimeofday(&t2, NULL);
//        long int ms2 = t2.tv_sec * 1000 + t2.tv_usec / 1000; //get current timestamp in milliseconds
//        cout << "Time duration in KDES: " << ms2-ms1 << endl;
    }
    if (MODEL_TYPE==4) {
        cout << "before SpinKDESDense\n";
        SpinKDESDense(feaArr, fgrid_y, fgrid_x, img, this->top_left, this->kdes_params, get_value<float>(this->model_kdes, "modelspinkdes->kdes->grid_space"), get_value<float>(this->model_kdes, "modelspinkdes->kdes->patch_size"));
        cout << "after SpinKDESDense\n";
    }


    //  MatrixXf imfea;
    MatrixXf emkWords;
    get_matrix(emkWords, this->model_kdes, (string(this->model_var)+"->emk->words").c_str());
    MatrixXf emkG;
    get_matrix(emkG, this->model_kdes, (string(this->model_var)+"->emk->G").c_str());
    MatrixXf emkPyramid;
    get_matrix(emkPyramid, this->model_kdes, (string(this->model_var)+"->emk->pyramid").c_str());

//    struct timeval t4;
//    gettimeofday(&t4, NULL);
//    long int ms4 = t4.tv_sec * 1000 + t4.tv_usec / 1000; //get current timestamp in milliseconds
//    cout << "4: " << ms4 << endl;
    CKSVDEMK(imfea, feaArr, feaMag, fgrid_y, fgrid_x, img_h, img_w, emkWords, emkG, emkPyramid, get_value<float>(this->model_kdes, (string(this->model_var)+"->emk->kparam").c_str()) );
//    struct timeval t5;
//    gettimeofday(&t5, NULL);
//    long int ms5 = t5.tv_sec * 1000 + t5.tv_usec / 1000; //get current timestamp in milliseconds
//    cout << "5: " << ms5 << endl;



    cvReleaseImage(&img);


    fflush(stdout);
    return true;
}

void KernelDescManager::PrintModelList() {
    printf("Available models:\n");
    foreach(string s, *model_list) {
        printf("\t%s\n", s.c_str());
    }
}

vector<string> * KernelDescManager::GetModelList() {
    return this->model_list;
}

KernelDescManager::~KernelDescManager() {
    delete model_kdes;
    delete kdes_params;
    delete model_list;
}


















