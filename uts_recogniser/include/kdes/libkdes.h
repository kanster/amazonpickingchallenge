/*
 * libkdes.h
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

#ifndef LIBKDES_H
#define LIBKDES_H

#include "helpfun.hpp"
#include "mat.h"

#include <string>
#include <vector>


class KernelDescManager {

public:
    KernelDescManager(string model_name_,string model_file_, string param_file_, string models_folder_, unsigned int MODEL_TYPE_, bool use_output_=true, int MAX_IMAGE_SIZE_=0);
    ~KernelDescManager();
    //		bool Process(MatrixXf& imfea, string image_path);
    bool Process(MatrixXf&imfea, IplImage* image);
    bool Process(MatrixXf&imfea, IplImage* image, const VectorXf& top_left);
    string GetObjectName(VectorXf& scores);
    string GetObjectName(VectorXf& scores, vector<string>& top_objects, vector<double>& top_scores);
    int Classify(VectorXf& scores, const MatrixXf& imfea);
    void PrintModelList();
    vector<string> * GetModelList();

private:
    string model_name;
    string model_file;
    string param_file;
    string model_var;
    string models_folder;
    string model_dir;
    string param_dir;
    string model_kdes_treeroot;
    matvarplus_t* model_kdes, *kdes_params;
    vector<string>* model_list;
    unsigned int MODEL_TYPE;
    int MAX_IMAGE_SIZE;
    VectorXf top_left;
    bool use_output;
    MatrixXf svmW;
};


#endif
