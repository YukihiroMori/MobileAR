//
//  CalcDistance.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/21.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "CalcDistance.hpp"


void CalcDistance::setTargetHist(Mat &image){
    const int channels[] = { 0, 1, 2 };
    float r_r[] = { 0.0f, 256.0f };
    float g_r[] = { 0.0f, 256.0f };
    float b_r[] = { 0.0f, 256.0f };
    const float* ranges[] = { r_r, g_r, b_r };
    const int histSize[] = { 32, 32, 32 };
    
    calcHist(&image, 1, channels, cv::Mat(), hist, 3, histSize, ranges, true ,false);
}

float CalcDistance::calcDistance(Mat &image){
    const int channels[] = { 0, 1, 2 };
    float r_r[] = { 0.0f, 256.0f };
    float g_r[] = { 0.0f, 256.0f };
    float b_r[] = { 0.0f, 256.0f };
    const float* ranges[] = { r_r, g_r, b_r };
    const int histSize[] = { 32, 32, 32 };
    
    calcHist(&image, 1, channels, cv::Mat(), hist2, 3, histSize, ranges, true ,false);
    
    return compareHist(hist, hist2, CV_COMP_CORREL);
}
























