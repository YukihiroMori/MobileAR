//
//  TemplateMatching.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/22.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef TemplateMatcher_hpp
#define TemplateMatcher_hpp

#import <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "Utils.hpp"

using namespace std;
using namespace cv;
using namespace glm;

class TemplateMatcher{
    Mat pyramid(Mat src, int n);
    
    Mat tmp;
    Mat tmp1_2;
    Mat tmp1_4;
    
    int width;
    int height;
    
    void coaseToFine(Mat src, Mat tmp, double &value, cv::Point &point ,int n);
    
public:
    float evaluate;
    vec2 estimate;
    
    void setTemplate(Mat tmp);
    void TM(Mat src);
    void drawResult(Mat &rsc);
    
};


#endif /* TemplateMatcher_hpp */
