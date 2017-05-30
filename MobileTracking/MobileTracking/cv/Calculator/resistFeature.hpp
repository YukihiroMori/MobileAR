//
//  resistFeature.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/24.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef resistFeature_hpp
#define resistFeature_hpp
#import <opencv2/opencv.hpp>
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/features2d.hpp>
#import <opencv2/xfeatures2d.hpp>
#include <glm/glm.hpp>

using namespace std;
using namespace cv;
using namespace glm;

void resistFeature(vector<DMatch> matches,vector<KeyPoint> k1, vector<KeyPoint> k2, vector<vec2> &point1, vector<vec2> &point2);


#endif /* resistFeature_hpp */
