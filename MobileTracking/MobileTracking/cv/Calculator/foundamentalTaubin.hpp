//
//  foundamentalTaubin.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/24.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef foundamentalTaubin_hpp
#define foundamentalTaubin_hpp

#import <opencv2/opencv.hpp>
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/features2d.hpp>
#import <opencv2/xfeatures2d.hpp>
#include <glm/glm.hpp>

using namespace std;
using namespace cv;
using namespace glm;

Mat fundamentalTaubin(vector<vec2> x1, vector<vec2> x2, int N, double f);

#endif /* foundamentalTaubin_hpp */
