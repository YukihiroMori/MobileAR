//
//  FrameTracker.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/24.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef FrameTracker_hpp
#define FrameTracker_hpp

#import <opencv2/opencv.hpp>
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/features2d.hpp>
#import <opencv2/xfeatures2d.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

using namespace std;
using namespace cv;
using namespace glm;

class FrameTracker{
    Ptr<Feature2D> feature;
    Ptr<DescriptorMatcher> matcher;
    const float match_par = .6f;
    
    Mat startImage;
    Mat endImage;
    vector<KeyPoint> s_Keyptr;
    vector<KeyPoint> e_Keyptr;
    vector<DMatch> good_matches;
    
public:
    FrameTracker(){
        feature = AKAZE::create();
        matcher = DescriptorMatcher::create("BruteForce-Hamming(2)");
    }
    
    bool init_start(Mat start){
        startImage = start;
        Mat s_gray;
        Mat s_desc;
        
        cvtColor(startImage, s_gray, CV_RGB2GRAY);
        feature->detectAndCompute(s_gray, noArray(), s_Keyptr, s_desc);
        
        cout << "Template Features : " << s_desc.rows << endl;
        
        if(s_desc.rows == 0){
            return false;
        }
        
        return true;
    }
    
    void init_end(Mat end){
        endImage = end;
    }
    
};





















#endif /* FrameTracker_hpp */
