//
//  FeatureMatcher.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/23.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef FeatureMatcher_hpp
#define FeatureMatcher_hpp

#import <opencv2/opencv.hpp>
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/features2d.hpp>
#import <opencv2/xfeatures2d.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

using namespace std;
using namespace cv;
using namespace glm;

class FeatureMatcher{
    Mat target;
    Mat t_gray;
    Mat t_desc;
    
    vector<KeyPoint> t_Keyptr;
    vector<KeyPoint> f_Keyptr;
    vector<DMatch> good_matches;
    
    Ptr<Feature2D> feature;
    Ptr<DescriptorMatcher> matcher;
    const float match_par = .6f;
    
    Mat H;
    Mat frame;
    
public:
    
    FeatureMatcher(){
        feature = AKAZE::create();
        matcher = DescriptorMatcher::create("BruteForce-Hamming(2)");
    }
    
    bool setTarget(Mat image){
        target = image;
        cvtColor(target, t_gray, CV_RGB2GRAY);
        feature->detectAndCompute(t_gray, noArray(), t_Keyptr, t_desc);
        
        cout << "Template Features : " << t_desc.rows << endl;
        
        if(t_desc.rows == 0){
            return false;
        }
        
        return true;
    }
    
    void FMSearch(Mat image, vec2 &estimate, bool &isTracked){
        isTracked = false;
        
        frame = image;
        
        Mat f_gray;
        Mat f_desc;
        
        f_Keyptr.clear();
        cvtColor(frame, f_gray, CV_RGB2GRAY);
        feature->detectAndCompute(f_gray, cv::noArray(), f_Keyptr, f_desc);
        
        if(f_desc.rows == 0){
            return;
        }
        
        
        vector<vector<DMatch>> matches, match12, match21;
        matcher->knnMatch(t_desc, f_desc, match12,2);
        matcher->knnMatch(f_desc, t_desc, match21,2);
        for (size_t i = 0; i < match12.size(); i++)
        {
            DMatch forward = match12[i][0];
            DMatch backward = match21[forward.trainIdx][0];
            if (backward.trainIdx == forward.queryIdx)
            {
                matches.push_back(match12[i]);
            }
        }
        
        good_matches.clear();
        
        vector<Point2f> match_point1;
        vector<Point2f> match_point2;
        
        for (size_t i = 0; i < matches.size(); ++i) {
            auto dist1 = matches[i][0].distance;
            auto dist2 = matches[i][1].distance;
            
            if (dist1 <= dist2 * match_par) {
                good_matches.push_back(matches[i][0]);
                match_point1.push_back(t_Keyptr[matches[i][0].queryIdx].pt);
                match_point2.push_back(f_Keyptr[matches[i][0].trainIdx].pt);
            }
        }
        
        Mat masks;
        H.release();
        if (match_point1.size() != 0 && match_point2.size() != 0) {
            H = findHomography(match_point1, match_point2, masks, 0, 3.f);
        }else {
            return;
        }
        
        if (!H.empty()) {
            vector<Point2f> obj_corners(4);
            obj_corners[0] = Point2f(.0f, .0f);
            obj_corners[1] = Point2f(static_cast<float>(target.cols), .0f);
            obj_corners[2] = Point2f(static_cast<float>(target.cols), static_cast<float>(target.rows));
            obj_corners[3] = Point2f(.0f, static_cast<float>(target.rows));
            
            vector<Point2f> scene_corners(4);
            perspectiveTransform(obj_corners, scene_corners, H);
            
            Point2f pos = scene_corners[0] + scene_corners[1] + scene_corners[2] + scene_corners[3];
            pos.x /= 4.0f;
            pos.y /= 4.0f;
            estimate = vec2(pos.x, pos.y);
            isTracked = true;
        }
    }
    
    void drawResult(Mat &preview){
        vector<bool> matched(f_Keyptr.size());
        
        for(int i = 0;i < f_Keyptr.size(); i++){
            matched[i] = false;
            for(int j = 0; j < good_matches.size(); j++){
                if(i == good_matches[j].trainIdx){
                    matched[i] = true;
                }
            }
        }
        
        for(int i = 0;i < f_Keyptr.size(); i++){
            if(matched[i])
                circle(preview, f_Keyptr[i].pt , 3, Scalar(0,255,0,255), 1, CV_AA);
            else
                circle(preview, f_Keyptr[i].pt , 3, Scalar(255,0,0,255), 1, CV_AA);
        }
        
        if (!H.empty() && good_matches.size() > 8 ) {
            vector<Point2f> obj_corners(4);
            obj_corners[0] = Point2f(.0f, .0f);
            obj_corners[1] = Point2f(static_cast<float>(target.cols), .0f);
            obj_corners[2] = Point2f(static_cast<float>(target.cols), static_cast<float>(target.rows));
            obj_corners[3] = Point2f(.0f, static_cast<float>(target.rows));
            
            vector<Point2f> scene_corners(4);
            perspectiveTransform(obj_corners, scene_corners, H);
            
            line(preview, scene_corners[0] , scene_corners[1] , Scalar(0, 255, 0, 255), 1);
            line(preview, scene_corners[1] , scene_corners[2] , Scalar(0, 255, 0, 255), 1);
            line(preview, scene_corners[2] , scene_corners[3] , Scalar(0, 255, 0, 255), 1);
            line(preview, scene_corners[3] , scene_corners[0] , Scalar(0, 255, 0, 255), 1);
        }
    }
    
    vec2 estimate;
};



#endif /* FeatureMatcher_hpp */
