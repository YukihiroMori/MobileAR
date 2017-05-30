//
//  ImageProcessor.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/17.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef ImageProcessor_hpp
#define ImageProcessor_hpp

#import <opencv2/opencv.hpp>
#import <opencv2/imgcodecs/ios.h>
#include <opencv2/highgui/highgui.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#import <UIKit/UIKit.h>
#include <mutex>
#include "Utils.hpp"
#include "FrameTracker.hpp"
#include <Eigen/Dense>

using namespace std;
using namespace cv;
using namespace glm;

class ImageProcessor{
    enum Process{ NotInitialized, Tracking };
    
private:
    Process state = NotInitialized;
    
    mutex loc;
    
    Mat current;
    vec2 area;
    
    bool selectObject = false;
    
    vec2 trackpos;
    vec2 trackSize = vec2(100.0f);
    
    auto_ptr<FrameTracker> ft;
    
    Matrix3d Rot = Matrix3d::Identity(3,3);
    Vector3d Pos = Vector3d::Zero(3);
    
public:
    ImageProcessor();
    
    UIImageView *leftImageView;
    UIImageView *rightImageView;
    UILabel *trackingTimeLabel;
    UILabel *currentStateLabel;
    UITextView *informationView;
    
    UIImage* processing(UIImage *image);
    
    void touchStart(vec2 position);
    void touchMove(vec2 position);
    void touchEnd(vec2 position);
    
    void drawPreview(Mat result);
};

#endif /* ImageProcessor_hpp */
