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
#include "ParticleFilter.hpp"
#include "CalcDistance.hpp"
#include "TemplateMatcher.hpp"
#include "FeatureMatcher.hpp"
#include "OpticalFlowCalculator.hpp"
#include "QRFinder.hpp"

using namespace std;
using namespace cv;
using namespace glm;

class ImageProcessor{
    enum Process{ NoTrackingObject, NoTracked, PF, TM , FM , OF, QR};
    
private:
    Process state = NoTrackingObject;
    
    mutex loc;
    
    Mat current;
    vec2 area;
    
    bool selectObject = false;
    bool trackObject = false;
    
    vec2 trackpos;
    
    vec2 trackSize = vec2(120.0f);
    
    Mat searchObject;
    
    auto_ptr<ParticleFilter> pf;
    auto_ptr<TemplateMatcher> tm;
    auto_ptr<FeatureMatcher> fm;
    auto_ptr<OpticalFlowCalculator> of;
    auto_ptr<QRFinder> qr;
    
    void PFSearch(vec2 &position, bool &isTracked);
    void TMSearch(vec2 &position, bool &isTracked);
    
    cv::Rect trect;
    
public:
    ImageProcessor();
    
    UIImageView *searchImageView;
    UILabel *trackingTimeLabel;
    UILabel *currentStateLabel;
    UILabel *informationLabel;
    
    UIImage* processing(UIImage *image);
    
    void touchStart(vec2 position);
    void touchMove(vec2 position);
    void touchEnd(vec2 position);
    
    void drawPreview(Mat result);
    
    UIImage* drawRect(UIImage *image, int x, int y, int w, int h){
        Mat preview;
        UIImageToMat(image, preview);
        
        rectangle(preview, cv::Point(x,y), cv::Point(x+w, y+h), Scalar(200,0,0,255), 1, CV_AA);
        
        return MatToUIImage(preview);
    }
};

#endif /* ImageProcessor_hpp */
