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
#include <mutex>
#include "Utils.hpp"
#include "ATAM.hpp"
#import <UIKit/UIKit.h>

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

    std::vector<cv::Mat> vimg;
    
public:
    CATAM atam;
    
    ImageProcessor();
    
    UILabel *trackingTimeLabel;
    UILabel *currentStateLabel;
    UITextView *informationView;
    
    UIImage* processing(UIImage *image);
    
    void setGyro(double x, double y, double z);
    
    void touchStart(vec2 position);
    void touchMove(vec2 position);
    void touchEnd(vec2 position);
    
    void start();
    void reset();
    void N();
    void C();
    void Q();
    
    void stop();
    void Calibration();
};


#endif /* ImageProcessor_hpp */
