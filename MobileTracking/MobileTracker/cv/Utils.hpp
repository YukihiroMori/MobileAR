//
//  Utils.h
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/22.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//
#ifndef Utils_h
#define Utils_h

#define Debug 1

#include <chrono>

#define CALC_START() chrono::system_clock::time_point  start, end;\
start = chrono::system_clock::now();

#define CALC_END(frase) end = chrono::system_clock::now();\
double elapsed = chrono::duration_cast<std::chrono::milliseconds>(end-start).count();\
cout << frase << elapsed << "ms" << endl;

#define CALC_ENDV2(frase) end = chrono::system_clock::now();\
double elapsed = chrono::duration_cast<std::chrono::milliseconds>(end-start).count();\
stringstream ss;\
ss << frase << elapsed << "ms";\
{string s_calc = ss.str();\
dispatch_async(dispatch_get_main_queue(), ^{\
    trackingTimeLabel.text =  [NSString stringWithUTF8String:s_calc.c_str()];\
});}

#define Infomation(inf) {string s_inf = inf;\
dispatch_async(dispatch_get_main_queue(), ^{\
    informationLabel.text =  [NSString stringWithUTF8String:s_inf.c_str()];\
});}

#define State(state) {string s_state = state;\
dispatch_async(dispatch_get_main_queue(), ^{\
currentStateLabel.text =  [NSString stringWithUTF8String:s_state.c_str()];\
});}

#import <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#import <glm/glm.hpp>


static bool isArea(cv::Mat m, cv::Rect rc){
    if( rc.x < 0.0f || rc.y < 0.0f || rc.x + rc.width > m.size().width || rc.y + rc.height > m.size().height ){
        return false;
    }
    return true;
}

static bool isArea(glm::vec2 area, glm::vec2 pos, glm::vec2 size){
    float x = pos.x - size.x / 2.0f;
    float y = pos.y - size.y / 2.0f;
    float w = size.x;
    float h = size.y;
    
    if( x < 0.0f || y < 0.0f || x + w > area.x || y + h > area.y){
        return false;
    }
    return true;
}

static cv::Rect createRect(glm::vec2 pos, glm::vec2 size){
    cv::Rect rct;
    rct.x = pos.x - size.x / 2.0f;
    rct.y = pos.y - size.y / 2.0f;
    rct.width = size.x;
    rct.height = size.y;
    
    return rct;
}


#endif /* Utils_h */
