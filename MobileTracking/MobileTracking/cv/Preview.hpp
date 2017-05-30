//
//  Preview.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/13.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef Preview_hpp
#define Preview_hpp

#include <string>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>


class CATAM;

class Preview {
    
public:
    std::string mText;	//!< instruction on image
    
    void drawChallenge(CATAM &atam, cv::Mat &img);
    
    void drawView(CATAM &atam, cv::Mat &img);
    
    void draw(CATAM &atam, cv::Mat &img);
    void drawProcess(CATAM &atam, cv::Mat &img);
    void drawMap(CATAM &atam, cv::Mat &img) const;
    void drawGrid(CATAM &atam, cv::Mat &img) const;
    void drawTrack(CATAM &atam, cv::Mat &img) const;
};

#endif /* Preview_hpp */
