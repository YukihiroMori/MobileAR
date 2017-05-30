//
//  CalcDistance.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/21.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef CalcDistance_hpp
#define CalcDistance_hpp

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class CalcDistance{
public:
    MatND hist , hist2;
    
    CalcDistance(){};
    
    void setTargetHist(Mat &image);
    float calcDistance(Mat &image);
};


#endif /* CalcDistance_hpp */
