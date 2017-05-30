//
//  BundleAdjusment.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/13.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef BundleAdjusment_hpp
#define BundleAdjusment_hpp

#include <thread>
#include <string>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "ATAMData.hpp"
#include "State.hpp"
#include "cvsba.h"

using namespace std;
using namespace glm;
using namespace Eigen;

class CATAM;

class BundleAdjusment{
    
public:
    BundleAdjusment();
    
    bool initialBA(sATAMData &mData,vector<cv::Point3f> &vPt3d,	const vector<cv::Point2f> &vDist1, const vector<cv::Point2f> &vDist2, sPose &pose1, sPose &pose2);
    void BA(CATAM &atam);
    
    bool getDoing();
    
    
    
private:
    cvsba::Sba mBA;			//!< bundle adjustment (BA)
    bool mDoingBA;			//!< doing BA or not
    mutex mBAMutex;	//!< for BA
    
    thread BAth;
};

#endif /* BundleAdjusment_hpp */
