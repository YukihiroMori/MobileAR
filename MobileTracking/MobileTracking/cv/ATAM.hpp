//
//  ATAM.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/07.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef ATAM_hpp
#define ATAM_hpp

/*!
 @file		ATAM.h
 @brief		header for CATAM
 */

#pragma once

#include <thread>
#include <string>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "ATAMData.hpp"
#include "Cam.hpp"
#include "PointDetector.hpp"
#include "Calibration.hpp"
#include "cvsba.h"
#include "FMatrixComputation.hpp"
#include "TwoViewReconstruction.hpp"
#include "Utils.hpp"
#include "BundleAdjusment.hpp"
#include "State.hpp"
#include "Preview.hpp"

using namespace std;
using namespace glm;
using namespace Eigen;


/*!
 @class		CATAM
 @brief		ATAM
 */
class CATAM
{
public:
    CATAM();
    
public:
    void Start(string path);
    void Stop(void);
    void loop(cv::Mat img , cv::Mat &prev);
    
    bool operation(const int key);
    
    sPose Gyro;
    
private:
    // setting
    bool init(string path);
    
    // process
    void process(void);
    void startInit(void);
    void startTAM(void);
    void changeState(void);
    void reset(void);
    
    // tracking
    bool setKeyframe(void);
    bool checkInsideImage(const cv::Point2f &pt) const;
    int trackFrame(void);
    bool matchKeyframe(void);
    bool computePose(void);
    
    // mapping
    void computePosefromE(const std::vector<cv::Point2f> &vUnPt1, const std::vector<cv::Point2f> &vUnPt2, cv::Mat &rvec, cv::Mat &tvec) const;
    void triangulate(const std::vector<cv::Point2f> &vUnPt1, const std::vector<cv::Point2f> &vUnPt2, const sPose &pose1, const sPose &pose2, std::vector<cv::Point3f> &vpt3d) const;
    bool makeMap(void);
    bool mappingCriteria(void);
    void mapping(void);
    void trackAndMap(void);
    void whileInitialize(void);
    
    // registration with world coordinate system
public:
    void transformToWorld(const sPose &local, sPose &world) const;
private:
    bool getWorldCoordinate(sPose &pose) ;
    void registerWorld(void);
    
    // relocalization
    void changeRelocalImage(void);
    void relocalize(void);
    
    // challenge points for competition
    void loadChallenge(const std::string &name);
    
    
private:
public:
    CCam mCam;			//!< camera
    cv::Mat mImg;		//!< current image (from camera)
    cv::Mat mGImg;		//!< current image (gray scale)
    sPose mPose;		//!< current local camera pose
    sPose mWPose;		//!< current world camera pose
    int mFrameNumber;	//!< frame number
    int mChallengeNumber;		//!< challenge point number
    
    CCalibration mCalibrator;	//!< calibration
    CPointDetector mDetector;	//!< keypoint detector
    
    double mFPS;		//!< frame per second in main process
    
    BundleAdjusment BA;
    
    STATE mState;		//!< state in ATAM
    sATAMData mData;	//!< all data in ATAM
    
    bool mReflesh;
    
private:
    int relocalfailed;
    
    Matrix3d Rot = Matrix3d::Identity(3,3);
    Vector3d Pos = Vector3d::Zero(3);
    vector<cv::Point2f> point1;
    vector<cv::Point2f> point2;
    
    Preview preview;
};

#endif /* ATAM_hpp */
