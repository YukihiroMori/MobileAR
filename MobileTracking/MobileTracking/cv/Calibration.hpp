//
//  Calibration.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/07.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef Calibration_hpp
#define Calibration_hpp
/*!
 @file		Calibration.h
 @brief		header for CCalibration
 */

#pragma once

#include "opencv2/opencv.hpp"

/*!
 @class		CCalibration
 @brief		calibration
 */
class CCalibration
{
public:
    CCalibration();
    
    bool EstimatePose(const cv::Mat &img, const cv::Mat &A, const cv::Mat &D, cv::Mat &rvec, cv::Mat &tvec) const;
    void Calibrate(const std::vector< std::vector< cv::Point2f > > &imagePoints, const cv::Size &imageSize, cv::Mat &intrinsic, cv::Mat &distortion) const;
    bool DetectCorners(const cv::Mat &gImg, std::vector<cv::Point2f> &vCorner) const;
    void DrawCorners(cv::Mat &img, const std::vector<cv::Point2f> &vCorner) const;
    
private:
    cv::Size mPattern;		//!< calibration pattern
    float mSize;			//!< square size or dot interval
    std::vector<cv::Point3f> mVpt3d;	//!< 3D coordinates of corners
};

#endif /* Calibration_hpp */
