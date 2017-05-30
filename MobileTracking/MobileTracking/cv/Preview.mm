//
//  Preview.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/13.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "Preview.hpp"
#include "ATAM.hpp"

void Preview::draw(CATAM &atam, cv::Mat &img)
{
    switch (atam.mState) {
        case STATE::INIT:
            drawTrack(atam, img);
            break;
        case STATE::TAM:
            drawTrack(atam, img);
            drawMap(atam, img);
            drawGrid(atam, img);
            drawChallenge(atam, img);
            break;
        case STATE::RELOCAL:
            //drawView(atam, img);
            break;
        default:
            break;
    }
    
    drawProcess(atam, img);
}

void Preview::drawView(CATAM &atam, cv::Mat &img)
{
    sKeyframe kf = atam.mData.map.GetNearestKeyframe(atam.mPose);
    
    cv::Mat mono = cv::Mat(img.size(), CV_8U);
    cv::Canny(kf.img, mono, 10, 100);
    
    std::vector<cv::Mat> color(4, mono);
    
    cv::Mat cimg;
    cv::merge(color, cimg);
    
    double alpha = 0.5;
    double beta = (1.0 - alpha);
    addWeighted(img.clone(), alpha, cimg, beta, 0.0, img);
}


void Preview::drawProcess(CATAM &atam, cv::Mat &img)
{
    cv::Scalar textCol(0, 255, 0 , 255);
    putText(img, mText, cv::Point(0, img.rows - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, textCol, 2);
    
    textCol = cv::Scalar(0, 255, 0 , 255);
    putText(img, to_string(int(atam.mFPS)) + " FPS", cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.3, textCol, 1);
    
    if (atam.BA.getDoing()) {
        textCol = cv::Scalar(0, 0, 255 , 255);
        putText(img, "Doing BA", cv::Point(img.cols - 75, 15), cv::FONT_HERSHEY_SIMPLEX, 0.3, textCol, 1);
    }
}

void Preview::drawMap(CATAM &atam, cv::Mat &img) const
{
    const std::list<sTrack> &vTrack = atam.mData.vTrack;
    
    std::vector<cv::Point3f> vPt3d;
    
    for (std::list<sTrack>::const_iterator it = vTrack.begin(), itend = vTrack.end();
         it != itend; ++it) {
        if (it->ptID != NOID) {
            vPt3d.push_back(atam.mData.map.GetPoint(it->ptID));
        }
    }
    
    if (vPt3d.size() != 0) {
        vector<cv::Point2f> vpt2d;
        cv::projectPoints(vPt3d, atam.mPose.rvec, atam.mPose.tvec, atam.mData.A, atam.mData.D, vpt2d);
        
        cv::Scalar col(0, 0, 255 , 255);
        const int size = 1;
        for (int i = 0, iend = int(vpt2d.size()); i < iend; ++i) {
            circle(img, cv::Point(vpt2d[i]), size, col, -1);
        }
    }
}

void Preview::drawGrid(CATAM &atam, cv::Mat &img) const
{
    if (atam.mData.haveScale) {
        
        sPose tmp;
        atam.transformToWorld(atam.mPose, tmp);
        
        cv::Scalar col(0, 255, 0 , 255);
        const int lineWidth = 1;
        
        const int size = 2;
        std::vector<cv::Point3f> vPt3d(2 * size);
        const float interval = 100.f;
        
        for (int i = 0; i < size; ++i) {
            vPt3d[2 * i] = cv::Point3f(interval*i, 0, 0);
            vPt3d[2 * i + 1] = cv::Point3f(interval*i, interval*(size - 1), 0);
        }
        
        vector<cv::Point2f> vPt2d;
        projectPoints(vPt3d, tmp.rvec, tmp.tvec, atam.mData.A, atam.mData.D, vPt2d);
        
        for (int i = 0, iend = int(vPt2d.size()) / 2; i < iend; ++i) {
            cv::line(img, cv::Point(vPt2d[2 * i]), cv::Point(vPt2d[2 * i + 1]), col, lineWidth);
        }
        
        for (int i = 0; i < size; ++i) {
            vPt3d[2 * i] = cv::Point3f(0, interval*i, 0);
            vPt3d[2 * i + 1] = cv::Point3f(interval*(size - 1), interval*i, 0);
        }
        
        projectPoints(vPt3d, tmp.rvec, tmp.tvec, atam.mData.A, atam.mData.D, vPt2d);
        
        for (int i = 0, iend = int(vPt2d.size()) / 2; i < iend; ++i) {
            line(img, cv::Point(vPt2d[2 * i]), cv::Point(vPt2d[2 * i + 1]), col, lineWidth);
        }
    }
}

void Preview::drawTrack(CATAM &atam, cv::Mat &img) const
{
    const int pointSize = 1;
    
    const std::list<sTrack> &vTrack = atam.mData.vTrack;
    
    cv::Scalar border(0, 0, 0 , 255);
    cv::Scalar mapped(255, 255, 255 , 255);
    cv::Scalar newpt(255, 0, 0 , 255);
    
    for (list<sTrack>::const_iterator it = vTrack.begin(), itend = vTrack.end();
         it != itend; ++it) {
        if (it->ptID != NOID) {
            circle(img, cv::Point(it->vPt.back()), pointSize, border, -1);
            circle(img, cv::Point(it->vPt.back()), pointSize - 1, mapped, -1);
        }
        else {
            circle(img, cv::Point(it->vPt.back()), pointSize, border, -1);
            circle(img, cv::Point(it->vPt.back()), pointSize - 1, newpt, -1);
        }
    }
}

void Preview::drawChallenge(CATAM &atam, cv::Mat &img)
{
    if (atam.mData.haveScale) {
        
        cv::Scalar col(0, 0, 255 ,255);
        
        const int lineWidth = 2;
        const int radius = 15;
        
        sPose tmp;
        atam.transformToWorld(atam.mPose, tmp);
        
        std::vector<cv::Point2f> vPt2d;
        std::vector<cv::Point3f> vPt3d;
        int ID = 0;
        
        int i = 0;
        for (map<int, cv::Point3f>::const_iterator it = atam.mData.vChallenge.begin(),
             itend = atam.mData.vChallenge.end(); it != itend; ++it, ++i) {
            if (atam.mChallengeNumber == i) {
                ID = it->first;
                vPt3d.push_back(it->second);
                break;
            }
        }
        
        if (vPt3d.size() == 1) {
            projectPoints(vPt3d, tmp.rvec, tmp.tvec, atam.mData.A, atam.mData.D, vPt2d);
            
            cv::Point pt = vPt2d[0];
            circle(img, pt, radius, col, lineWidth);
            putText(img, std::to_string(ID), pt + cv::Point(radius, 0), cv::FONT_HERSHEY_SIMPLEX, 1.5, col, 2);
        }
        else {
            mText = "No more challenge points";
        }
    }
}
