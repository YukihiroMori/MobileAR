//
//  BundleAdjusment.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/13.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "ATAM.hpp"
#include "BundleAdjusment.hpp"

BundleAdjusment::BundleAdjusment(){
    mDoingBA = false;
}

bool BundleAdjusment::getDoing() {
    bool tmp = false;
    
    {
        lock_guard<mutex> lock(mBAMutex);
        tmp = mDoingBA;
    }
    return tmp;
}

bool BundleAdjusment::initialBA(
                                 sATAMData &mData,
                                 vector<cv::Point3f> &vPt3d,
                                 const vector<cv::Point2f> &vDist1,
                                 const vector<cv::Point2f> &vDist2,
                                 sPose &pose1,
                                 sPose &pose2)
{
    vector< vector<cv::Point2f> > imagePoints(2);
    imagePoints[0] = vDist1;
    imagePoints[1] = vDist2;
    
    vector< vector<int> > visibility(2);
    vector<int> vvis(vDist1.size(), 1);
    visibility[0] = vvis;
    visibility[1] = vvis;
    
    vector<cv::Mat> cameraMatrix(2);
    cameraMatrix[0] = mData.A.clone();
    cameraMatrix[1] = mData.A.clone();
    
    vector<cv::Mat> R(2);
    pose1.rvec.copyTo(R[0]);
    pose2.rvec.copyTo(R[1]);
    
    vector<cv::Mat> T(2);
    pose1.tvec.copyTo(T[0]);
    pose2.tvec.copyTo(T[1]);
    
    vector<cv::Mat> distCoeffs(2);
    distCoeffs[0] = cv::Mat::zeros(cv::Size(1, 5), CV_64F);
    distCoeffs[1] = cv::Mat::zeros(cv::Size(1, 5), CV_64F);
    
    LOGOUT("initial BA with %d points\n", int(vPt3d.size()));
    double val = mBA.run(vPt3d, imagePoints, visibility, cameraMatrix, R, T, distCoeffs);
    LOGOUT("Initial error = \t %f\n", mBA.getInitialReprjError());
    LOGOUT("Final error = \t %f\n", mBA.getFinalReprjError());
    
    if (val < 0.0) {
        return false;
    }
    
    R[0].copyTo(pose1.rvec);
    R[1].copyTo(pose2.rvec);
    
    T[0].copyTo(pose1.tvec);
    T[1].copyTo(pose2.tvec);
    
    return true;
}


void BundleAdjusment::BA(CATAM &atam)
{
    bool tmp;
    mBAMutex.lock();
    tmp = atam.mReflesh;
    mBAMutex.unlock();
    
    if(!tmp)
        return;
    
    STATE mainState = atam.mState;
    
    if (mainState == STATE::TAM) {
        
        sBAData baData;
        
        bool copied = atam.mData.map.CopytoBA(baData);
        
        vector<sKeyframe> &vKf = baData.vKeyframe;
        int numKeyframes = int(vKf.size());
        
        if (copied && numKeyframes > 2) {
            
            mBAMutex.lock();
            mDoingBA = true;
            mBAMutex.unlock();
            
            vector<cv::Point3f> &vPt3d = baData.vPt3d;
            vector<int> checkVis(vPt3d.size(), 0);
            
            for (int i = 0, iend = numKeyframes; i < iend; ++i) {
                sKeyframe &kf = vKf[i];
                for (int j = 0, jend = int(kf.vPtID.size()); j < jend; ++j) {
                    ++checkVis[kf.vPtID[j]];
                }
            }
            
            vector<cv::Point3f> vUsedPt3d;
            vector<int> vVisibleID;
            
            for (int i = 0, iend = int(checkVis.size()); i < iend; ++i) {
                if (checkVis[i] > 1) {
                    int num = int(vUsedPt3d.size());
                    checkVis[i] = num;
                    vVisibleID.push_back(i);
                    vUsedPt3d.push_back(vPt3d[i]);
                }
                else {
                    checkVis[i] = -1;
                }
            }
            
            vector< vector<cv::Point2f> > imagePoints(numKeyframes);
            vector< vector<int> > visibility(numKeyframes);
            vector<cv::Mat> cameraMatrix(numKeyframes);
            vector<cv::Mat> R(numKeyframes);
            vector<cv::Mat> T(numKeyframes);
            vector<cv::Mat> distCoeffs(numKeyframes);
            
            for (int i = 0; i < numKeyframes; ++i) {
                
                vector<cv::Point2f> points(vUsedPt3d.size());
                vector<int> vis(vUsedPt3d.size(), 0);
                
                sKeyframe &kf = vKf[i];
                for (int j = 0, jend = int(kf.vPtID.size()); j < jend; ++j) {
                    int id = checkVis[kf.vPtID[j]];
                    
                    if (id != -1) {
                        points[id] = kf.vPt[j];
                        vis[id] = 1;
                    }
                }
                
                imagePoints[i] = points;
                visibility[i] = vis;
                cameraMatrix[i] = atam.mData.A;
                distCoeffs[i] = atam.mData.D;
                kf.pose.rvec.copyTo(R[i]);
                kf.pose.tvec.copyTo(T[i]);
            }
            
            LOGOUT("BA started with %d points %d frames\n", int(vPt3d.size()), numKeyframes);
            
            double val = mBA.run(vUsedPt3d, imagePoints, visibility, cameraMatrix, R, T, distCoeffs);
            
            if (val < 0) {
                LOGOUT("BA failed\n");
            }
            else {
                LOGOUT("Initial error = \t %f\n", mBA.getInitialReprjError());
                LOGOUT("Final error = \t %f\n", mBA.getFinalReprjError());
                
                for (int i = 0; i < numKeyframes; ++i) {
                    sKeyframe &kf = vKf[i];
                    
                    R[i].copyTo(kf.pose.rvec);
                    T[i].copyTo(kf.pose.tvec);
                }
                
                for (int i = 0, iend = int(vVisibleID.size()); i < iend; ++i) {
                    int id = vVisibleID[i];
                    vPt3d[id] = vUsedPt3d[i];
                }
                
                baData.vVisibleID = vVisibleID;
                
                atam.mData.map.CopyfromBA(baData);
            }
            
            mBAMutex.lock();
            mDoingBA = false;
            mBAMutex.unlock();
        }
    }
    else if (mainState == STATE::CLOSE) {
        return;
    }
    
}
