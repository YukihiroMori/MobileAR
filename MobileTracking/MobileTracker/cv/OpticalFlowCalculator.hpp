//
//  OpticalFlowCalculator.hpp
//  MobilTracker
//
//  Created by 森 幸浩 on 2017/04/03.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef OpticalFlowCalculator_hpp
#define OpticalFlowCalculator_hpp

#import <opencv2/opencv.hpp>
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/features2d.hpp>
#import <opencv2/xfeatures2d.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <chrono>

using namespace std;
using namespace cv;
using namespace glm;

class OpticalFlowCalculator {
    const int CORNER_MAX = 200;
    const double QUARITY_LEVEL = 0.3;
    const double MIN_DISTANCE = 7.0;
    const double BLOCK_SIZE = 7.0;
    
    Mat prevFrame;
    Mat prevGray;
    vector<Point2f> prevCorners;
    
    Mat currFrame;
    Mat currGray;
    vector<Point2f> currCorners;
    vector<Point2f> diffLength;
    vector<chrono::system_clock::time_point> spawnTime;
    
    vector<uchar> Status;
    vector<float> Errors;
    
    double life_time = 2000.0;
    double interval = 1000;
    double maxTrackLength = 50.0;
    int maxCount = 200;
    int minCount = 30;
    
    chrono::system_clock::time_point lastAppend;
    
public:
    
    void initFrame(Mat frame){
        Mat prevFrame = frame;
        cvtColor(prevFrame, prevGray, CV_RGB2GRAY);
        
        goodFeaturesToTrack(prevGray, prevCorners, CORNER_MAX, QUARITY_LEVEL, MIN_DISTANCE,
                            Mat(), BLOCK_SIZE);
        
        cornerSubPix(prevGray, prevCorners, cv::Size(21, 21), cv::Size(-1, -1), TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 30, 0.01));
    }
    
    float length_( cv::Point2f &v ) {
        float ll = (float)(v.x * v.x + v.y * v.y);
        float l = (float)sqrt(ll);
        return l;
    }
    
    bool isTracked(unsigned int &idx)
    {
        if (!Status[idx]) return false;
        std::chrono::duration<double> life = chrono::system_clock::now() - spawnTime[idx];
        if (life.count() > life_time) return false;
        if (length_(diffLength[idx]) > maxTrackLength) return false;
        return true;
    }
    
    void appendFeatures(const cv::Mat &img)
    {
        if (img.empty()) return;
        
        Mat target_img = img;
        if (target_img.type() == CV_8UC4) {
            cvtColor(target_img, target_img, CV_BGR2GRAY);
        }
        
        vector<cv::Point2f> results;
        goodFeaturesToTrack(target_img, results, maxCount, QUARITY_LEVEL, MIN_DISTANCE,
                                Mat(), BLOCK_SIZE);
        
        currCorners.insert(currCorners.end(), results.begin(), results.end());
        
        chrono::system_clock::time_point t = chrono::system_clock::now();
        for (unsigned int i = 0; i < results.size(); ++i) {
            spawnTime.push_back(t);
        };
        
        diffLength.resize(spawnTime.size());
        
        lastAppend = chrono::system_clock::now();
    }
    
    void CalcOpt(Mat curr){
        Mat target_img = curr;
        if (target_img.type() == CV_8UC4) {
            cvtColor(target_img, target_img, CV_BGR2GRAY);
        }
        
        int count = 0;
        for (unsigned int i = 0; i < Status.size(); ++i) {
            if (isTracked(i)) {
                currCorners[count] = currCorners[i];
                spawnTime[count] = spawnTime[i];
                count++;
            }
        }
        currCorners.resize(count);
        diffLength.resize(count);
        spawnTime.resize(count);
        
        std::chrono::duration<double> dt = chrono::system_clock::now() - lastAppend;
        if ((int)currCorners.size() < minCount || dt.count() >= interval) {
            appendFeatures(target_img);
        }
        
        prevCorners = currCorners;
        currCorners.clear();
        
        currFrame.copyTo(prevFrame);
        currFrame = curr;
        if (prevFrame.empty()) prevFrame = curr;
        
        calcOpticalFlowPyrLK(prevFrame, currFrame, prevCorners, currCorners, Status, Errors);
        
        for (unsigned int i = 0; i < currCorners.size(); ++i) {
            diffLength[i].x = currCorners[i].x - prevCorners[i].x;
            diffLength[i].y = currCorners[i].y - prevCorners[i].y;
        }
    }
    
    void prevOpt(Mat &preview){
        for (int i = 0; i < Status.size(); i++) {
            cv::Point p1 = cv::Point((int) prevCorners[i].x, (int) prevCorners[i].y);
            cv::Point p2 = cv::Point((int) currCorners[i].x, (int) currCorners[i].y);
            line(preview, p1, p2, Scalar(255, 0, 0, 255), 1 , CV_AA);
            circle(preview, p2, 2, Scalar(255, 0, 0, 255), CV_FILLED);
        }
    }
    
    void clear(){
        prevFrame.release();
        currFrame.release();
        prevCorners.clear();
        currCorners.clear();
        diffLength.clear();
        spawnTime.clear();
        Status.clear();
        Errors.clear();
        lastAppend = chrono::system_clock::now();
    }
    
    unsigned int size()
    {
        return currCorners.size();
    }
    
    bool isInner(unsigned int &idx,cv::Rect &rect)
    {
        if (idx < 0 || size() <= idx) return false;
        return rect.contains(prevCorners[idx]);
    }
    
    cv::Point2f getFlow(cv::Rect rect)
    {
        cv::Point2f max_diff;
        float max_diff_length = 0.0f;
        
        if (size() == 0) return cv::Point2f(0.0f, 0.0f);
        
        for (unsigned i = 0; i < size(); ++i) {
            if (isTracked(i) == true && isInner(i, rect) == true) {
                float diff_length = length_(diffLength[i]);
                if (max_diff_length < diff_length) {
                    max_diff = diffLength[i];
                    max_diff_length = diff_length;
                }
            }
        }
        
        return max_diff;
    }
    
};

#endif /* OpticalFlowCalculator_hpp */
