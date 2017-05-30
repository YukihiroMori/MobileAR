//
//  ImageProcessor.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/17.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "ImageProcessor.hpp"
#include <iomanip>
#include "opencv2/opencv.hpp"
#include "Cam.hpp"
#include "Calibration.hpp"

static string path;

const int FRAME_SIZE = 360;

UIImage* ImageProcessor::processing(UIImage *image){
    Mat preview;
    UIImageToMat(image, preview);
    current = preview.clone();
    area.x = current.size().width;
    area.y = current.size().height;
    
    cv::Rect rect;
    rect.x = (current.size().width - FRAME_SIZE) / 2.0;
    rect.y = (current.size().height - FRAME_SIZE) / 2.0;
    rect.width = FRAME_SIZE;
    rect.height = FRAME_SIZE;
    Mat trim(current, rect);
    
    preview = trim.clone();
    
    {
        lock_guard<mutex> lock(loc);
        atam.loop(trim, preview);
    }
    return MatToUIImage(preview);
}

void ImageProcessor::setGyro(double x, double y, double z){
    {
        lock_guard<mutex> lock(loc);
        atam.Gyro.rvec = (Mat_<double>(3,1) << x, y, z);
    }
}

void ImageProcessor::stop(){
    {
        lock_guard<mutex> lock(loc);
        atam.Stop();
    }
}

ImageProcessor::ImageProcessor(){
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *docs = [paths objectAtIndex:0];
    path = [docs UTF8String];
    {
        lock_guard<mutex> lock(loc);
        atam.Start(path);
    }
}

void ImageProcessor::touchStart(vec2 pos){
    {
        lock_guard<mutex> lock(loc);
        atam.operation(' ');
    }
}

void ImageProcessor::touchMove(vec2 pos){
}


void ImageProcessor::touchEnd(vec2 pos){
}

void ImageProcessor::start(){
}

void ImageProcessor::reset(){
    {
        lock_guard<mutex> lock(loc);
        atam.operation('r');
    }
}

void ImageProcessor::N(){
    {
        lock_guard<mutex> lock(loc);
        atam.operation('n');
    }
}

void ImageProcessor::C(){
    {
        lock_guard<mutex> lock(loc);
        atam.operation('c');
    }
}

void ImageProcessor::Q(){
    {
        lock_guard<mutex> lock(loc);
        atam.operation('q');
    }
}

void ImageProcessor::Calibration(){
    if(current.empty()){
        return;
    }
    
    cv::Mat im = current;
    
    CCalibration calib;
    
    cv::Mat detect = im.clone();
    
    if (vimg.size() < 12) {	// save image
        // extract corners
        std::vector< cv::Point2f > tmp;
        cv::Mat gDetect;
        cv::cvtColor(detect, gDetect, cv::COLOR_BGR2GRAY);
        
        bool found = calib.DetectCorners(gDetect, tmp);
        if (found) {
            calib.DrawCorners(detect, tmp);
            vimg.push_back(im.clone());
            printf("image:%02d saved\n", int(vimg.size()));
        }
        
        return;
    }
    
    std::vector< std::vector< cv::Point2f > > imagePoints;
    bool iscalib = true;
    
    // for each image
    for (int i = 0, iend = int(vimg.size()); i < iend; ++i) {
        
        // extract corners
        std::vector< cv::Point2f > tmp;
        cv::Mat gIm;
        cv::cvtColor(vimg[i], gIm, cv::COLOR_BGR2GRAY);
        bool found = calib.DetectCorners(gIm, tmp);
        if (found) {
            imagePoints.push_back(tmp);
            
            calib.DrawCorners(vimg[i], tmp);
        }
    }
    
    if (imagePoints.size() == 0) {
        printf("calibration failed\n");
        iscalib = false;
    }
    
    cv::Mat intrinsic, distortion;
    
    calib.Calibrate(imagePoints, vimg[0].size(), intrinsic, distortion);
    
    if (iscalib) {
        CCam camera;
        camera.A = intrinsic;
        camera.D = distortion;
        
        camera.SaveParameters(path + "/camera");
    }

}
