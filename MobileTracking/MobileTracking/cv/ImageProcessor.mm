//
//  ImageProcessor.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/17.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "ImageProcessor.hpp"
#include <iomanip>

UIImage* ImageProcessor::processing(UIImage *image){
    Mat preview;
    UIImageToMat(image, preview);
    current = preview.clone();
    area.x = current.size().width;
    area.y = current.size().height;
    
    if(selectObject){
        drawPreview(preview);
    }
    
    if(state == NotInitialized){
        CALC_START()
        
        {
            stringstream state;
            state << "RecogValue : " << endl;
            State(state.str())
        }
        //CALC_END("Tracking Time :")
        CALC_ENDV2("Tracking Time :")
        
        stringstream inf;
        inf << "IsTracked : " <<  (state != NotInitialized? "true" : "false") << endl;
        Infomation(inf.str())
    }
    
    if(state == Tracking){
        bool isTracked = false;
        Vector3d t;
        Matrix3d R;
        
        CALC_START()
        {
            lock_guard<mutex> lock(loc);
            ft->Tracking(current, isTracked, t, R);
        }
        
        if(isTracked){
            
            vec3 rot;
            
            if(R(2,1) == 1.0){
                rot.x = asin(R(2,1));
                rot.y = 0.0;
                rot.z = atan(R(1,0),R(0,0));
            } else if(R(2,1) == -1.0){
                rot.x = -asin(R(2,1));
                rot.y = 0.0;
                rot.z = atan(R(1,0),R(0,0));
            } else {
                rot.x = asin(R(2,1));
                rot.y = atan(-R(0,1),R(1,1));
                rot.z = atan(-R(2,0),R(2,2));
            }
            
            rot *= 180.0 / M_PI;
            
            //cout << fixed << setprecision(2) << "x: " << t(0) << "y: " << t(1) << "z: " << t(2)<< endl;
            //cout << "Rx: " << rot.x << "Ry: " << rot.y << "Rz: " << rot.z << endl;
            
            Vector3d v = t.transpose() * Rot;
            
            Pos += v;
            Rot *= R;
            
            if(Rot(2,1) == 1.0){
                rot.x = asin(Rot(2,1));
                rot.y = 0.0;
                rot.z = atan(Rot(1,0),Rot(0,0));
            } else if(Rot(2,1) == -1.0){
                rot.x = -asin(Rot(2,1));
                rot.y = 0.0;
                rot.z = atan(Rot(1,0),Rot(0,0));
            } else {
                rot.x = asin(Rot(2,1));
                rot.y = atan(-Rot(0,1),Rot(1,1));
                rot.z = atan(-Rot(2,0),Rot(2,2));
            }
            
            rot *= 180.0 / M_PI;
            
            stringstream inf;
            inf << fixed << setprecision(2) << "x: " << Pos(0) << "y: " << Pos(1) << "z: " << Pos(2) << endl << "Rx: " << rot.x << "Ry: " << rot.y << "Rz: " << rot.z << endl;
            Infomation(inf.str())
        }
        
        stringstream state;
        state << "IsTracked : " <<  (isTracked? "true" : "false") << endl;
        State(state.str())
        
        CALC_ENDV2("Reconstruction Time :")
    }

    return MatToUIImage(preview);
}

void ImageProcessor::drawPreview(Mat result){
    vec2 pos = trackpos * area;
    
    if(isArea(area, pos, trackSize)){
        rectangle(result,cv::Point(pos.x - trackSize.x/2.0f,pos.y - trackSize.y/2.0f), cv::Point(pos.x + trackSize.x/2.0f, pos.y + trackSize.y/2.0f), cv::Scalar(0,0,255,255), 1, CV_AA);
    }
}

ImageProcessor::ImageProcessor(){
    {
        lock_guard<mutex> lock(loc);
        ft = auto_ptr<FrameTracker>(new FrameTracker());
    }
}

void ImageProcessor::touchStart(vec2 pos){
    selectObject = true;
    if(!current.empty()){
        Mat preview;
        
        {
            lock_guard<mutex> lock(loc);
            ft->init_start(current);
            
            preview = ft->getStartPreview().clone();
        }
        
        
        dispatch_async(dispatch_get_main_queue(), ^{
            leftImageView.image = MatToUIImage(preview);
        });
        
    }
}

void ImageProcessor::touchMove(vec2 pos){
    trackpos = pos;
}


void ImageProcessor::touchEnd(vec2 pos){
    selectObject = false;
    
    if(!current.empty()){
        Mat lpreview;
        Mat rpreview;
        
        CALC_START()
        {
            lock_guard<mutex> lock(loc);
            ft->init_end(current);
            
            lpreview = ft->getStartPreview().clone();
            rpreview = ft->getEndPreview().clone();
            
            state = Tracking;
        }
        CALC_ENDV2("Reconstruction Time :")
        
        
        dispatch_async(dispatch_get_main_queue(), ^{
            leftImageView.image = MatToUIImage(lpreview);
        });
        
        dispatch_async(dispatch_get_main_queue(), ^{
            rightImageView.image = MatToUIImage(rpreview);
        });
        
        
    }
}
