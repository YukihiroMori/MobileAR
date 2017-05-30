//
//  ImageProcessor.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/17.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "ImageProcessor.hpp"

UIImage* ImageProcessor::processing(UIImage *image){
    Mat preview;
    UIImageToMat(image, preview);
    current = preview.clone();
    area.x = current.size().width;
    area.y = current.size().height;
    
    if(selectObject){
        drawPreview(preview);
    }
    
    vec2 estimate;
    bool isTracked = false;
    
    if(state != NoTrackingObject){
        
        if(state == PF){
            lock_guard<mutex> lock(loc);
            
            PFSearch(estimate, isTracked);
            
            if(isTracked){
                pf->drawResult(preview);
            } else {
                state = TM;
            }
        }
        
        if(state == TM){
            lock_guard<mutex> lock(loc);
            
            TMSearch(estimate, isTracked);
            
            if(isTracked){
                tm->drawResult(preview);
                pf->Redifine(tm->estimate);
                state = PF;
            } else {
                state = NoTracked;
            }
        }
        
        if(state == FM || state == NoTracked){
            lock_guard<mutex> lock(loc);
            
            CALC_START()
            fm->FMSearch(current, estimate, isTracked);
            CALC_ENDV2("Tracking Time :")
            
            fm->drawResult(preview);
            
            if(isTracked){
                state = FM;
            } else {
                state = NoTracked;
            }
        }
        
        if(state == OF){
            lock_guard<mutex> lock(loc);
            
            of->CalcOpt(current);
            of->prevOpt(preview);
            cv::Point2f diff = of->getFlow(trect);
            trect.x += (int)diff.x;
            trect.y += (int)diff.y;
            
            Point2f m;
            m.x = trect.x + trect.width / 2.0;
            m.y = trect.y + trect.height / 2.0;
            
            rectangle(preview, trect, Scalar(255,0,0,255), 2);
            
        }
        
        if(state == QR){
            qr->Tracking(current, preview);
        }
        
        stringstream inf;
        inf << "IsTracked : " <<  (state != NoTracked? "true" : "false") << endl;
        Infomation(inf.str())
    }

    return MatToUIImage(preview);
}

void ImageProcessor::PFSearch(vec2 &position, bool &isTracked){
    CALC_START()
    
    vec2 estimate;
    
    pf->Resampling();
    pf->Predict();
    pf->CalcWeight(current);
    estimate = pf->Measure();
    
    if(isArea(area, estimate, trackSize))
    {
        cv::Rect rt = createRect(estimate, trackSize);
        Mat roi(current, rt);
        double res = pf->cd.calcDistance(roi);
        
        {
            stringstream state;
            state << "RecogValue : " << res << endl;
            State(state.str())
        }
        
        if(res > 0.5){
            isTracked = true;
            position = estimate;
        } else {
            isTracked = false;
        }
    }
    
    //CALC_END("Tracking Time :")
    CALC_ENDV2("Tracking Time :")
}

void ImageProcessor::TMSearch(vec2 &position, bool &isTracked){
    CALC_START()
    
    tm->TM(current);
    
    {
        stringstream state;
        state << "TemplateMathced : " << tm->evaluate << endl;
        State(state.str())
    }
    
    if(tm->evaluate > 0.8){
        isTracked = true;
        position = tm->estimate;
    }
    
    //CALC_END("Tracking Time :")
    CALC_ENDV2("Tracking Time :")
}

void ImageProcessor::drawPreview(Mat result){
    vec2 pos = trackpos * area;
    
    if(isArea(area, pos, trackSize)){
        rectangle(result,cv::Point(pos.x - trackSize.x/2.0f,pos.y - trackSize.y/2.0f), cv::Point(pos.x + trackSize.x/2.0f, pos.y + trackSize.y/2.0f), cv::Scalar(0,0,255,255), 1, CV_AA);
    }
}

ImageProcessor::ImageProcessor(){
    pf = auto_ptr<ParticleFilter>(new ParticleFilter());
    tm = auto_ptr<TemplateMatcher>(new TemplateMatcher());
    fm = auto_ptr<FeatureMatcher>(new FeatureMatcher());
    of = auto_ptr<OpticalFlowCalculator>(new OpticalFlowCalculator());
    qr = auto_ptr<QRFinder>(new QRFinder());

}

void ImageProcessor::touchStart(vec2 pos){
    selectObject = true;
}

void ImageProcessor::touchMove(vec2 pos){
    trackpos = pos;
}


void ImageProcessor::touchEnd(vec2 pos){
    selectObject = false;
    
    if(!current.empty()){
        vec2 pos = trackpos * area;
        
        if(!isArea(area, pos, trackSize)){
            return;
        }
        
        trect = createRect(pos, trackSize);
        Mat roi(current, trect);
        roi.copyTo(searchObject);
        
        dispatch_async(dispatch_get_main_queue(), ^{
            searchImageView.image = MatToUIImage(searchObject);
        });
        
        {
            lock_guard<mutex> lock(loc);
            //pf->Define(pos ,searchObject , area);
            //tm->setTemplate(searchObject);
            
            /*of->initFrame(current);
            state = OF;
            */
            
            /*
            if(!fm->setTarget(searchObject)){
                
                cout << "No Matcher" << endl;
                
                return;
            }
            state = PF;
            */
            
            state = QR;
        }
        
        trackObject = true;
    }
}
