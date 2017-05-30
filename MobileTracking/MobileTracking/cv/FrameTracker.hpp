//
//  FrameTracker.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/24.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef FrameTracker_hpp
#define FrameTracker_hpp

#import <opencv2/opencv.hpp>
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/features2d.hpp>
#import <opencv2/xfeatures2d.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <map>
#include <math.h>
#include <Eigen/Dense>
#include "FMatrixComputation.hpp"
#include "TwoViewReconstruction.hpp"
#include "MultipleViewTriangulation.hpp"
#include "Utils.hpp"

using namespace std;
using namespace cv;
using namespace glm;
using namespace Eigen;

class Frame{
    const int CORNER_MAX = 200;
    const double QUARITY_LEVEL = 0.3;
    const double MIN_DISTANCE = 8.0;
    const double BLOCK_SIZE = 8.0;
    
    const int min_limit = 7;
public:
    const int FRAME_SIZE = 360;
    
    Mat image;
    Mat gray;
    vector<Point2f> corners;
    
    bool isCalc = false;
    
    void initFrame(Mat frame){
        //resize(frame, image, cv::Size(FRAME_SIZE, FRAME_SIZE), 0, 0, INTER_LINEAR);
        cv::Rect rect;
        rect.x = (frame.size().width - FRAME_SIZE) / 2.0;
        rect.y = (frame.size().height - FRAME_SIZE) / 2.0;
        rect.width = FRAME_SIZE;
        rect.height = FRAME_SIZE;
        Mat trim(frame, rect);
        image = trim;
    }
    
    void Compute(){
        if(image.empty())
            return;
        
        cvtColor(image, gray, CV_RGBA2GRAY);
        goodFeaturesToTrack(gray, corners, CORNER_MAX, QUARITY_LEVEL, MIN_DISTANCE,
                            Mat(), BLOCK_SIZE);
        cornerSubPix(gray, corners, cv::Size(21, 21), cv::Size(-1, -1), TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 30, 0.01));
        
        if(corners.size() < min_limit)
            return;
        
        isCalc = true;
    }
    
    Mat getPreview(Mat &preview){
        for(int i = 0;i < corners.size(); i++){
            circle(preview, corners[i] , 3, Scalar(255,0,0,255), 1, CV_AA);
        }
        
        return preview;
    }
    
    void clear(){
        isCalc = false;
        image.release();
        gray.release();
        corners.clear();
    }
};

class P {

public:
    shared_ptr<Frame> frame;
    
    Point2f point;
    
    Vector3d position;
    
    int referense = 0;
    int died = 0;
    
    P clone(){
        P p;
        p.frame = frame;
        p.position = position;
        p.point = point;
        return p;
    }
};

class Map3D{
public:
    vector<P> Map;
    
    void push(vector<P> plist){
        
        copy(plist.begin(),plist.end(),back_inserter(Map));
    }
    
    void ProjectMat(Vector3d t , Matrix3d R, vector<P> &pList){
        if(Map.empty())
            return;
        
        int x = Map.size();
        
        for(int i = 0; i < x; i++){
            P p = Map[i];
            
            Matrix4d LocalModel;
            LocalModel.topLeftCorner<3,3>() = R;
            LocalModel.topRightCorner<3,1>() = Vector3d::Zero(3);
            LocalModel(3,3) = 1.0f;
            
            Matrix4d Model;
            Model.topLeftCorner<3,3>() = Matrix3d::Identity(3,3).transpose();
            Model.topRightCorner<3,1>() = t.transpose();
            Model(3,3) = 1.0f;
            
            Vector4d pp;
            pp << p.position(0) , p.position(1) , p.position(2) , 1.0;
            
            Vector4d t = pp.transpose() * LocalModel;
            Vector4d vp = Model * t;
            
            double tanHalfFovy = tan(2.70531270047 / 2.0);
            
            Matrix4d P;
            P = Matrix4d::Zero(4,4);
            P(0,0) = 1.0 / (1.0 * tanHalfFovy);
            P(1,1) = 1.0 / (tanHalfFovy);
            P(2,3) = -1.0;
            
            //P(2,2) = 100 / (0 - 100);
            //P(3,2) = -(100 * 0) / (100 - 0);
            P(2,2) = - (100 + 0) / (100 - 0);
            P(3,2) = - (2.0 * 100 * 0) / (100 - 0);
            
            Vector4d sp = P * vp;
            
            //cout << "x" << sp(0) << " y" << sp(1) << " z" << sp(2) << " w" << sp(3) << endl;
            
        }
        
        pList = Map;
    }
};

class MatchResult{
public:
    int Num;
    
    vector<Point2f> point1;
    vector<Point2f> point2;
    
    void clear(){
        Num = 0;
        point1.clear();
        point2.clear();
    }
};


class FrameMatch{
    vector<uchar> Status;
    vector<float> Errors;
    
public:
    void Match(Frame frame1, Frame frame2, MatchResult &result){
        result.clear();
        
        calcOpticalFlowPyrLK(frame1.gray, frame2.gray, frame1.corners, frame2.corners, Status, Errors);
        
        int cnt = 0;
        for( int i = 0; i < Status.size(); i++ ){
            if(Status[i] == 1){
                result.point1.push_back(frame1.corners[i]);
                result.point2.push_back(frame2.corners[i]);
                cnt++;
            }
        }
        
        result.Num = cnt;
    }
};

class DataConverter{
public:
    void Convert(MatchResult result, vector<Vector2d> &a, vector<Vector2d> &b){
        
        vector<Point2f> p1;
        vector<Point2f> p2;
        
        for(int i = 0; i < result.Num; i++){
            p1.push_back(result.point1[i] - Point2f(180.0, 180.0));
            p2.push_back(result.point2[i] - Point2f(180.0, 180.0));
        }
        
        int Num = result.Num;
        
        vector<Vector2d> pos0 = vector<Vector2d>(Num);
        vector<Vector2d> pos1 = vector<Vector2d>(Num);
        for (int al = 0; al < Num; al++)
        {
            pos0[al] << p1[al].x, p1[al].y;
            pos1[al] << p2[al].x, p2[al].y;
        }
        
        a = pos0;
        b = pos1;
    }
};

class TrackingResult{
public:
    
    
    Vector9d theta;
    Matrix3d R;
    Vector3d t;
};


class FrameTracker{
    Frame start;
    Frame end;
    
    FrameMatch match;
    
    shared_ptr<Frame> prev;
    Frame current;
    
    DataConverter conv;
    
    vec3 dp;
    vec3 dr;
    
    Map3D map;
    
    vector<Frame> list;
    
    MatchResult result;
    
    int mnFrame = 0;
    int mnLastKeyFrameDropped = -20;
    
public:
    bool Initialized;
    
    FrameTracker(){
        Initialized = false;
    }
    
    void Tracking(Mat image,bool &tracked, Vector3d &pos , Matrix3d &rot){
        current.clear();
        
        if(prev == nullptr){
            return;
        }
        
        current.initFrame(image);
        current.Compute();
        
        if(!current.isCalc || !prev->isCalc){
            return;
        }
        
        match.Match(current, *prev, result);
        
        if( result.Num < 20){
            return;
        }
        
        vector<Vector2d> pos1;
        vector<Vector2d> pos2;
        Vector9d theta;
        int Num = result.Num;
        
        conv.Convert(result , pos1, pos2);
        
        if(!FMatrixComputation::ransac(pos1, pos2, Num, theta)) return;
        
        double nfl0, nfl1;
        
        // focal length computation
        //if(!TwoViewReconstruction::focal_length_computation(theta, &nfl0, &nfl1)) return;
        
        nfl0 = 407;//288
        nfl1 = 407;//288
        
        // motion parameters
        Matrix3d R;
        Vector3d t;
        
        if(!TwoViewReconstruction::motion_parameter_computation(theta, nfl0, nfl1, pos1, pos2, Num, R, t)) return;
        
        // 3-D reconstruction
        //vector<Vector3d> X(Num);
        //if(!TwoViewReconstruction::reconstruction(pos1, pos2, Num, theta, R, t, nfl0, nfl1, X)) return;
        
        pos = t;
        rot = R;
        tracked = true;
        
        prev = make_shared<Frame>(current);
        
        mnFrame++;
        if(mnFrame - mnLastKeyFrameDropped > 20){
            list.push_back(current);
            mnLastKeyFrameDropped = mnFrame;
        }
        
        
        if(list.size() > cam_num){
            MakeMap(list);
            
            list.clear();
        }
    }
    
    void init_start(Mat image){
        start.initFrame(image);
        start.Compute();
    }
    
    void init_end(Mat image){
        end.initFrame(image);
        end.Compute();
        
        if(!start.isCalc || !end.isCalc){
            return;
        }
        
        match.Match(start, end, result);
            
        DataConverter conv;
        vector<Vector2d> pos1;
        vector<Vector2d> pos2;
        Vector9d theta;
        
        int Num = result.Num;
        
        conv.Convert(result , pos1, pos2);
            
        double nfl0, nfl1;
        // motion parameters
        Matrix3d R;
        Vector3d t;
        // 3-D reconstruction
        vector<Vector3d> X(Num);
        
        if(!FMatrixComputation::ransac(pos1, pos2, Num, theta)) return;
            
        // focal length computation
        //if(!TwoViewReconstruction::focal_length_computation(theta, &nfl0, &nfl1)) return;
        
        nfl0 = 407;//288
        nfl1 = 407;//288
        
        if(!TwoViewReconstruction::motion_parameter_computation(theta, nfl0, nfl1, pos1, pos2, Num, R, t)) return;
            
        //if(!TwoViewReconstruction::reconstruction(pos1, pos2, Num, theta, R, t, nfl0, nfl1, X)) return;
        
        list.push_back(start);
        prev = make_shared<Frame>(end);
        list.push_back(end);
        mnFrame += 2;
        
        Initialized = true;
    }
    
    void hasPoints(Point2f p, vector<Point2f> plist, int &pid){
        pid = -1;
        for(int i = 0; i < plist.size(); i++){
            if(abs(p.x - plist[i].x) < 0.5 && abs(p.y - plist[i].y) < 0.5){
                pid = i;
            }
        }
    }
    
    int cam_num = 3;
    vector<int> ids;
    
    void renb(int &c, int &id, vector<MatchResult> res){
        if(c + 1 == res.size()){
            return;
        }
        if(id < 0){
            return;
        }
        
        hasPoints(res[c].point2[id], res[c+1].point1 ,id);
        ids.push_back(id);
        c++;
        renb(c, id, res);
    }
    
    void MakeMap(vector<Frame> frames){
        dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
            vector<MatchResult> result(cam_num);
            vector<vector<Point2f>> ptr(cam_num);
            
            for(int c = 0; c < cam_num; c++){
                MatchResult res;
                match.Match(frames[c], frames[c+1], res);
                result[c] = res;
            }
            
            for(int id = 0; id < result[0].point1.size(); id++){
                int t = id;
                int c = 0;
                ids.clear();
                
                ids.push_back(t);
                renb(c, t, result);
                
                if(ids.size() == cam_num){
                    for(int c = 0; c < ids.size(); c++){
                        ptr[c].push_back(result[c].point1[ids[c]]);
                    }
                }
            }
            
            if(ptr[0].size() == 0){
                cout << "failed";
                return;
            }
            
            Matrix34d    P, T;
            DiagonalMatrix<double,3> A(360, 360, 407);
            
            // Projection (camera) matrices
            T.block<3,3>(0,0) = Matrix3d::Identity();
            T.block<3,1>(0,3) = Vector3d::Zero();
            P = A * T;
            
            int CamNumAll = cam_num;
            
            int PtNum = ptr[0].size();
            vector<Matrix34d> Proj(CamNumAll);
            vector<MatrixXd> pt(PtNum), ptc(PtNum);
            vector<double> rerr(PtNum);
            vector<Vector3d> rp(PtNum);
            MatrixXi idx(PtNum,CamNumAll);
            
            for(int i = 0; i < CamNumAll; i++){
                Proj[i] = P;
            }
            
            for (int i = 0; i < PtNum; i++)
            {
                pt[i].resize(2,CamNumAll);
                ptc[i].resize(2,CamNumAll);
            }
            
            for (int p = 0; p < PtNum; p++)
            {
                for (int cm = 0; cm < CamNumAll; cm++)
                {
                    Vector2d pos;
                    vector<Point2f> pv;
                    
                    pv = ptr[cm];
                    
                    pos(0) = pv[p].x;
                    pos(1) = pv[p].y;
                    
                    pt[p].col(cm) = pos;
                }
            }
            
            
            // optimal correction and triangulation
            MultipleViewTriangulation::optimal_correction_all(Proj, CamNumAll,
                                                              pt, ptc, idx,
                                                              &rerr[0], PtNum);
            
            MultipleViewTriangulation::triangulation_all(Proj, CamNumAll,
                                                         ptc, rp, PtNum,
                                                         idx);
            
            for(int i = 0; i < rp.size(); i++){
                cout << rp[i](0) << " " << rp[i](1) << " " << rp[i](2) << endl;
            }
        });
        
        }
    
    Mat getStartPreview(){
        return start.getPreview(start.image);
    }
    
    Mat getEndPreview(){
        return end.getPreview(end.image);
    }
    
};





















#endif /* FrameTracker_hpp */
