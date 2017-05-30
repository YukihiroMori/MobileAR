//
//  ATAM.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/07.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "ATAM.hpp"
#include "Timer.hpp"

#include <numeric>
#include <fstream>

#import <UIKit/UIKit.h>

mutex loc;

extern sATAMParams PARAMS;

CATAM::CATAM()
{
    mReflesh = false;
    relocalfailed = 0;
    reset();
}

void CATAM::Start(string path)
{
    if (!init(path)) {
        return;
    }
}

void CATAM::Stop(void){
    mState = STATE::CLOSE;
}

bool CATAM::init(string path)
{
    PARAMS.loadParams(path + "/params.xml");
    
    if (!mCam.LoadParameters(path + PARAMS.CAMERANAME)) {
        printf("Cannot open %s\n", PARAMS.CAMERANAME.c_str());
        return false;
    }
    
    mCam.A.copyTo(mData.A);
    mCam.D.copyTo(mData.D);
    mData.focal = mCam.A.at<double>(0, 0);
    
    loadChallenge(path + "/challenge.txt");
    
    mDetector.Init(PARAMS.MAXPTS, PARAMS.LEVEL);
    
    return true;
}

void CATAM::loop(cv::Mat img, cv::Mat &prev){
    CTimer timer;
    
    timer.Push(__FUNCTION__);
    
    mImg = img;
    cvtColor(mImg, mGImg, cv::COLOR_BGR2GRAY);
    
    process();
    
    double duration = double(timer.Pop());
    mFPS = 1.0 / duration * 1000.0;
    LOGOUT("--frame %d--\n", mFrameNumber);
    ++mFrameNumber;
    
    preview.draw(*this, prev);
}

void CATAM::process()
{
    mData.vKpt.clear();
    mDetector.Detect(mGImg, mData.vKpt);
    
    switch (mState) {
        case STATE::INIT:
            whileInitialize();
            break;
        case STATE::TAM:
            trackAndMap();
            break;
        case STATE::RELOCAL:
            relocalize();
            break;
        default:
            break;
    }
    
    mGImg.copyTo(mData.prevGImg);
}

void CATAM::startInit(void)
{
    mPose.rvec.setTo(0);
    mPose.tvec.setTo(0);
    
    setKeyframe();
    
    mState = STATE::INIT;
    preview.mText = "Translate camera and press space";
}

void CATAM::startTAM(void)
{
    if (makeMap()) {
        
        setKeyframe();
        
        mState = STATE::TAM;
        preview.mText = "Capture calibration board and press space";
    }
}

void CATAM::changeState(void)
{
    if (mState == STATE::STOP) {
        startInit();
    }
    else if (mState == STATE::INIT) {
        startTAM();
    }
    else if (mState == STATE::TAM) {
        registerWorld();
    }
}


void CATAM::reset(void)
{
    mState = STATE::STOP;
    
    mFrameNumber = 0;
    mFPS = 0.0;
    mChallengeNumber = 0;
    
    bool tmp = true;
    while (tmp) {
        tmp = BA.getDoing();
    }
    
    mData.clear();
    
    preview.mText = "Press space to start";
    LOGOUT("-------------RESET-------------\n");
}


bool CATAM::setKeyframe(void)
{
    mData.clearTrack(NOID);
    
    sKeyframe tmpKf;
    tmpKf.pose = mPose;
    
    for (std::list<sTrack>::iterator it = mData.vTrack.begin(),
         itend = mData.vTrack.end(); it != itend; ++it) {
        tmpKf.vPt.push_back(it->vPt.back());
        tmpKf.vPtID.push_back(it->ptID);
    }
    
    std::vector<int> vNewptID;
    std::vector<cv::KeyPoint> &vKpt = mData.vKpt;
    
    for (int i = 0, iend = int(vKpt.size()); i < iend; ++i) {
        
        bool foundSame = false;
        double minDist = PARAMS.PROJERR;
        int ID = NOID;
        
        for (std::list<sTrack>::iterator it = mData.vTrack.begin(),
             itend = mData.vTrack.end(); it != itend; ++it) {
            
            if (it->ptID != NOID) {
                
                cv::Point2f &pt = it->vPt.back();
                
                double dist = cv::norm(vKpt[i].pt - pt);
                
                if (dist < minDist) {
                    ID = it->ptID;
                    minDist = dist;
                    foundSame = true;
                }
            }
        }
        
        if (!foundSame) {
            vNewptID.push_back(i);
        }
        else {
            tmpKf.vKpt.push_back(vKpt[i]);
            tmpKf.vKptID.push_back(ID);
        }
    }
    
    if (int(vNewptID.size()) < PARAMS.MINPTS) {
        return false;
    }
    
    mGImg.copyTo(tmpKf.img);
    cv::Mat vDesc;
    if (tmpKf.vKpt.size() != 0) {
        mDetector.Describe(tmpKf.img, tmpKf.vKpt, tmpKf.vDesc);
    }
    mData.map.AddKeyframe(tmpKf);
    
    for (int i = 0, iend = int(vNewptID.size()); i < iend; ++i) {
        sTrack tmpTrack;
        tmpTrack.kpt = vKpt[vNewptID[i]];
        tmpTrack.ptID = NOID;
        mData.addTrack(tmpTrack);
    }
    
    return true;
}

bool CATAM::checkInsideImage(const cv::Point2f &pt) const
{
    int space = PARAMS.PATCHSIZE * 2;
    
    if (pt.x < space
        || mGImg.cols - space < pt.x
        || pt.y < space
        || mGImg.rows - space < pt.y) {
        return false;
    }
    else {
        return true;
    }
}


int CATAM::trackFrame(void)
{
    std::vector<cv::Point2f> vTracked;
    std::vector<unsigned char> vStatus;
    std::vector<float> vError;
    const cv::Size patch(PARAMS.PATCHSIZE, PARAMS.PATCHSIZE);
    cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10, 0.1);
    
    cv::calcOpticalFlowPyrLK(mData.prevGImg, mGImg, mData.vPrevPt, vTracked, vStatus, vError, patch);
    
    int count = 0;
    mData.vPrevPt.clear();
    
    std::list<sTrack>::iterator it = mData.vTrack.begin();
    for (size_t i = 0; i < vStatus.size(); ++i) {
        if (!vStatus[i] || !checkInsideImage(vTracked[i])) {
            it = mData.vTrack.erase(it);
        }
        else {
            mData.vPrevPt.push_back(vTracked[i]);
            it->vPt.push_back(vTracked[i]);
            
            if (it->ptID != NOID) {
                ++count;
            }
            ++it;
        }
    }
    
    return count;
}

bool CATAM::matchKeyframe(void)
{
    std::vector<cv::KeyPoint> &vKpt = mData.vKpt;
    if (vKpt.size() < PARAMS.MINPTS) {
        return false;
    }
    cv::Mat vDesc;
    mDetector.Describe(mGImg, vKpt, vDesc);
    
    sKeyframe kf = mData.map.GetNearestKeyframe(mPose);
    
    if (kf.vKpt.size() < PARAMS.MINPTS) {
        return false;
    }
    
    std::vector<cv::DMatch> vMatch;
    mDetector.Match(vDesc, kf.vDesc, vMatch);
    
    std::vector<cv::Point3f> vPt3d;
    std::vector<cv::Point2f> vPt2d;
    std::vector<int> vID;
    
    for (int i = 0, iend = int(vMatch.size()); i < iend; ++i) {
        if (vMatch[i].distance < PARAMS.DESCDIST) {
            vPt2d.push_back(vKpt[vMatch[i].queryIdx].pt);
            
            int ID = vMatch[i].trainIdx;
            vPt3d.push_back(mData.map.GetPoint(kf.vKptID[ID]));
            vID.push_back(kf.vKptID[ID]);
        }
    }
    
    if (vPt2d.size() > PARAMS.MINPTS) {
        
        sPose tmpPose = mPose;
        const int iteration = 100;
        const double confidence = 0.98;
        std::vector<int> vInliers;
        
        cv::solvePnPRansac(vPt3d, vPt2d, mData.A, mData.D, tmpPose.rvec, tmpPose.tvec, true, iteration, PARAMS.PROJERR, confidence, vInliers);
        
        if (int(vInliers.size()) > PARAMS.MINPTS
            && float(vInliers.size()) / float(vPt2d.size()) > PARAMS.MATCHKEYFRAME) {
            
            int numRecovered = 0;
            for (int i = 0, iend = int(vInliers.size()); i < iend; ++i) {
                
                bool found = false;
                int pos = vInliers[i];
                for (std::list<sTrack>::iterator it = mData.vTrack.begin(),
                     itend = mData.vTrack.end();	it != itend; ++it) {
                    
                    if (it->ptID == vID[pos]) {
                        found = true;
                        break;
                    }
                }
                
                if (!found) {
                    sTrack track;
                    track.kpt.pt = vPt2d[pos];
                    track.ptID = vID[pos];
                    mData.addTrack(track);
                    ++numRecovered;
                }
            }
            
            if (mState == STATE::RELOCAL) {
                mPose = tmpPose;
            }
            
            LOGOUT("Recovered %d points with keyframe %d at %s\n", numRecovered, kf.ID, __FUNCTION__);
            return true;
        }
    }
    
    return false;
}

bool CATAM::computePose(void)
{
    std::vector<cv::Point2f> vPt2d;
    std::vector<cv::Point3f> vPt3d;
    
    for (std::list<sTrack>::iterator it = mData.vTrack.begin(),
         itend = mData.vTrack.end();	it != itend; ++it) {
        if (it->ptID != NOID) {
            vPt2d.push_back(it->vPt.back());
            vPt3d.push_back(mData.map.GetPoint(it->ptID));
        }
    }
    
    cv::solvePnP(vPt3d, vPt2d, mData.A, mData.D, mPose.rvec, mPose.tvec, true);
    
    std::vector< cv::Point2f > vReproPt;
    cv::projectPoints(vPt3d, mPose.rvec, mPose.tvec, mData.A, mData.D, vReproPt);
    
    int numAll = 0;
    int numDiscard = 0;
    
    for (std::list<sTrack>::iterator it = mData.vTrack.begin(),
         itend = mData.vTrack.end();	it != itend; ++it) {
        if (it->ptID != NOID) {
            
            double dist = cv::norm(vReproPt[numAll] - vPt2d[numAll]);
            
            if (dist > PARAMS.PROJERR) {
                it->ptID = DISCARD;
                ++numDiscard;
            }
            ++numAll;
        }
    }
    
    mData.clearTrack(DISCARD);
    LOGOUT("Discarded:%d used:%d at %s\n", numDiscard, numAll - numDiscard, __FUNCTION__);
    
    if (mData.haveScale) {
        transformToWorld(mPose, mWPose);
    }
    
    if (numAll - numDiscard > PARAMS.MINPTS) {	// if enough points not exist
        return true;
    }
    else {
        return false;
    }
}

void CATAM::computePosefromE(
                             const std::vector<cv::Point2f> &vUnPt1,
                             const std::vector<cv::Point2f> &vUnPt2,
                             cv::Mat &rvec,
                             cv::Mat &tvec
                             ) const
{
    double focal = 1.0;
    cv::Point2d pp = cv::Point2d(0, 0);
    int method = cv::LMEDS;
    double prob = 0.99;
    double th = 1.0 / mData.focal;
    
    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(vUnPt1, vUnPt2, focal, pp, method, prob, th, mask);
    
    cv::Mat R;
    cv::recoverPose(E, vUnPt1, vUnPt2, R, tvec, focal, pp, mask);
    
    cv::Rodrigues(R, rvec);
}

void CATAM::triangulate(
                        const std::vector<cv::Point2f> &vUnPt1,
                        const std::vector<cv::Point2f> &vUnPt2,
                        const sPose &pose1,
                        const sPose &pose2,
                        std::vector<cv::Point3f> &vpt3d
                        ) const
{
    cv::Mat R1, R2;
    
    cv::Rodrigues(pose1.rvec, R1);
    cv::Rodrigues(pose2.rvec, R2);
    
    cv::Mat P1(3, 4, R1.type()), P2(3, 4, R2.type());
    R1.copyTo(P1(cv::Rect(0, 0, 3, 3)));
    R2.copyTo(P2(cv::Rect(0, 0, 3, 3)));
    
    pose1.tvec.copyTo(P1(cv::Rect(3, 0, 1, 3)));
    pose2.tvec.copyTo(P2(cv::Rect(3, 0, 1, 3)));
    
    cv::Mat triangulated;
    cv::triangulatePoints(P1, P2, vUnPt1, vUnPt2, triangulated);
    
    vpt3d.resize(vUnPt1.size());
    
    for (int i = 0, iend = int(vUnPt1.size()); i < iend; ++i) {
        
        float x = triangulated.at < float >(0, i);
        float y = triangulated.at < float >(1, i);
        float z = triangulated.at < float >(2, i);
        float w = triangulated.at < float >(3, i);
        
        vpt3d[i].x = x / w;
        vpt3d[i].y = y / w;
        vpt3d[i].z = z / w;

    }
}


bool CATAM::makeMap(void)
{
    std::vector<cv::Point2f> vStart;
    std::vector<cv::Point2f> vEnd;
    
    std::vector<sTrack*> vNewTrack;
    for (std::list<sTrack>::iterator it = mData.vTrack.begin(),
         itend = mData.vTrack.end();	it != itend; ++it) {
        if (it->ptID == NOID) {
            vStart.push_back(it->vPt[0]);
            vEnd.push_back(it->vPt.back());
            vNewTrack.push_back(&(*it));
        }
    }
    
    if (vStart.size() < PARAMS.MINPTS) {
        return false;
    }
    
    std::vector<cv::Point2f> vUndistStart, vUndistEnd;
    cv::undistortPoints(vStart, vUndistStart, mData.A, mData.D);
    cv::undistortPoints(vEnd, vUndistEnd, mData.A, mData.D);
    
    if (mState == STATE::INIT) {
        computePosefromE(vUndistStart, vUndistEnd, mPose.rvec, mPose.tvec);
    }
    
    sKeyframe &lkf = mData.map.GetLastKeyframe();
    
    std::vector<cv::Point3f> vPt3d;
    triangulate(vUndistStart, vUndistEnd, lkf.pose, mPose, vPt3d);
    
    if (mState == STATE::INIT) {
        sPose tmpPose = lkf.pose;
        if (!BA.initialBA(mData, vPt3d, vStart, vEnd, tmpPose, mPose)) {
            return false;
        }
        else {
            lkf.pose = tmpPose;
        }
    }
    
    std::vector<cv::Point2f> vpPt1, vpPt2;
    cv::projectPoints(vPt3d, lkf.pose.rvec, lkf.pose.tvec, mData.A, mData.D, vpPt1);
    cv::projectPoints(vPt3d, mPose.rvec, mPose.tvec, mData.A, mData.D, vpPt2);
    
    int numinliers = 0;
    std::vector<bool> vInlier(vPt3d.size(), false);
    std::vector<sTrack*>::iterator it = vNewTrack.begin();
    
    std::vector<cv::Point3f> vinPt3d;
    std::vector<cv::KeyPoint> vinKpt;
    
    for (int i = 0, iend = int(vPt3d.size()); i < iend; ++i, ++it) {
        
        double dist1 = cv::norm(vStart[i] - vpPt1[i]);
        double dist2 = cv::norm(vEnd[i] - vpPt2[i]);
        
        if (dist1 < PARAMS.PROJERR && dist2 < PARAMS.PROJERR) {
            vInlier[i] = true;
            vinPt3d.push_back(vPt3d[i]);
            vinKpt.push_back((*it)->kpt);
            ++numinliers;
        }
    }
    
    cv::Mat vinDesc;
    if (vinKpt.size() != 0) {
        mDetector.Describe(lkf.img, vinKpt, vinDesc);
    }
    
    std::vector<int> vID;
    mData.map.UpdateLastKeyframe(vinPt3d, vinKpt, vinDesc, vID);
    
    it = vNewTrack.begin();
    int counter = 0;
    for (int i = 0, iend = int(vInlier.size()); i < iend; ++i, ++it) {
        if (vInlier[i]) {
            (*it)->ptID = vID[counter];
            ++counter;
        }
    }
    
    return true;
}

bool CATAM::mappingCriteria(void)
{
    sKeyframe nkf = mData.map.GetNearestKeyframe(mPose);
    
    cv::Mat R, nkfR;
    cv::Rodrigues(mPose.rvec, R);
    cv::Rodrigues(nkf.pose.rvec, nkfR);
    
    cv::Mat pos, mkfPos;
    pos = -R.inv() * mPose.tvec;
    mkfPos = -nkfR.inv() * nkf.pose.tvec;
    
    double distkeyframe = cv::norm(pos - mkfPos);
    cv::Point3f middle = (cv::Point3f(pos) + cv::Point3f(mkfPos)) / 2.0f;
    
    struct dist3D {
        double dist;
        int ID;
        bool operator< (const dist3D &r) const { return dist < r.dist; }
    };
    
    std::vector<dist3D> vDist3D;
    for (std::list<sTrack>::const_iterator it = mData.vTrack.begin(), itend = mData.vTrack.end();
         it != itend; ++it) {
        if (it->ptID != NOID) {
            dist3D tmp;
            tmp.ID = it->ptID;
            tmp.dist = cv::norm(mData.map.GetPoint(it->ptID) - middle);
            vDist3D.push_back(tmp);
        }
    }
    
    std::sort(vDist3D.begin(), vDist3D.end());
    const cv::Point3f &median = mData.map.GetPoint(vDist3D[int(vDist3D.size()) / 2].ID);
    
    double distpoints = cv::norm(median - middle);
    
    if (PARAMS.BASETAN < distkeyframe / distpoints) {
        return true;
    }
    
    return false;
}

void CATAM::mapping(void)
{
    makeMap();
    setKeyframe();
    
    dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
        BA.BA(*this);
    });
}

void CATAM::trackAndMap(void)
{
    bool relocal = false;
    
    int mappedPts = trackFrame();
    
    if (mappedPts < PARAMS.MINPTS) {
        relocal = true;
    }
    else {
        if (!computePose()) {
            relocal = true;
        }
        else {
            matchKeyframe();
            
            bool doMapping = mappingCriteria();
            
            bool tmp = true;
            tmp = BA.getDoing();
            
            if (doMapping && !tmp) {
                mapping();
                
                mReflesh = true;
            }
        }
    }
    
    if (relocal) {
        mData.clearAllTrack();
        LOGOUT("Lost\n");
        preview.mText = "Go back to this view";
        mState = STATE::RELOCAL;
    }
}

void CATAM::whileInitialize(void)
{
    if (mData.vPrevPt.size() > PARAMS.MINPTS) {
        trackFrame();
    }
    else {
        LOGOUT("Initialization failed\n");
        reset();
    }
}


void CATAM::transformToWorld(const sPose &local, sPose &world) const
{
    cv::Mat tmpM;
    local.getM(tmpM);
    
    tmpM *= mData.scale;
    
    cv::Mat M = tmpM * mData.transMat;
    
    cv::Rodrigues(M(cv::Rect(0, 0, 3, 3)), world.rvec);
    M(cv::Rect(3, 0, 1, 3)).copyTo(world.tvec);
}

bool CATAM::getWorldCoordinate(sPose &pose)
{
    bool found = true;// mCalibrator.EstimatePose(mGImg, mData.A, mData.D, pose.rvec, pose.tvec);
    
    pose.rvec = (cv::Mat_<double>(3,1) << 3.1415926535 / 2.0, 3.1415926535 / 2.0, -3.1415926535 / 4.0);
    pose.tvec = (cv::Mat_<double>(3,1) << 0.0, 0.0, 2300.0);
    
    if (!found) {
        LOGOUT("Calibration board not found\n");
    }
    
    return found;
}

void CATAM::registerWorld(void)
{
    sPose world;
    if (getWorldCoordinate(world)) {
        std::pair<sPose, sPose> tmp;
        tmp.first = world;
        tmp.second = mPose;
        mData.vPosePair.push_back(tmp);
        preview.mText = "Translate camera and capture again";
    }
    
    if (mData.vPosePair.size() > 1) {
        
        std::vector<double> vScale;

        for (int i = 0, iend = int(mData.vPosePair.size()) - 1; i < iend; ++i) {
            
            for (int j = i + 1, jend = int(mData.vPosePair.size()); j < jend; ++j) {
                
                cv::Mat Ri, Rj;
                mData.vPosePair[i].second.getR(Ri);
                mData.vPosePair[j].second.getR(Rj);
                
                cv::Mat R = Ri * Rj.inv();
                double numerator = cv::norm(mData.vPosePair[i].first.tvec - R * mData.vPosePair[j].first.tvec);
                double denominator = cv::norm(mData.vPosePair[i].second.tvec - R * mData.vPosePair[j].second.tvec);
                
                vScale.push_back(numerator / denominator);
            }
        }
        
        mData.scale = std::accumulate(vScale.begin(), vScale.end(), 0.0) / double(vScale.size());
        
        cv::Mat transMat = cv::Mat::zeros(cv::Size(4, 4), CV_64F);
        int size = int(mData.vPosePair.size());
        for (int i = 0; i < size; ++i) {
            sPose world = mData.vPosePair[i].first;
            sPose local = mData.vPosePair[i].second;
            local.tvec *= mData.scale;
            
            cv::Mat worldM, localM;
            world.getM(worldM);
            local.getM(localM);
            
            cv::add(transMat.clone(), localM.inv() * worldM, transMat);
        }
        
        cv::Mat X, D, Y;
        cv::SVD::compute(transMat(cv::Rect(0, 0, 3, 3)), D, X, Y);
        mData.transMat(cv::Rect(0, 0, 3, 3)) = (X * Y) / mData.scale;
        
        mData.transMat(cv::Rect(3, 0, 1, 3)) = transMat(cv::Rect(3, 0, 1, 3)) / double(size) / mData.scale;
        
        mData.haveScale = true;
        
        preview.mText = "Capture more for improvement";
    }
}

void CATAM::changeRelocalImage(void)
{
    mData.map.GetRandomKeyFramePose(mPose);
}

void CATAM::relocalize(void)
{
    if (mData.vKpt.size() == 0) {
        return;
    }
    else if (matchKeyframe()) {
        if (mData.haveScale) {
            preview.mText = "Relocalized";
        }
        else {
            preview.mText = "Capture calibration board and press space";
        }
        
        LOGOUT("Relocalized\n");
        mState = STATE::TAM;
    } else {
        relocalfailed++;
        if(relocalfailed > 3){
            changeRelocalImage();
            relocalfailed = 0;
        }
    }
}


void CATAM::loadChallenge(const std::string &name)
{
    std::ifstream in(name);
    
    if (in.is_open()) {
        
        int num;
        in >> num;
        
        for (int i = 0; i < num; ++i) {
            if (!in.eof()) {
                int id;
                cv::Point3f pt;
                in >> id >> pt.x >> pt.y >> pt.z;
                mData.vChallenge.insert(std::pair<int, cv::Point3f>(id, pt));
            }
        }
    }
    else {
        LOGOUT("%s not found\n", name.c_str());
        
        mData.vChallenge.insert(std::pair<int, cv::Point3f>(0, cv::Point3f(0, 0, 0)));
    }
}

bool CATAM::operation(const int key)
{
    if (key == ' ') {
        changeState();
    }
    else if (key == 'r') {
        reset();
    }
    else if (key == 'n') {
        if (mState == STATE::RELOCAL) {
            changeRelocalImage();
        }
        else if (mState == STATE::TAM) {
            mapping();
        }
    }
    else if (key == 'c') {
        ++mChallengeNumber;
    }
    else if (key == 'q') {
        return true;
    }
    
    return false;
}


