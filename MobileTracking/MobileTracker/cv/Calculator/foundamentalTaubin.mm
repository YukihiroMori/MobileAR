//
//  foundamentalTaubin.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/24.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "foundamentalTaubin.hpp"

#define sq(x) ((x) * (x))

Mat fundamentalTaubin(vector<vec2> x1, vector<vec2> x2, int N, double f){
    
    /* {{{ 8 次元ベクトル z の計算 */
    vector<Mat> z(N);
    for(int i = 0; i < N; i++) {
        z[i] = Mat(8, 1, CV_64FC1);
        z[i].at<float>(0 , 0) = x1[i].x * x2[i].x;
        z[i].at<float>(1 , 0) = x1[i].x * x2[i].y;
        z[i].at<float>(2 , 0) = f * x1[i].x;
        z[i].at<float>(3 , 0) = x1[i].y * x2[i].x;
        z[i].at<float>(4 , 0) = x1[i].y * x2[i].y;
        z[i].at<float>(5 , 0) = f * x1[i].y;
        z[i].at<float>(6 , 0) = f * x2[i].x;
        z[i].at<float>(7 , 0) = f * x2[i].y;
    }
    /* }}} */
    
    /* {{{ 8x8 行列 V0 の計算 */
    vector<Mat> V(N);
    for(int i = 0; i < N; i++) {
        V[i] = Mat(8, 8, CV_64FC1);
        V[i].at<float>(0 , 0) = sq(x1[i].x) + sq(x2[i].x);
        V[i].at<float>(0 , 1) = x2[i].x * x2[i].y;
        V[i].at<float>(0 , 2) = f * x2[i].x;
        V[i].at<float>(0 , 3) = x1[i].x * x1[i].y;
        V[i].at<float>(0 , 4) = 0.0;
        V[i].at<float>(0 , 5) = 0.0;
        V[i].at<float>(0 , 6) = f * x1[i].x;
        V[i].at<float>(0 , 7) = 0.0;
        
        V[i].at<float>(1 , 0) = x2[i].x * x2[i].y;
        V[i].at<float>(1 , 1) = sq(x1[i].x) + sq(x2[i].y);
        V[i].at<float>(1 , 2) = f * x2[i].y;
        V[i].at<float>(1 , 3) = 0.0;
        V[i].at<float>(1 , 4) = x1[i].x * x1[i].y;
        V[i].at<float>(1 , 5) = 0.0;
        V[i].at<float>(1 , 6) = 0.0;
        V[i].at<float>(1 , 7) = f * x1[i].x;
        
        V[i].at<float>(2 , 0) = f * x2[i].x;
        V[i].at<float>(2 , 1) = f * x2[i].y;
        V[i].at<float>(2 , 2) = sq(f);
        V[i].at<float>(2 , 3) = 0.0;
        V[i].at<float>(2 , 4) = 0.0;
        V[i].at<float>(2 , 5) = 0.0;
        V[i].at<float>(2 , 6) = 0.0;
        V[i].at<float>(2 , 7) = 0.0;
        
        V[i].at<float>(3 , 0) = x1[i].x * x1[i].y;
        V[i].at<float>(3 , 1) = 0.0;
        V[i].at<float>(3 , 2) = 0.0;
        V[i].at<float>(3 , 3) = sq(x1[i].y) + sq(x2[i].x);
        V[i].at<float>(3 , 4) = x2[i].x * x2[i].y;
        V[i].at<float>(3 , 5) = f * x2[i].x;
        V[i].at<float>(3 , 6) = f * x1[i].y;
        V[i].at<float>(3 , 7) = 0.0;
        
        V[i].at<float>(4 , 0) = 0.0;
        V[i].at<float>(4 , 1) = x1[i].x * x1[i].y;
        V[i].at<float>(4 , 2) = 0.0;
        V[i].at<float>(4 , 3) = x2[i].x * x2[i].y;
        V[i].at<float>(4 , 4) = sq(x1[i].y) + sq(x2[i].y);
        V[i].at<float>(4 , 5) = f * x2[i].y;
        V[i].at<float>(4 , 6) = 0.0;
        V[i].at<float>(4 , 7) = f * x1[i].y;
        
        V[i].at<float>(5 , 0) = 0.0;
        V[i].at<float>(5 , 1) = 0.0;
        V[i].at<float>(5 , 2) = 0.0;
        V[i].at<float>(5 , 3) = f * x2[i].x;
        V[i].at<float>(5 , 4) = f * x2[i].y;
        V[i].at<float>(5 , 5) = sq(f);
        V[i].at<float>(5 , 6) = 0.0;
        V[i].at<float>(5 , 7) = 0.0;
        
        V[i].at<float>(6 , 0) = f * x1[i].x;
        V[i].at<float>(6 , 1) = 0.0;
        V[i].at<float>(6 , 2) = 0.0;
        V[i].at<float>(6 , 3) = f * x1[i].y;
        V[i].at<float>(6 , 4) = 0.0;
        V[i].at<float>(6 , 5) = 0.0;
        V[i].at<float>(6 , 6) = sq(f);
        V[i].at<float>(6 , 7) = 0.0;
        
        V[i].at<float>(7 , 0) = 0.0;
        V[i].at<float>(7 , 1) = f * x1[i].x;
        V[i].at<float>(7 , 2) = 0.0;
        V[i].at<float>(7 , 3) = 0.0;
        V[i].at<float>(7 , 4) = f * x1[i].y;
        V[i].at<float>(7 , 5) = 0.0;
        V[i].at<float>(7 , 6) = 0.0;
        V[i].at<float>(7 , 7) = sq(f);
    }
        /* }}} */
    
    /* {{{ 8 次元ベクトル zt の計算 */
    Mat zb = Mat::zeros(8, 1, CV_64FC1);
    for(int i = 0; i < N; i++) {
        zb = z[i] + zb;
    }
    for(int i = 0; i < 8; i++) {
        zb.at<float>(i , 0) = zb.at<float>(i , 0) / (double) N;
    }
    vector<Mat> zt(N);
    for(int i = 0; i < N; i++) {
        zt[i] = Mat(8, 1, CV_64FC1);
        z[i] = z[i] - zb;
    }
    /* }}} */
    
    /* {{{ 8x8 行列 M と L の計算*/
    Mat M = Mat::zeros(8, 8, CV_64FC1);
    Mat L = Mat::zeros(8, 8, CV_64FC1);
    Mat MTmp = Mat::zeros(8, 8, CV_64FC1);
    
    for(int i = 0; i < N; i++) {
        mulTransposed(zt[i], MTmp, 0);
        M = M + MTmp;
        L = L + V[i];
    }
    /* }}} */
    
    /* {{{ 一般固有ベクトルの計算 */
    SVD svdL = SVD(L);
    
    Mat g = Mat(8, 1, CV_64FC1);
    Mat T = Mat::zeros(8, 8, CV_64FC1);
    Mat TTmp = Mat(8, 8, CV_64FC1);
    Mat IdMat = Mat(8, 8, CV_64FC1);
    
    for(int i = 0; i < 8; i++) {
        for(int j = 0; j < 8; j++) {
            g.at<float>(j, 0) = svdL.u.at<float>(j, i);
        }
        TTmp = g * g.t();
        setIdentity(IdMat, cvRealScalar(1.0 / sqrt(svdL.w.at<float>(i, i))));
        TTmp = IdMat + TTmp;
        T = T + TTmp;
    }
    
    M = T * M;
    M = M * T;
    SVD svdM = SVD(M);
    
    Mat v = Mat(8, 1, CV_64FC1);
    for(int i = 0; i < 8; i++) {
        v.at<float>(i , 0) = svdM.u.at<float>(i , 7);
    }
    v = T * v;
    normalize(v, v);
    
    Mat u = Mat(9, 1, CV_64FC1);
    for(int i = 0; i < 8; i++) {
        u.at<float>(i , 0) = v.at<float>(i, 0);
    }
    u.at<float>(8 , 0)  = - v.dot(zb) / sq(f);
    normalize(u, u);
    
    Mat F = Mat(3, 3, CV_64FC1);
    for(int i = 0; i < 9; i++) {
        F.at<float>(i/3, i%3) = u.at<float>(i, 0);
    }
    /* }}} */
    
    return F;
}
