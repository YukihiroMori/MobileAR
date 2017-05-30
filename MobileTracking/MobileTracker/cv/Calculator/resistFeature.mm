//
//  resistFeature.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/03/24.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "resistFeature.hpp"

void resistFeature(vec2 size, vector<DMatch> matches,vector<KeyPoint> k1, vector<KeyPoint> k2, vector<vec2> &point1, vector<vec2> &point2){
    vector<vec2> p1list;
    vector<vec2> p2list;
    
    vec2 Estimate = vec2(400);
    
    for (int i = 0; i < matches.size(); i++){
        
        vec2 p1 = vec2(k1[matches[i].queryIdx].pt.x,k1[matches[i].queryIdx].pt.y);
        vec2 p2 = vec2(k2[matches[i].trainIdx].pt.x,k2[matches[i].trainIdx].pt.y);
        
        p1 = (p1 - size / 2.0f) / size * Estimate;
        p2 = (p2 - size / 2.0f) / size * Estimate;
        
        p1list.push_back(p1);
        p2list.push_back(p2);
    }
    
    point1 = p1list;
    point2 = p2list;
}
