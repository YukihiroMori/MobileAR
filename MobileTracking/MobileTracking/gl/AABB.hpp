//
//  AABB.h
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef AABB_hpp
#define AABB_hpp

#include "GLMain.hpp"

class AABB {
public:
    AABB() : min(vec3(FLT_MAX)), max(vec3(-FLT_MAX)) {}
    
    vec3 min;
    vec3 max;
};

#endif /* AABB_h */
