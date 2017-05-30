//
//  Transform.h
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef Transform_hpp
#define Transform_hpp

#define GLM_ENABLE_EXPERIMENTAL
#include "GLMain.hpp"

using namespace glm;

namespace gl{
    class Transform{
    public:
        mat4 asMatrix() const;
        
        vec3 position = vec3(0.0);
        quat rotation = quat(vec3(0.0));
        vec3 scale = vec3(1.0);
        
    };
}

#endif /* Transform_h */
