//
//  Transform.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "Transform.hpp"

mat4 gl::Transform::asMatrix() const {
    mat4 ModelMatrix;
    mat4 Transform = translate(position);
    mat4 Scale = glm::scale(scale);
    mat4 Rotation = mat4_cast(rotation);
    
    ModelMatrix = Transform * Rotation * Scale;
    
    return ModelMatrix;
}
