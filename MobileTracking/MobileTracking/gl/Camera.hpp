//
//  Camera.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef Camera_hpp
#define Camera_hpp

#include "GLMain.hpp"

using namespace glm;

class Camera{
    int width;
    int height;
    
public:
    vec3 position;
    vec3 rotation;
    
    mat4 R;
    
    mat4 ProjectionMatrix;
    
    vec3 lookat = vec3(0.0f,0.0f,0.0f);
    
    float fovY;
    float zNear, zFar;
    
    Camera():
    position(dvec3(0.0, 0.0, 0.0))
    , rotation(dvec3(0.0, 0.0, 0.0))
    , fovY(47.7)
    , zNear(1), zFar(3000)
    {
    }
    
    mat4 getViewMatrix();
    
    mat4 getProjectionMatrix();
    
    void setViewPort();
    
    void setSize(int w, int h);
};

#endif /* Camera_hpp */
