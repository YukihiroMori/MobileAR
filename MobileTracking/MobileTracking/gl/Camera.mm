//
//  Camera.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "Camera.hpp"
#include <iostream>

using namespace std;

mat4 Camera::getViewMatrix(){
    //mat4 R = mat4(1.0);
    
    //mat4 R = mat4(0.0);
    
    /*
    vec3 E(rotation.x,rotation.y,rotation.z);
    
    quat q(E);
    R *= toMat4(q);
     */
     
    /*
    R[0][0] = cos(y)*cos(z) - sin(x)*sin(y)*sin(z);
    R[0][1] = -cos(x)*sin(z);
    R[0][2] = sin(y)*cos(z) + sin(x)*cos(y)*sin(z);
    R[1][0] = cos(y)*sin(z) + sin(x)*sin(y)*cos(z);
    R[1][1] = cos(x)*cos(z);
    R[1][2] = sin(y)*sin(z) - sin(x)*cos(y)*cos(z);
    R[2][0] = -cos(x)*sin(y);
    R[2][1] = sin(x);
    R[2][2] = cos(x)*cos(y);
    */
    

    mat4 T = mat4(1.0);
    T[3][0] = position.x * width / (float)height;
    T[3][1] = -position.y;
    T[3][2] = -position.z;
    
    return T*inverse(R);
    
    /*
    vec4 viewDirection = vec4(0.0,0.0,1.0,1.0);
    viewDirection = rotate((float)rotation.x, vec3(1.0,0.0,0.0)) * viewDirection;
    viewDirection = rotate((float)rotation.y, vec3(0.0,1.0,0.0)) * viewDirection;
    viewDirection = rotate((float)rotation.z, vec3(0.0,0.0,1.0)) * viewDirection;
    
    mat4 ViewMatrix = lookAt((vec3)position, lookat + viewDirection.xyz + (vec3)position, vec3(0.0, 1.0, 0.0));
    
    return ViewMatrix;
    */
}

mat4 Camera::getProjectionMatrix(){
    
    mat4 ProjectionMatrix;
    ProjectionMatrix = perspective(radians(fovY), (0 != height)? width / (float)height : 10000, zNear, zFar);
    
    //cout << to_string(ProjectionMatrix) << endl;
    
    return ProjectionMatrix;
}

void Camera::setViewPort(){
    ASSERT_GL(glViewport( 0, 0, width, height ));
}

void Camera::setSize(int w, int h){
    width = w;
    height = h;
}
