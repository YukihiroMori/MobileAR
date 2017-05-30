//
//  Utils.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef GLUtils_hpp
#define GLUtils_hpp

#include <opencv2/opencv.hpp>
#include "Obj.hpp"
#include "GLSLCompiler.hpp"
#include "AABB.hpp"
#include "PathFinder.hpp"


namespace glutils{
    GLuint LoadImage( const char *imagename);
    GLuint LoadMipMapImage( const char *imagename , GLint levels);
    GLuint LoadShader( const char *vert, const char *frag);
    
    bool LoadObj( const char *name, vector<Obj::grp> &group, vector<vec3> &pos, vector<vec3> &norm, bool normalized, AABB &aabb);
}


static GLboolean printShaderInfoLog(GLuint shader, const char *str);
static GLboolean printProgramInfoLog(GLuint program);
static bool readShaderSource(GLuint shader, const char *name);

#endif /* Utils_hpp */
