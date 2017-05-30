//
//  GLEngine.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef GLEngine_hpp
#define GLEngine_hpp

#include "GLMain.hpp"
#include "ATAM.hpp"
#include "PathFinder.hpp"
#include "ObjMesh.hpp"
#include "Shader.hpp"
#include "Quad.hpp"
#include "Camera.hpp"
#include "Transform.hpp"

using namespace std;
using namespace glm;

#define FBOWIDTH 360
#define FBOHEIGHT 480

class ObjMesh;

class GLEngine{
    
    GLuint attachTexture(GLint internal, GLint format, GLsizei width, GLsizei height, GLenum attachment);
    
    GLuint prepareFBO(vector<GLenum> &buf, vector<GLuint> &tex);
    
private:
    GLuint fbo;
    vector<GLenum> buf;
    vector<GLuint> tex;
    vector<GLuint> texname;
    
    int irselect = 0;
    GLsizei texnum;
    
public:
    GLEngine(){}
    
    shared_ptr<ObjMesh> obj;
    shared_ptr<Quad> quad;
    
    shared_ptr<Shader> pass1;
    shared_ptr<Shader> pass2;
    
    Camera camera;
    
    void init();
    
    void draw(GLuint fb);
    
    string Path;
    
    gl::Transform T;
    
};

#endif /* GLEngine_hpp */
