//
//  Mesh.h
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef Mesh_hpp
#define Mesh_hpp

#include "GLMain.hpp"

class Mesh
{
    GLuint vao;
    
public:
    Mesh() {
        ASSERT_GL(glGenVertexArrays(1, &vao));
    }
    
    ~Mesh() {
        ASSERT_GL(glDeleteVertexArrays(1, &vao));
    }
    
    void use()
    {
        ASSERT_GL(glBindVertexArray(vao));
    }
    
    void unuse()
    {
        ASSERT_GL(glBindVertexArray(0));
    }
    
    GLuint get() {
        return vao;
    }
};

#endif /* Mesh_hpp */
