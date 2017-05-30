//
//  Quad.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/17.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef Quad_hpp
#define Quad_hpp

#include "GLMain.hpp"
#include "Mesh.hpp"
#include "Transform.hpp"
#include "GLUtils.hpp"

using namespace std;
using namespace glm;

class Quad
: public Mesh
{
public:
    gl::Transform transform;
    
    ~Quad() {
        use();
        
        ASSERT_GL(glBindBuffer(GL_ARRAY_BUFFER, 0));
        ASSERT_GL(glDeleteBuffers(1, &vbo));
        
        unuse();
    };
    
    Quad(vector<vec3> pos, GLenum mode = GL_POINTS, GLenum usage = GL_STATIC_DRAW)
    : Mesh(),
    position(pos),
    mode(mode)
    {
        ASSERT_GL(glGenBuffers(1, &vbo));
        loadPosition(pos, usage);
    };
    
    void draw(int first = 0, int count = 0) {
        use();
        
        ASSERT_GL(glDrawArrays(mode, first, count > 0 ? count : pnum() - first));
        
        unuse();
    }
    
    
private:
    GLuint vbo;
    vector<vec3> position;
    GLenum mode;
    
    GLenum getMode() {
        return this->mode;
    }
    
    void setMode(GLenum mode) {
        this->mode = mode;
    }
    
    int pnum() {
        return position.size();
    }
    
    void loadPosition(vector<vec3> pos, GLenum usage) {
        use();
        
        ASSERT_GL(glBindBuffer(GL_ARRAY_BUFFER, vbo));
        ASSERT_GL(glBufferData(GL_ARRAY_BUFFER, sizeof(vec3) * pos.size(), &pos[0], usage));
        ASSERT_GL(glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0));
        ASSERT_GL(glEnableVertexAttribArray(0));
        
        unuse();
    };
};

#endif /* Quad_hpp */
