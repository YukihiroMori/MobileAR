//
//  ObjShader.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef ObjShader_hpp
#define ObjShader_hpp

#include "GLMain.hpp"
#include "Shader.hpp"
#include "GLUtils.hpp"

class ObjShader : public Shader {
    bool isresisterd;
    
public:
    
    ObjShader( const char* vert, const char* frag);
    
    virtual void use();
    
    void setMaterial(vec4 amb, vec4 diff, vec4 spec, float shi) {
        if (!isresisterd) {
            glUniform4fv(kamb, 1, value_ptr(amb));
            glUniform4fv(kdiff, 1, value_ptr(diff));
            glUniform4fv(kspec, 1, value_ptr(spec));
            glUniform1f(kshi, shi);
            isresisterd = true;
        }
    }
    
    void setProjectionMatrix(mat4 projection) {
        m.ProjectionMatrix = projection;
    }
    
    void setViewMatrix(mat4 view) {
        m.ViewMatrix = view;
    }
    
    void setModelMatrix(mat4 model) {
        m.ModelMatrix = model;
    }
    
    void setLightMatrix(mat4 light) {
        m.LightMatrix = light;
    }
    
    virtual void unuse();
    
    GLuint get();
    
    class light {
    public:
        vec4 pos;
        vec4 amb;
        vec4 diff;
        vec4 spec;
    };
    
    class Matrix {
    public:
        mat4 ProjectionMatrix;
        mat4 ViewMatrix;
        mat4 ModelMatrix;
        mat4 LightMatrix;
    };
    
    light l;
    Matrix m;
    
    GLint pl;
    GLint lamb;
    GLint ldiff;
    GLint lspec;
    GLint kamb;
    GLint kdiff;
    GLint kspec;
    GLint kshi;
    
    GLint uniform_projection;
    GLint uniform_view;
    GLint uniform_model;
    GLint uniform_light;
};

#endif /* ObjShader_hpp */
