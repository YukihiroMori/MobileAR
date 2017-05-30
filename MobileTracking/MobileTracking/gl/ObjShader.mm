//
//  ObjShader.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "ObjShader.hpp"

ObjShader::ObjShader( const char *vert, const char *frag) : Shader( vert, frag), isresisterd(false) {
    GLuint program = get();
    
    //pl = glGetUniformLocation(program, "pl");
    //lamb = glGetUniformLocation(program, "lamb");
    //ldiff = glGetUniformLocation(program, "ldiff");
    //lspec = glGetUniformLocation(program, "lspec");
    kamb = glGetUniformLocation(program, "kamb");
    kdiff = glGetUniformLocation(program, "kdiff");
    kspec = glGetUniformLocation(program, "kspec");
    kshi = glGetUniformLocation(program, "kshi");
    
    uniform_projection = glGetUniformLocation(program, "ProjectionMatrix");
    uniform_view = glGetUniformLocation(program, "ViewMatrix");
    uniform_model = glGetUniformLocation(program, "ModelMatrix");
    uniform_light = glGetUniformLocation(program, "LightMatrix");
}

void ObjShader::use() {
    Shader::use();
    
    //glUniform4fv(pl, 1, value_ptr(l.pos));
    //glUniform4fv(lamb, 1, value_ptr(l.amb));
    //glUniform4fv(ldiff, 1, value_ptr(l.diff));
    //glUniform4fv(lspec, 1, value_ptr(l.spec));
    
    glUniformMatrix4fv(uniform_projection, 1, GL_FALSE, value_ptr(m.ProjectionMatrix));
    glUniformMatrix4fv(uniform_view, 1, GL_FALSE, value_ptr(m.ViewMatrix));
    glUniformMatrix4fv(uniform_model, 1, GL_FALSE, value_ptr(m.ModelMatrix));
    glUniformMatrix4fv(uniform_light, 1, GL_FALSE, value_ptr(m.LightMatrix));
}

void ObjShader::unuse() {
    Shader::unuse();
}

GLuint ObjShader::get() {
    return Shader::get();
}
