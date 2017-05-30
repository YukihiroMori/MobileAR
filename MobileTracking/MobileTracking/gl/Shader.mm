//
//  Shader.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "Shader.hpp"
#include "GLUtils.hpp"

void Shader::use() {
    ASSERT_GL(glUseProgram(program));
}

void Shader::unuse() {
    ASSERT_GL(glUseProgram(0));
}

GLuint Shader::get() {
    return program;
}

Shader::Shader( const char *vert, const char *frag) : program(glutils::LoadShader( vert, frag )) {
}
