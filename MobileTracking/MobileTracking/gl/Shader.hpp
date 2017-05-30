//
//  Shader.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef Shader_hpp
#define Shader_hpp

#include "GLMain.hpp"

using namespace glm;
using namespace std;

class Shader {
    
public:
    virtual ~Shader() {
        if (program != 0) {
            ASSERT_GL(glDeleteProgram(program));
            program = 0;
        }
    }
    
    Shader() : program(0) {}
    
    Shader( const char *vert, const char *frag = 0);
    
    virtual void use(void);
    
    virtual void unuse(void);
    
    virtual GLuint get();
    
    bool UpdateUniform( int location, const mat4& m)
    {
        if (location == -1) return false;
        ASSERT_GL(glUniformMatrix4fv( location, 1, GL_FALSE, value_ptr(m)));
        return location > -1;
    }
    
    bool UpdateUniform(const std::string& name, const mat4& m)
    {
        GLint location = GetUniformLocation(name);
        if (location == -1) return false;
        return UpdateUniform( location, m);
    }
    
    bool UpdateUniform(const std::string& name, const vec2& v)
    {
        GLint location = GetUniformLocation(name);
        if (location == -1) return false;
        ASSERT_GL(glUniform2fv( location, 1, value_ptr(v)));
        return location > -1;
    }
    
    bool UpdateUniform(const std::string& name, const vec3& v)
    {
        GLint location = GetUniformLocation(name);
        if (location == -1) return false;
        ASSERT_GL(glUniform3fv( location, 1, value_ptr(v)));
        return location > -1;
    }
    
    bool UpdateUniform(const std::string& name, const vec4& v)
    {
        GLint location = GetUniformLocation(name);
        if (location > -1)
            ASSERT_GL(glUniform4fv( location, 1, value_ptr(v)));
        return location > -1;
    }
    
    bool UpdateUniform(int programId, int location, const vec4& v)
    {
        ASSERT_GL(glUniform4fv( location, 1, value_ptr(v)));
        return location > -1;
    }
    
    bool UpdateUniform(int programId, int location, const vec3& v)
    {
        ASSERT_GL(glUniform3fv( location, 1, value_ptr(v)));
        return location > -1;
    }
    
    bool UpdateUniform(const string& name, float f)
    {
        GLint location = GetUniformLocation(name);
        if (location == -1) return false;
        ASSERT_GL(glUniform1f( location, f));
        return location > -1;
    }
    
    bool UpdateUniformi(const string& name, int i)
    {
        GLint location = GetUniformLocation(name);
        if (location == -1) return false;
        ASSERT_GL(glUniform1i( location, i));
        return location > -1;
    }
    
    bool UpdateUniformiv(const string& name, int nums, int* i)
    {
        GLint location = GetUniformLocation(name);
        if (location == -1) return false;
        ASSERT_GL(glUniform1iv( location, nums, i));
        return location > -1;
    }
    
    bool UpdateUniformui(const string& name, GLuint ui)
    {
        GLint location = GetUniformLocation(name);
        if (location == -1) return false;
        ASSERT_GL(glUniform1ui( location, ui));
        return location > -1;
    }
    
    int GetUniformLocation(GLuint program, const std::string& name)
    {
        return glGetUniformLocation(program, name.c_str());
    }
    
    int GetUniformLocation(const string& name)
    {
        const auto& f = m_uniformLocations.find(name);
        if (f != m_uniformLocations.end())
        {
            return f->second;
        }
        
        GLint location = glGetUniformLocation(program, name.c_str());
        m_uniformLocations[name] = location;
        
        return location;
    }
    
protected:
    GLuint program;
    
    map<string, GLint> m_uniformLocations;
};




#endif /* Shader_hpp */
