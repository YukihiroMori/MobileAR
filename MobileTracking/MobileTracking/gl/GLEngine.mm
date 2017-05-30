//
//  GLEngine.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "GLEngine.hpp"
#include "GLUtils.hpp"

void GLEngine::init(){
    PathFinder::getInstance().ResourcesPath = Path;
    
    GLint levels = 5.0;
    
    vector<string> texfile =
    {
        //"Cave-Room_Ref.hdr",
        //"Gold-Room.Ref.hdr",
        "ProvWash_Ref.hdr",
        //"white.hdr"
    };
    
    texname = vector<GLuint>(texfile.size());
    ASSERT_GL(glGenTextures(texfile.size(), &texname[0]));
    for (int i = 0; i < texfile.size(); ++i)
    {
        texname[i] = glutils::LoadMipMapImage(texfile[i].c_str() , levels);
    }
    
    buf = {
        GL_COLOR_ATTACHMENT0,
        GL_COLOR_ATTACHMENT1,
        GL_COLOR_ATTACHMENT2,
        GL_COLOR_ATTACHMENT3
    };
    
    texnum = buf.size() + 2;
    tex = vector<GLuint>(texnum);
    
    fbo = prepareFBO(buf, tex);
    
    pass1 = shared_ptr<Shader>(new Shader("pass1.vert", "pass1.frag"));
    pass2 = shared_ptr<Shader>(new Shader("PBR.vert", "PBR2.frag"));
    
    Obj::mat mat(
                 vec4(0.1f, 0.1f, 0.1f, 1.0f),
                 vec4(0.6f, 0.6f, 0.6f, 1.0f),
                 vec4(0.3f, 0.3f, 0.3f, 1.0f),
                 60.0f, 1.0f);
    
    pass1->GetUniformLocation("kamb");
    pass1->GetUniformLocation("kdiff");
    pass1->GetUniformLocation("kspec");
    pass1->GetUniformLocation("kshi");
    
    pass1->GetUniformLocation("ProjectionMatrix");
    pass1->GetUniformLocation("ViewMatrix");
    pass1->GetUniformLocation("ModelMatrix");
    
    pass1->use();
    
    pass1->UpdateUniform("kamb", mat.amb);
    pass1->UpdateUniform("kdiff", mat.diff);
    pass1->UpdateUniform("kspec", mat.spec);
    pass1->UpdateUniform("kshi", mat.shi);
    
    pass1->unuse();
    
    GLint unit[6];
    for (GLint i = 0; i < texnum; ++i) unit[i] = i;
    
    GLuint g_m = 8;
    GLfloat g_roughness = 0.1f;
    GLfloat g_R0 = 1.0f;
    
    static vec4 ambient = vec4(0.3f, 0.3f, 0.3f, 1.0f);
    
    pass2->GetUniformLocation("NumSamples");
    pass2->GetUniformLocation("u_m");
    pass2->GetUniformLocation("roughness");
    pass2->GetUniformLocation("f0");
    
    pass2->GetUniformLocation("ambient");
    pass2->GetUniformLocation("unit");
    
    pass2->use();
    
    pass2->UpdateUniformui("NumSamples", 1 << g_m);
    pass2->UpdateUniformui("u_m", g_m);
    pass2->UpdateUniform("roughness", g_roughness);
    pass2->UpdateUniform("f0", g_R0);
    
    pass2->UpdateUniform("ambient", ambient);
    pass2->UpdateUniformiv("unit", texnum, unit);
    
    pass2->unuse();
    
    obj = shared_ptr<ObjMesh>(new ObjMesh("bunny.obj", true));
    obj->attachShader(*pass1);
    obj->transform.position = vec3(0.0f, 0.0f, 1200.0f);
    obj->transform.scale = vec3(100.0f);
    
    vector<vec3> position = {
        vec3( -1.0f, -1.0f, 0.0f),
        vec3(  1.0f, -1.0f, 0.0f),
        vec3( -1.0f,  1.0f, 0.0f),
        vec3(  1.0f,  1.0f, 0.0f)
    };
    
    quad = shared_ptr<Quad>(new Quad(position, GL_TRIANGLE_STRIP));
    
}

void GLEngine::draw(GLuint fb){
    
    ASSERT_GL(glBindFramebuffer(GL_FRAMEBUFFER, fbo));
    
    ASSERT_GL(glDrawBuffers(buf.size(), &buf[0]));
    
    ASSERT_GL(glViewport(0, 0, FBOWIDTH, FBOHEIGHT));
    
    //camera.setViewPort();
    
    ASSERT_GL(glEnable(GL_CULL_FACE));
    
    ASSERT_GL(glCullFace(GL_BACK));
    
    ASSERT_GL(glEnable(GL_DEPTH_TEST));
    
    ASSERT_GL(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
    
    pass1->use();
    
    pass1->UpdateUniform("ProjectionMatrix", camera.getProjectionMatrix());
    pass1->UpdateUniform("ViewMatrix", camera.getViewMatrix());
    
    pass1->unuse();
    
    obj->draw();
    
    //obj->transform.rotation = window.getLeftTrack().getQuat();
    
    //obj->draw();
    
    
    ASSERT_GL(glDisable(GL_CULL_FACE));
    
    ASSERT_GL(glBindFramebuffer(GL_FRAMEBUFFER, fb));
    
    //ASSERT_GL(glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo));
    
    ASSERT_GL(glDrawBuffers(1, &buf[0]));
    
    camera.setViewPort();
    
    ASSERT_GL(glDisable(GL_DEPTH_TEST));
    
    tex[5] = texname[irselect];
    
    ASSERT_GL(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
    
    mat4 proj = camera.getProjectionMatrix();
    mat4 view = camera.getViewMatrix();
    
    pass2->use();
    
    pass2->UpdateUniform("ProjectionMatrix", proj);
    pass2->UpdateUniform("ViewMatrix", view);
    pass2->UpdateUniform("ModelMatrix", T.asMatrix());
    
    for (GLsizei i = 0; i < texnum; ++i) {
        ASSERT_GL(glActiveTexture(GL_TEXTURE0 + i));
        ASSERT_GL(glBindTexture(GL_TEXTURE_2D, tex[i]));
    }
    
    quad->draw();
    
    pass2->unuse();
}


GLuint GLEngine::attachTexture(GLint internal, GLint format, GLsizei width, GLsizei height, GLenum attachment){
    GLuint texture;
    
    ASSERT_GL(glGenTextures(1, &texture));
    
    ASSERT_GL(glBindTexture(GL_TEXTURE_2D, texture));
    
    ASSERT_GL(glPixelStorei(GL_UNPACK_ALIGNMENT, (format == GL_RGBA || format == GL_BGRA) ? 4 : 1));
    
    ASSERT_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    ASSERT_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    ASSERT_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    ASSERT_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    
    ASSERT_GL(glTexImage2D(GL_TEXTURE_2D, 0, internal, width, height, 0, format, (internal == GL_RGBA) ?GL_UNSIGNED_BYTE : GL_FLOAT, NULL));
    
    ASSERT_GL(glFramebufferTexture2D(GL_FRAMEBUFFER, attachment, GL_TEXTURE_2D, texture, 0));
    
    ASSERT_GL(glBindTexture(GL_TEXTURE_2D, 0));
    
    return texture;
}

GLuint GLEngine::prepareFBO(vector<GLenum> &buf, vector<GLuint> &tex){
    
    GLuint fbo;
    ASSERT_GL(glGenFramebuffers(1, &fbo));
    ASSERT_GL(glBindFramebuffer(GL_FRAMEBUFFER, fbo));
    
    tex[0] = attachTexture(GL_RGBA, GL_RGBA, FBOWIDTH, FBOHEIGHT, buf[0]);
    
    tex[1] = attachTexture(GL_RGBA, GL_RGBA, FBOWIDTH, FBOHEIGHT, buf[1]);
    
    tex[2] = attachTexture(GL_RGB16F, GL_RGB, FBOWIDTH, FBOHEIGHT, buf[2]);
    
    tex[3] = attachTexture(GL_RGB16F, GL_RGB, FBOWIDTH, FBOHEIGHT, buf[3]);
    
    tex[4] = attachTexture(GL_DEPTH_COMPONENT32F, GL_DEPTH_COMPONENT, FBOWIDTH, FBOHEIGHT, GL_DEPTH_ATTACHMENT);
    
    ASSERT_GL(glBindFramebuffer(GL_FRAMEBUFFER, 0));
    
    return fbo;
}






























