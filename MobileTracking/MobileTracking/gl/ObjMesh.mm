//
//  ObjMesh.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "ObjMesh.hpp"
#include "GLUtils.hpp"

ObjMesh::~ObjMesh(){
    use();
    
    ASSERT_GL(glBindBuffer(GL_ARRAY_BUFFER, 0));
    ASSERT_GL(glDeleteBuffers(2 , vbo));
    
    unuse();
};

ObjMesh::ObjMesh( const char *name, bool normalaize)
: Mesh(),
transform()
{
    
    vector<vec3> pos, norm;
    
    if (glutils::LoadObj( name, group, pos, norm , normalaize, aabb)) {
        
        use();
        
        ASSERT_GL(glGenBuffers(2 ,vbo));
        
        ASSERT_GL(glBindBuffer(GL_ARRAY_BUFFER, vbo[0]));
        ASSERT_GL(glBufferData(GL_ARRAY_BUFFER, sizeof(vec3) * pos.size(), &pos[0], GL_STATIC_DRAW));
        ASSERT_GL(glEnableVertexAttribArray(0));
        ASSERT_GL(glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0));
        
        ASSERT_GL(glBindBuffer(GL_ARRAY_BUFFER, vbo[1]));
        ASSERT_GL(glBufferData(GL_ARRAY_BUFFER, sizeof(vec3) * norm.size(), &norm[0], GL_STATIC_DRAW));
        ASSERT_GL(glEnableVertexAttribArray(1));
        ASSERT_GL(glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0));
        
        unuse();
        
    }
}

void ObjMesh::draw() {
    shader->use();
    shader->UpdateUniform("ModelMatrix", transform.asMatrix());
    
    use();
    
    for (unsigned int g = 0; g < group.size(); ++g)
    {
        Obj::mat mat = group[g].m;
        
        shader->UpdateUniform("kamb", mat.amb);
        shader->UpdateUniform("kdiff", mat.diff);
        shader->UpdateUniform("kspec", mat.spec);
        shader->UpdateUniform("kshi", mat.shi);
        
        ASSERT_GL(glDrawArrays(GL_TRIANGLES, group[g].b , group[g].c));
    }
    
    unuse();
    shader->unuse();
}

void ObjMesh::shadow_pass() {
    shadow->UpdateUniform("ModelMatrix", transform.asMatrix());
    
    shadow->use();
    use();
    
    for (unsigned int g = 0; g < group.size(); ++g)
    {
        Obj::mat mat = group[g].m;
        ASSERT_GL(glDrawArrays(GL_TRIANGLES, group[g].b, group[g].c));
    }
    
    unuse();
    shader->unuse();
}



