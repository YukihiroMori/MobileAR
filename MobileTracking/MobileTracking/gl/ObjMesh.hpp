//
//  ObjMesh.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef ObjMesh_hpp
#define ObjMesh_hpp

#include "Shader.hpp"
#include "Mesh.hpp"
#include "Obj.hpp"
#include "AABB.hpp"
#include "Transform.hpp"

class ObjMesh : public Mesh
{
private:
    GLuint vbo[2];
    
public:
    gl::Transform transform;
    AABB aabb;
    
    ~ObjMesh();
    
    ObjMesh( const char *name, bool normalaize = false);
    
    void attachShader(Shader &shader) {
        this->shader = &shader;
    }
    
    void attachShader(Shader *shader) {
        this->shader = shader;
    }
    
    void draw();
    
    void attachShadowShader(Shader &shader) {
        this->shadow = &shader;
    }
    
    void attachShadowShader(Shader *shader) {
        this->shadow = shader;
    }
    
    void shadow_pass();
    
private:
    vector<Obj::grp> group;
    Shader *shader;
    Shader *shadow;
};

#endif /* ObjMesh_hpp */
