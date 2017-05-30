//
//  Obj.h
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef Obj_hpp
#define Obj_hpp

#include "GLMain.hpp"

using namespace std;
using namespace glm;

namespace Obj {
    class mat      // マテリアル
    {
    public:
        mat() {}
        mat(mat *m) {
            amb = m->amb;
            diff = m->diff;
            spec = m->spec;
            shi = m->shi;
            dis = m->dis;
        }
        mat(vec4 _amb, vec4 _diff, vec4 _spec, float shi, float dis)
        : amb(_amb), diff(_diff), spec(_spec), shi(shi), dis(dis) {}
        
        vec4 amb;      // ambient
        vec4 diff;     // diffuse
        vec4 spec;     // specular
        float shi;    // shininess
        float dis;    // dissolve
    };
    class face      // 面データ
    {
    public:
        vector<int> p;  // 頂点座標番号
        vector<int> t;  // テクスチャ座標番号
        vector<int> n;  // 頂点法線番号
        bool smooth;  // スムーズシェーディングの有無
    };
    class grp      // 面グループ
    {
    public:
        string name;
        int b;     // 面グループの開始番号
        int c;     // 面グループの頂点数
        mat m; // 面グループのマテリアル
        grp() {}
        grp(string name,int begin, int count, mat material)
        : name(name), b(begin), c(count), m(material) {}
    };
}

#endif /* Obj_h */
