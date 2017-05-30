//
//  Utils.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#include "GLUtils.hpp"
#include "GLMain.hpp"

GLuint glutils::LoadImage( const char *imagename) {
    
    string path(PathFinder::getInstance().ResourcesPath);
    path += imagename;
    cv::Mat image = cv::imread(path);
    //cv::cvtColor(image, image, CV_BGR2RGB);
    
    if (image.empty()) {
        cerr << "image empty" << endl;
        return 0;
    }
    
    GLuint tex;
    ASSERT_GL(glGenTextures(1, &tex));
    ASSERT_GL(glBindTexture(GL_TEXTURE_2D, tex));
    
    cv::flip(image, image, 0);
    ASSERT_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    ASSERT_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    
    ASSERT_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    ASSERT_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    
    ASSERT_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, image.ptr()));
    
    ASSERT_GL(glBindTexture(GL_TEXTURE_2D, 0));
    
    return tex;
}

GLuint glutils::LoadMipMapImage( const char *imagename, GLint levels) {
    
    string path(PathFinder::getInstance().ResourcesPath);
    path += imagename;
    cv::Mat image = cv::imread(path);
    cv::cvtColor(image, image, CV_BGR2RGB);
    
    if (image.empty()) {
        cerr << "image empty" << endl;
        return 0;
    }
    
    GLuint tex;
    ASSERT_GL(glGenTextures(1, &tex));
    ASSERT_GL(glBindTexture(GL_TEXTURE_2D, tex));
    
    cv::flip(image, image, 0);
    
    vector<cv::Mat> l(levels);
    for (GLint level = 0; level < levels; level++)
    {
        resize(image, l[level], cv::Size(), pow(0.5,level) , pow(0.5,level) , cv::INTER_AREA);
        ASSERT_GL(glTexImage2D(GL_TEXTURE_2D, level, GL_RGB, l[level].cols, l[level].rows, 0, GL_RGB, GL_UNSIGNED_BYTE, l[level].ptr()));
    }
    
    ASSERT_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR));
    ASSERT_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    
    ASSERT_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    ASSERT_GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    
    ASSERT_GL(glGenerateMipmap(GL_TEXTURE_2D));
    
    ASSERT_GL(glBindTexture(GL_TEXTURE_2D, 0));
    
    return tex;
}

GLuint glutils::LoadShader( const char *vert, const char *frag) {
    
    GLSLCompiler compiler;
    compiler.AddIncludePath(PathFinder::getInstance().ResourcesPath);
    
    GLuint program = glCreateProgram();
    
    string vpath(PathFinder::getInstance().ResourcesPath);
    vpath += vert;
    GLuint vertShader = glCreateShader(GL_VERTEX_SHADER);
    if (!compiler.CompileFromFile(vertShader, vpath.c_str())) {
        ASSERT_GL(glDeleteShader(vertShader));
        ASSERT_GL(glDeleteProgram(program));
        
        return 0;
    }
    
    ASSERT_GL(glAttachShader(program, vertShader));
    ASSERT_GL(glDeleteShader(vertShader));
    
    string fpath(PathFinder::getInstance().ResourcesPath);
    fpath += frag;
    if (frag) {
        GLuint fragShader = glCreateShader(GL_FRAGMENT_SHADER);
        
        if (!compiler.CompileFromFile(fragShader, fpath.c_str())) {
            ASSERT_GL(glDeleteShader(fragShader));
            ASSERT_GL(glDeleteProgram(program));
            return 0;
        }
        
        ASSERT_GL(glAttachShader(program, fragShader));
        ASSERT_GL(glDeleteShader(fragShader));
    }
    
    ASSERT_GL(glLinkProgram(program));
    if (printProgramInfoLog(program) == GL_FALSE) {
        ASSERT_GL(glDeleteProgram(program));
        return 0;
    }
    
    return program;
}

bool glutils::LoadObj( const char *name, vector<Obj::grp> &group, vector<vec3> &pos, vector<vec3> &norm, bool normalized, AABB &aabb) {
    group = {};
    int nv = 0;
    pos = norm = {};
    
    std::string path(PathFinder::getInstance().ResourcesPath);
    path += name;
    const size_t base = path.find_last_of("/\\");
    string dirname = (base == string::npos) ? "" : path.substr(0, base + 1);
    
    ifstream file(path.c_str());
    if (file.fail()) {
        cerr << "Error: Can't open OBJ file: " << path << endl;
        return false;
    }
    
    map<string, Obj::mat> mtl;
    static const char *defmtl = "Default";
    string mtlname(defmtl);
    
    Obj::mat def(
                     vec4(0.1f, 0.1f, 0.1f, 1.0f),
                     vec4(0.6f, 0.6f, 0.6f, 1.0f),
                     vec4(0.3f, 0.3f, 0.3f, 1.0f),
                     60.0f, 1.0f);
    
    mtl[mtlname] = def;
    
    vector<vec3> _pos;
    vector<vec3> _tex;
    vector<vec3> _norm;
    vector<Obj::face> _face;
    vector<Obj::grp> _group;
    
    int groupbegin = 0;
    
    bool smooth = false;
    
    float xmin, xmax, ymin, ymax, zmin, zmax;
    xmax = ymax = zmax = -(xmin = ymin = zmin = FLT_MAX);
    
    string line;
    while (getline(file, line)) {
        if (line == "") continue;
        
        istringstream str(line);
        
        string token;
        str >> token;
        
        if (token == "v") {
            vec3 v;
            
            str >> v.x >> v.y >> v.z;
            
            xmin = std::min(xmin, v.x);
            xmax = std::max(xmax, v.x);
            ymin = std::min(ymin, v.y);
            ymax = std::max(ymax, v.y);
            zmin = std::min(zmin, v.z);
            zmax = std::max(zmax, v.z);
            
            _pos.push_back(v);
        }
        else if (token == "vt") {
            vec3 t;
            
            str >> t.x >> t.y;
            if (!str.eof()) {
                str >> t.z;
            }
            else
                t.z = 0.0f;
            
            _tex.push_back(t);
        }
        else if (token == "vn") {
            vec3 n;
            
            str >> n.x >> n.y >> n.z;
            
            _norm.push_back(n);
        }
        else if (token == "f") {
            Obj::face f;
            
            f.p = { 0,0,0 };
            f.n = { 0,0,0 };
            f.t = { 0,0,0 };
            
            f.smooth = smooth;
            
            for (int i = 0; i < 3; i++) {
                string s;
                str >> s;
                
                f.p[i] = atoi(s.c_str());
                size_t l = s.find('/', 0);
                if (l != string::npos) {
                    ++l;
                    
                    f.t[i] = atoi(s.c_str() + l);
                    
                    l = s.find('/', l);
                    
                    if (l != string::npos) {
                        ++l;
                        
                        f.n[i] = atoi(s.c_str() + l);
                    }
                }
            }
            
            _face.push_back(f);
        }
        else if (token == "s") {
            string s;
            str >> s;
            smooth = s == "1";
        }
        else if (token == "usemtl") {
            int groupcount = static_cast<int>(_face.size()) * 3 - groupbegin;
            if (groupcount > 0) {
                Obj::grp b(mtlname, groupbegin, groupcount, mtl[mtlname]);
                _group.push_back(b);
                
                groupbegin += groupcount;
            }
            
            str >> mtlname;
            
            if (mtl.find(mtlname) == mtl.end()) {
                cerr << "Warning: Undefined material: " << mtlname << endl;
                mtlname = defmtl;
            }
            else {
                cerr << "usemtl: " << mtlname << endl;
            }
        }
        else if (token == "mtllib") {
            str >> std::ws;
            string mtlpath;
            getline(str, mtlpath);
            mtlpath = dirname + mtlpath;
            
            ifstream mtlfile(mtlpath.c_str(), ios::binary);
            if (mtlfile.fail()) {
                cerr << "Warning: Can't open MTL file: " << mtlpath << endl;
            }
            else {
                string mtlline;
                
                while (getline(mtlfile, mtlline)) {
                    istringstream mtlstr(mtlline);
                    string mtltoken;
                    mtlstr >> mtltoken;
                    
                    if (mtltoken == "newmtl")
                    {
                        mtlstr >> mtlname;
                        cerr << "newmtl: " << mtlname << endl;
                        mtl[mtlname] = def;
                    }
                    else if (mtltoken == "Ka")
                    {
                        mtlstr >> mtl[mtlname].amb.r >> mtl[mtlname].amb.g >> mtl[mtlname].amb.b;
                    }
                    else if (mtltoken == "Kd")
                    {
                        mtlstr >> mtl[mtlname].diff.r >> mtl[mtlname].diff.g >> mtl[mtlname].diff.b;
                    }
                    else if (mtltoken == "Ks")
                    {
                        mtlstr >> mtl[mtlname].spec.r >> mtl[mtlname].spec.g >> mtl[mtlname].spec.b;
                    }
                    else if (mtltoken == "Ns")
                    {
                        mtlstr >> mtl[mtlname].shi;
                    }
                    else if (mtltoken == "d")
                    {
                        mtlstr >> mtl[mtlname].dis;
                    }
                }
                
                if (mtlfile.bad())
                {
                    cerr << "Warning: Can't read MTL file: " << mtlpath << endl;
                }
                mtlfile.close();
            }
        }
    }
    
    if (file.bad()) {
        cerr << "Warning: Can't read OBJ file: " << path << endl;
        file.close();
        return false;
    }
    
    file.close();
    
    int groupcount = static_cast<int>(_face.size()) * 3 - groupbegin;
    if (groupcount > 0)
    {
        Obj::grp b(mtlname, groupbegin, groupcount, mtl[mtlname]);
        _group.push_back(b);
    }
    
    int nf = static_cast<int>(_face.size());
    nv = nf * 3;
    int ng = static_cast<int>(_group.size());
    
    try {
        group = vector<Obj::grp>(ng);
        pos = vector<vec3>(nv);
        norm = vector<vec3>(nv);
    }
    catch (bad_alloc e) {
        group.clear();
        pos.clear();
        
        group = {};
        nv = 0;
        pos = norm = {};
        
        return false;
    }
    
    float scale;
    vec3 c;
    if (normalized)
    {
        float sx = xmax - xmin;
        float sy = ymax - ymin;
        float sz = zmax - zmin;
        scale = sx;
        if (sy > scale) scale = sy;
        if (sz > scale) scale = sz;
        scale = (scale != 0.0f) ? 2.0f / scale : 1.0f;
        c.x = (xmax + xmin) * 0.5f;
        c.y = (ymax + ymin) * 0.5f;
        c.z = (zmax + zmin) * 0.5f;
    }
    else {
        scale = 1.0f;
        c.x = c.y = c.z = 0.0f;
    }
    
    aabb.max = vec3(xmax, ymax, zmax) * scale;
    aabb.min = vec3(xmin, ymin, zmin) * scale;
    
    if (_norm.empty())
    {
        static const vec3 zero = vec3(0.0f, 0.0f, 0.0f);
        _norm.resize(_pos.size(), zero);
        
        for (vector<Obj::face>::iterator it = _face.begin(); it != _face.end(); ++it)
        {
            int v0 = it->p[0] - 1;
            int v1 = it->p[1] - 1;
            int v2 = it->p[2] - 1;
            
            const vec3 & p0 = _pos[v0];
            const vec3 & p1 = _pos[v1];
            const vec3 & p2 = _pos[v2];
            
            vec3 a = p1 - p0;
            vec3 b = p2 - p0;
            vec3 n = cross(a, b);
            
            if (it->smooth) {
                _norm[v0] += n;
                _norm[v1] += n;
                _norm[v2] += n;
                
                it->n[0] = it->p[0];
                it->n[1] = it->p[1];
                it->n[2] = it->p[2];
            }
            else {
                n = normalize(n);
                
                const size_t v = _norm.size();
                _norm.resize(v + 3);
                
                _norm[v + 0] = n;
                _norm[v + 1] = n;
                _norm[v + 2] = n;
                
                it->n[0] = v + 1;
                it->n[1] = v + 2;
                it->n[2] = v + 3;
            }
        }
    }
    
    int id = 0;
    for (Obj::face f : _face)
    {
        
        for (unsigned int i = 0; i < f.p.size(); i++) {
            
            pos[id] = (_pos[f.p[i] - 1] - c)* scale;
            
            if (f.smooth) {
                norm[id] = normalize(_norm[f.n[i] - 1]);
            }
            else {
                norm[id] = _norm[f.n[i] - 1];
            }
            
#if 0
            int t = _f.t[i] - 1;
            if (t > 0)
            {
                tex[id] = _tex[t];
            }
#endif
            id++;
        }
    }
    
    for (int i = 0; i < _group.size(); i++) {
        group[i].name = _group[i].name;
        group[i].b = _group[i].b;
        group[i].c = _group[i].c;
        group[i].m = _group[i].m;
    }
    
    return true;
}

static GLboolean printProgramInfoLog(GLuint program) {
    GLint status;
    ASSERT_GL(glGetProgramiv(program, GL_LINK_STATUS, &status));
    if (status == GL_FALSE) cerr << "Link Error." << endl;
    
    GLsizei bufSize;
    ASSERT_GL(glGetProgramiv(program, GL_INFO_LOG_LENGTH, &bufSize));
    
    if (bufSize > 1)
    {
        vector<GLchar> infoLog(bufSize);
        GLsizei length;
        ASSERT_GL(glGetProgramInfoLog(program, bufSize, &length, &infoLog[0]));
        cerr << &infoLog[0] << endl;
    }
    
    return (GLboolean)status;
}
