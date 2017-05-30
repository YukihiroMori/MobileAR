//
//  GLSLCompiler.hpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef GLSLCompiler_hpp
#define GLSLCompiler_hpp

/*
 もんしょの巣穴
 
 OpenGLの話　第2回
 
 GLSLコンパイラ
 
 https://sites.google.com/site/monshonosuana/opengl/opengl_002
 */

#pragma once

#include "GLMain.hpp"


// GLSLコンパイラ
class GLSLCompiler
{
public:
    typedef std::pair<std::string, std::string>	DefineValue;
    
public:
    GLSLCompiler()
    {}
    ~GLSLCompiler()
    {}
    
    // インクルードパスを追加する
    int AddIncludePath(const std::string& path)
    {
        m_includePaths.push_back(path);
        return (int)m_includePaths.size();
    }
    int AddIncludePath(const char* path)
    {
        return AddIncludePath(std::string(path));
    }
    
    // インクルードパスをクリアする
    void ClearIncludePaths()
    {
        m_includePaths.clear();
    }
    
    // 定義を追加する
    int AddDefine(const std::string& def, const std::string& val)
    {
        m_defines.push_back(DefineValue(def, val));
        return (int)m_defines.size();
    }
    int AddDefine(const char* def, const char* val)
    {
        return AddDefine(std::string(def), std::string(val));
    }
    
    // 定義をクリアする
    void ClearDefines()
    {
        m_defines.clear();
    }
    
    // ファイルからGLSLをコンパイルする
    bool CompileFromFile(GLuint shader, const char* filename);
    
    // 文字列からGLSLをコンパイルする
    bool CompileFromString(GLuint shader, const std::string& source);
    
public:
    // GLSLで#includeディレクティブが使用できるかどうか調べる
    static bool HasIncludeARB();
    
private:
    std::vector<std::string>	m_includePaths;				// インクルードファイルを検索するパス
    std::vector<DefineValue>	m_defines;					// 標準的に使用する定義
};	// class GLSLCompiler


//	EOF


#endif /* GLSLCompiler_hpp */
