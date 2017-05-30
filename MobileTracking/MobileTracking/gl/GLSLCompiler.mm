//
//  GLSLCompiler.cpp
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/14.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

/*
 もんしょの巣穴
 
 OpenGLの話　第2回
 
 GLSLコンパイラ
 
 https://sites.google.com/site/monshonosuana/opengl/opengl_002
 */

#include "GLSLCompiler.hpp"
#include <string.h>
#include <memory>
#include <regex>
#include <iostream>
#include <sstream>
#include <fstream>

namespace
{
    //----
    std::string LoadTextFile(const char* filename)
    {
        std::string result;
        
        std::ifstream file(filename, std::ios::binary);
        
        if (file.fail()) {
            std::cerr << "Error : Can't open source file: " << filename << std::endl;
        }
        else {
            std::string buffer((std::istreambuf_iterator<char>(file)),
                               std::istreambuf_iterator<char>());
            
            if (file.bad()) {
                std::cerr << "Error: Could not read source file: " << filename << std::endl;
            }
            else {
                result  = buffer;
            }
            file.close();
        }
        
        return result;
    }
    
    //----
    std::string FindFile(const std::vector<std::string>& paths, const std::string& filename)
    {
        std::vector<std::string>::const_iterator it = paths.begin();
        for (; it != paths.end(); ++it)
        {
            std::string filepath = (*it) + filename;
            
            std::ifstream file(filepath, std::ios::binary);
            
            if (file.fail()) {
                std::cerr << "Error : Can't open source file: " << filename << std::endl;
            }
            else {
                file.close();
                return filepath;
            }
        }
        
        return std::string();
    }
    
    //----
    std::string ResolveInclude(const std::string& source, const std::vector<std::string>& paths, int level = 0)
    {
        if (level > 16)
        {
            std::cout << "警告！" << std::endl;
            std::cout << "    インクルードファイルの深度がリミットに到達しました" << std::endl;
            std::cout << "    循環参照の可能性があるため、インクルードファイルの解決を打ち切ります。" << std::endl;
            return source;
        }
        
        using namespace std;
        
        static const regex kIncludePattern("^[ ]*#[ ]*include[ ]+[\"<](.*)[\">].*");
        stringstream inputStream;
        stringstream outputStream;
        inputStream << source;
        
        smatch matches;
        string line;
        while (getline(inputStream, line))
        {
            if (regex_search(line, matches, kIncludePattern))
            {
                string include_filename = matches[1];
                string include_filepath = FindFile(paths, include_filename);
                
                if (include_filepath.empty())
                {
                    std::cout << "GLSLのコンパイルに失敗しました" << std::endl;
                    std::cout << "    インクルードファイル(" << include_filename << ")が指定パスに見つかりませんでした。" << std::endl;
                }
                string include_string = LoadTextFile(include_filepath.c_str());
                if (include_string.empty())
                {
                    std::cout << "GLSLのコンパイルに失敗しました" << std::endl;
                    std::cout << "    インクルードファイル(" << include_filepath << ")の読み込みに失敗しました。" << std::endl;
                }
                
                outputStream << ResolveInclude(include_string, paths, level + 1) << endl;
            }
            else
            {
                outputStream << line << endl;
            }
        }
        
        return outputStream.str();
    }
    
    //----
    // シェーダオブジェクトのコンパイル結果を表示する
    bool PrintShaderInfoLog(GLuint shader)
    {
        // コンパイル結果を取得する
        GLint status;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
        if (status == GL_FALSE)
        {
            std::cout << "GLSLのコンパイルに失敗しました" << std::endl;
        }
        
        // シェーダのコンパイル時のログの長さを取得する
        GLsizei bufSize;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &bufSize);
        
        if (bufSize > 1)
        {
            // シェーダのコンパイル時のログの内容を取得する
            std::vector<GLchar> infoLog(bufSize);
            GLsizei length;
            glGetShaderInfoLog(shader, bufSize, &length, &infoLog[0]);
            std::cout << "    " << &infoLog[0] << std::endl;
        }
        
        return status == GL_TRUE;
    }
    
}	// namespace

//----
bool GLSLCompiler::HasIncludeARB()
{
    const GLubyte* exts = glGetString(GL_EXTENSIONS);
    return strstr(reinterpret_cast<const char*>(exts), "GL_ARB_shading_language_include") != NULL;
}

//----
bool GLSLCompiler::CompileFromFile(GLuint shader, const char* filename)
{
    std::string source = LoadTextFile(filename);
    if (source.empty())
    {
        std::cout << "GLSLのコンパイルに失敗しました" << std::endl;
        std::cout << "    シェーダファイル(" << filename << ")が見つかりません。" << std::endl;
        return false;
    }
    
    return CompileFromString(shader, source);
}

//----
bool GLSLCompiler::CompileFromString(GLuint shader, const std::string& source)
{
    std::string resolve_source;
    
    // 定義を書き込む
    std::vector<DefineValue>::iterator it = m_defines.begin();
    for (; it != m_defines.end(); ++it)
    {
        resolve_source += "#define " + it->first + " " + it->second + "\n";
    }
    resolve_source += source;
    
    // #includeディレクティブを解決する
    resolve_source = ResolveInclude(resolve_source, m_includePaths);
    
    if (resolve_source.empty())
    {
        return false;
    }
    
    // コンパイルする
    const char* ps = resolve_source.c_str();
    glShaderSource(shader, 1, reinterpret_cast<const GLchar**>(&ps), NULL);
    glCompileShader(shader);
    if (!PrintShaderInfoLog(shader))
    {
        return false;
    }
    
    return true;
}

//	EOF
