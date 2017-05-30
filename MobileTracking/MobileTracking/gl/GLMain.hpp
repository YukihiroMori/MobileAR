//
//  Header.h
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/17.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef GLMain_hpp
#define GLMain_hpp

#include "glInclude.hpp"
#define GLM_SWIZZLE
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <string>
#include <vector>
#include <map>

#define GL_ERROR(msg) {\
GLenum error = glGetError();\
if (error != GL_NO_ERROR){\
if (msg) std::cerr << msg << ": ";\
switch (error){\
case GL_INVALID_ENUM:\
std::cerr << "An unacceptable value is specified for an enumerated argument" << std::endl;\
break;\
case GL_INVALID_VALUE:\
std::cerr << "A numeric argument is out of range" << std::endl;\
break;\
case GL_INVALID_OPERATION:\
std::cerr << "The specified operation is not allowed in the current state" << std::endl;\
break;\
case GL_OUT_OF_MEMORY:\
std::cerr << "There is not enough memory left to execute the command" << std::endl;\
break;\
case GL_INVALID_FRAMEBUFFER_OPERATION:\
std::cerr << "The specified operation is not allowed current frame buffer" << std::endl;\
break;\
default:\
std::cerr << "An OpenGL error has occured: " << std::hex << std::showbase << error << std::endl;\
break;\
}\
}\
}\

#endif /* Header_h */
