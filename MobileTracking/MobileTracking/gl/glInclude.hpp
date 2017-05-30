//
//  glInclude.h
//  MobileTracking
//
//  Created by 森 幸浩 on 2017/04/25.
//  Copyright © 2017年 森 幸浩. All rights reserved.
//

#ifndef glInclude_h
#define glInclude_h

#if defined(__APPLE__)
#include <OpenGLES/ES3/gl.h>
#include <OpenGLES/ES3/glext.h>
#else
#error Need an OpenGL implementation
#endif
#include "assert.h"

#define STATUS_CASE(enum) case enum: return #enum
static const char* _glStatusString(GLenum error)
{
    switch(error) {
            STATUS_CASE(GL_NO_ERROR);
            STATUS_CASE(GL_INVALID_ENUM);
            STATUS_CASE(GL_INVALID_VALUE);
            STATUS_CASE(GL_INVALID_OPERATION);
            STATUS_CASE(GL_INVALID_FRAMEBUFFER_OPERATION);
            STATUS_CASE(GL_OUT_OF_MEMORY);
            STATUS_CASE(GL_FRAMEBUFFER_COMPLETE);
            STATUS_CASE(GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT);
            STATUS_CASE(GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS);
            STATUS_CASE(GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT);
            STATUS_CASE(GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE);
            STATUS_CASE(GL_FRAMEBUFFER_UNSUPPORTED);
    }
    return NULL;
}
#undef STATUS_CASE

#ifndef ASSERT_GL
#ifndef NDEBUG
#define ASSERT_GL(x)                                    \
do {                                                \
GLenum _glError;                                \
x;                                              \
_glError = glGetError();                        \
if(_glError != GL_NO_ERROR) {                   \
printf("%s:%d:  %s Error: %s\n",        \
__FILE__, __LINE__,             \
#x, _glStatusString(_glError)); \
}                                               \
} while(__LINE__ == -1)
#else
#define ASSERT_GL(x)
#endif
#endif

#ifndef CheckGLError
#define CheckGLError()                              \
do {                                            \
GLenum _glError = glGetError();             \
if(_glError != GL_NO_ERROR) {               \
printf("%s:%d:  OpenGL Error: %s\n",\
__FILE__, __LINE__,         \
_glStatusString(_glError));  \
}                                           \
} while(__LINE__ == -1)
#endif

#endif /* glInclude_h */
