#ifndef GL_UTILS_H
#define GL_UTILS_H

#include <vector>
#include <memory>

#include <GL/glew.h>

void GLInit();
void GLTerminate();
void CheckGLError();
GLuint CompileShaders(const GLchar **vs_text, const GLchar **fs_text);

#endif // GL_UTIL_H
