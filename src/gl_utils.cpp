#include "gl_utils.h"

#include <sstream>
#include <cassert>
#include <iostream>
#include <fstream>
#include <memory>
#include <cstring>

#include <GLFW/glfw3.h>

static void ErrorCallback(int error, const char *message)
{
    std::cerr << "GLFW error: " << message << " (code " << error << ")" << std::endl;
}

void GLInit()
{
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        exit(-1);
    }
    glfwSetErrorCallback(ErrorCallback);
}

void GLTerminate()
{
    glfwTerminate();
}

void CheckGLError()
{
    GLenum error = glGetError();
    if (error != GL_NO_ERROR)
    {
        std::stringstream ss;
        ss << "OpenGL error " << error  << " ";
        if (error == GL_INVALID_VALUE) ss << "GL_INVALID_VALUE";
        if (error == GL_INVALID_OPERATION) ss << "GL_INVALID_OPERATION";
        std::cerr << ss.str() << std::endl;
    }
}

std::string ReadShader(const char *path)
{
    std::ifstream sf(path);
    std::stringstream ss;
    while (sf.good()) {
        std::string s;
        std::getline(sf, s);
        ss << s << std::endl;
    }
    return ss.str();
}

GLuint CompileShaders(const GLchar **vs_text, const GLchar **fs_text)
{
    GLint status;
    char infoLog[1024] = {};

    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, vs_text, NULL);
    glCompileShader(vs);
    glGetShaderInfoLog(vs, 1024, NULL, infoLog);
    if (*infoLog) {
        std::cout << infoLog << std::endl;
        memset(infoLog, 0, 1024);
    }
    glGetShaderiv(vs, GL_COMPILE_STATUS, &status);
    if (status == GL_FALSE) {
        std::cerr << "Vertex shader compilation failed" << std::endl;
    }

    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, fs_text, NULL);
    glCompileShader(fs);
    glGetShaderInfoLog(fs, 1024, NULL, infoLog);
    if (*infoLog) {
        std::cout << infoLog << std::endl;
        memset(infoLog, 0, 1024);
    }
    glGetShaderiv(fs, GL_COMPILE_STATUS, &status);
    if (status == GL_FALSE) {
        std::cerr << "Fragment shader compilation failed" << std::endl;
    }

    GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glValidateProgram(program);
    glGetProgramInfoLog(program, 1024, NULL, infoLog);
    if (*infoLog) {
        std::cout << infoLog << std::endl;
    }
    glGetProgramiv(program, GL_LINK_STATUS, &status);
    if (status == GL_FALSE) {
        std::cerr << "Shader program link failed" << std::endl;
    }

    glDeleteShader(vs);
    glDeleteShader(fs);

    CheckGLError();

    return program;
}
