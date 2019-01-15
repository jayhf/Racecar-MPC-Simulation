#include "OpenGlWrappers.hpp"

Shader::Shader(GLuint type, const char *code) {
    _shader = glCreateShader(type);
    glShaderSource(_shader, 1, &code, nullptr);
    glCompileShader(_shader);

    int ok;
    glGetShaderiv(_shader, GL_COMPILE_STATUS, &ok);
    if(!ok)
    {
        GLint length;
        glGetShaderiv(_shader, GL_INFO_LOG_LENGTH, &length);
        std::string error(length + 1, '\0');
        glGetShaderInfoLog(_shader, length, nullptr, error.data());
        glDeleteShader(_shader);
        throw std::runtime_error("Failed to create shader: " + error);
    }
}

Shader::~Shader() {
    glDeleteShader(_shader);
}

GLuint Shader::shader() const {
    return _shader;
}

Program::Program(std::initializer_list<Shader> shaders) {
    _program = glCreateProgram();
    for(const Shader& shader : shaders){
        glAttachShader(_program, shader.shader());
    }

    glLinkProgram(_program);

    int ok;
    glGetProgramiv(_program, GL_LINK_STATUS, &ok);
    if(ok)
    {
        for(const Shader& shader : shaders){
            glDetachShader(_program, shader.shader());
        }
    }
    else
    {
        GLint length;
        glGetProgramiv(_program, GL_INFO_LOG_LENGTH, &length);
        std::string error(length + 1, '\0');
        glGetProgramInfoLog(_program, length, nullptr, error.data());
        glDeleteProgram(_program);
        throw std::runtime_error("Failed to create program: " + error);
    }
}

Program::~Program() {
    glDeleteProgram(_program);
}

GLuint Program::program() {
    return _program;
}
