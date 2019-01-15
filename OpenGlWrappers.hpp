#pragma once

#include <initializer_list>
#include <stdexcept>
#include <string>

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glext.h>

/// An OpenGL Shader
class Shader{
public:
    /// Compiles a shader from the given source
    /// \param type The type of shader. Should be one of the GL_XX_SHADER constants
    /// \param code A pointer to a string containing the source code
    Shader(GLuint type, const char* code);

    ///Don't allow copying
    Shader(const Shader&) = delete;

    /// \return the OpenGL shader id
    GLuint shader() const;

    /// Frees the shader resources
    ~Shader();
private:
    GLuint _shader;
};

/// An OpenGL Program
class Program{
public:
    /// Creates program from a number of shaders
    /// \param shaders The shaders to include in the program
    Program(std::initializer_list<Shader> shaders);

    ///Don't allow copying
    Program(const Program&) = delete;

    /// \return the OpenGL program id
    GLuint program();

    /// Base case of variadic template below
    void get_shader_uniforms(){}

    /// Gets the GLints for each uniform in a program
    /// \tparam Targs The types of all the remaining uniforms. Should alternate between uniform names and GLint references.
    /// \param program The program containing the uniforms
    /// \param uniform_name The name of the next uniform to get
    /// \param uniform A reference to a place to store the uniform
    /// \param vargs Any further uniforms to get
    template<class ... Targs>
    void get_shader_uniforms(const char* uniform_name, GLint &uniform, Targs & ... vargs){
        auto error = glGetError();
        if(error != GL_NO_ERROR)
            throw std::runtime_error("A" + std::to_string(error));

        uniform = glGetUniformLocation(_program, uniform_name);
        error = glGetError();
        if(error != GL_NO_ERROR)
            throw std::runtime_error("B" + std::to_string(error));
        if(uniform == -1)
            throw std::invalid_argument(std::string("Could not find uniform: ") + uniform_name);
        get_shader_uniforms(vargs...);
    }

    /// Frees the program resources
    ~Program();
private:
    GLuint _program;
};