#pragma once

#include <string>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <GL/glew.h>
#include <GL/gl.h>

#include <glm/glm.hpp>
#include <vector>
#include <stdexcept>
#include <iostream>

/// Container for vertices with a position and normal on a GPU
struct Vertex{
    Vertex(const glm::vec3 &position, const glm::vec3 &normal) : position(position), normal(normal) {}

    glm::vec3 position;
    glm::vec3 normal;
};
static_assert(sizeof(Vertex)==24);

///A helper class for loading a 3D model with Assimp and rendering it with OpenGL
class GlModel {
public:
    /// Loads a 3D model from a file in any format supported by Assimp
    /// \param file The path of the file to load it from
    /// \param keep_vertices If true, the triangle data will be kept for CPU access
    explicit GlModel(const char* file, bool keep_vertices = false);

    /// Frees the OpenGL memory
    ~GlModel();

    /// Renders the model
    void draw();

    /// Returns a reference to the vertices that becomes invalidated if this object is destructed
    const std::vector<Vertex>& get_vertices(){
        return _vertices;
    }

    /// Returns a reference to the triangles that becomes invalidated if this object is destructed
    /// Triangles are stored as three consecutive indices into the vector provided by get_vertices
    const std::vector<unsigned> get_triangles(){
        return _triangles;
    }

private:
    GLuint _buffer{};
    GLuint _vertex_buffer{};
    GLuint _index_buffer{};
    unsigned _index_count{};

    std::vector<Vertex> _vertices;
    std::vector<unsigned> _triangles;
};