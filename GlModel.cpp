#include "GlModel.hpp"

GlModel::GlModel(const char *file, bool keep_vertices) {
    std::vector<Vertex> vertices;
    std::vector<unsigned> triangles;

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(file, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
    if(scene->mNumMeshes != 1){
        throw std::length_error("Expected a single mesh!");
    }

    glm::vec3 minVertex(1000);
    glm::vec3 maxVertex(-1000);
    glm::vec3 avgVertex;
    auto mesh = scene->mMeshes[0];
    for (int i = 0; i < mesh->mNumVertices; ++i) {
        auto p = mesh->mVertices[i];
        auto n = mesh->mNormals[i];
        vertices.emplace_back(glm::vec3(p.x, p.y, p.z), glm::vec3(n.x, n.y, n.z));
        minVertex = glm::min(minVertex, glm::vec3(p.x, p.y, p.z));
        maxVertex = glm::max(maxVertex, glm::vec3(p.x, p.y, p.z));
        avgVertex += glm::vec3(p.x, p.y, p.z)*(1.f/mesh->mNumVertices);
    }

    for (int i = 0; i < mesh->mNumFaces; i++){
        auto face = mesh->mFaces[i];
        for (int j = 2; j < face.mNumIndices; ++j) {
            triangles.emplace_back(face.mIndices[0]);
            triangles.emplace_back(face.mIndices[j-1]);
            triangles.emplace_back(face.mIndices[j]);
        }
    }
    //std::cout << vertices.size() << '\t' << triangles.size() << std::endl;

    glGenVertexArrays(1, &_buffer);
    glBindVertexArray(_buffer);

    glGenBuffers(1, &_vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, _vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(vertices[0]), vertices.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &_index_buffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _index_buffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, triangles.size() * sizeof(triangles[0]), &triangles[0], GL_STATIC_DRAW);
    _index_count = static_cast<unsigned>(triangles.size());

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (char *) 0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_TRUE, sizeof(Vertex), (char *) 12);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);

    if(keep_vertices){
        _vertices = std::move(vertices);
        _triangles = std::move(triangles);
    }
}

void GlModel::draw() {
    glBindVertexArray(_buffer);
    glDrawElements(GL_TRIANGLES, _index_count, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

GlModel::~GlModel() {
    glDeleteBuffers(1, &_vertex_buffer);
    glDeleteBuffers(1, &_index_buffer);
    glDeleteVertexArrays(1, &_buffer);
}