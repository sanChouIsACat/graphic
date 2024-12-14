#include "Holder/ModelHolder.hpp"
#include "OBJ_Loader.h"

#include "Holder/FillHelper.hpp"
#include "glad/glad.h"
#include "stb_image.h"

namespace Holder {
ModelHolder::ModelHolder(const std::string &filepath) {
  // Load model from file using objl
  objl::Loader loader;
  if (!loader.LoadFile(filepath)) {
    error_log = "can't open model file";
    return;
  }

  auto &indices = loader.LoadedIndices;
  auto &vertices = loader.LoadedVertices;
  indCount = vertices.size();
  GlFillHelper fillHelper;
  fillHelper.fillSomeThing(glGenVertexArrays, 1, &vao)
      .fillSomeThing(glGenBuffers, 1, &vbo)
      .fillSomeThing(glGenBuffers, 1, &ebo)
      .fillSomeThing(glBindVertexArray, vao)
      // Load indices
      .fillSomeThing(glBindBuffer, GL_ELEMENT_ARRAY_BUFFER, ebo)
      .fillSomeThing(glBufferData, GL_ELEMENT_ARRAY_BUFFER,
                     indices.size() * sizeof(unsigned int), indices.data(),
                     GL_STATIC_DRAW)
      // Load vertices
      .fillSomeThing(glBindBuffer, GL_ARRAY_BUFFER, vbo)
      .fillSomeThing(glBufferData, GL_ARRAY_BUFFER,
                     vertices.size() * sizeof(objl::Vertex), vertices.data(),
                     GL_STATIC_DRAW)
      // Set vertex attribute pointers
      .fillSomeThing(glVertexAttribPointer, 0, 3, GL_FLOAT, GL_FALSE,
                     sizeof(objl::Vertex), (void *)0) // Position
      .fillSomeThing(glEnableVertexAttribArray, 0)
      .fillSomeThing(glVertexAttribPointer, 1, 3, GL_FLOAT, GL_FALSE,
                     sizeof(objl::Vertex),
                     (void *)(1 * sizeof(objl::Vector3))) // Normal
      .fillSomeThing(glEnableVertexAttribArray, 1)
      .fillSomeThing(glVertexAttribPointer, 2, 3, GL_FLOAT, GL_FALSE,
                     sizeof(objl::Vertex),
                     (void *)(2 * sizeof(objl::Vector3))) // Texture Coord
      .fillSomeThing(glEnableVertexAttribArray, 2)
      .fillSomeThing(glBindBuffer, GL_ARRAY_BUFFER, 0)
      .fillSomeThing(glBindVertexArray, 0);
  error_log = fillHelper.error_log;
}

ModelHolder::~ModelHolder() {
  glDeleteVertexArrays(1, &vao);
  glDeleteBuffers(1, &vbo);
  glDeleteBuffers(1, &ebo);
}
} // namespace Holder
