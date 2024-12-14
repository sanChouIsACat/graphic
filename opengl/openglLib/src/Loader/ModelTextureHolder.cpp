#include "Holder/ModelTextureHolder.hpp"
#include "glad/glad.h"

namespace Holder {

ModelTextureHolder::ModelTextureHolder(const std::string &modelPath,
                                       const std::string &texturePath)
    : resourcePathHolder(),
      modelHolder(resourcePathHolder.getModelPath() + modelPath),
      textureHolder(resourcePathHolder.getModelPath() + texturePath){};

void ModelTextureHolder::bind() const {
  glBindTexture(GL_TEXTURE_2D, textureHolder.texture);
  glBindVertexArray(modelHolder.vao);
}
} // namespace Holder
