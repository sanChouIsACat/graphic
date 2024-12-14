#pragma once

#include "Holder/ModelHolder.hpp"
#include "Holder/ResourcePathHolder.hpp"
#include "Holder/TextureHolder.hpp"
namespace Holder {
class ModelTextureHolder {
private:
  ResourcePathHolder resourcePathHolder;

public:
  ModelTextureHolder(const std::string &modelPath,
                     const std::string &texturePath);
  void bind() const;
  ModelHolder modelHolder;
  TextureHolder textureHolder;
};
} // namespace Holder
