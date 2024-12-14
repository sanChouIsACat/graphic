#pragma once
#include "HolderI.hpp"
#include <string>

namespace Holder {
class TextureHolder : public HolderI {
public:
  TextureHolder(const std::string &texturePath);
  ~TextureHolder();
  unsigned int texture;
};
} // namespace Holder
