#pragma once

#include "HolderI.hpp"
#include <vector>

namespace Holder {
class ModelHolder : public HolderI {
public:
  ModelHolder(const std::string &filepath);
  ~ModelHolder();
  unsigned int vao = -1;
  unsigned int vbo = -1;
  unsigned int ebo = -1;
  unsigned int indCount = -1;
};
} // namespace Holder
