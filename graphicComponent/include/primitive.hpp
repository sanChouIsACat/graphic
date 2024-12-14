#pragma once
#include "eigen3/Eigen/Eigen"
#include "removable.hpp"
#include <shared_mutex>
namespace GTypes {
// Line primitives represent objects that consist of lines
class LinePrimitive : public Removable {
public:
  std::shared_mutex mutex;
  LinePrimitive() = default;
  LinePrimitive(const LinePrimitive &other){};
  virtual std::unique_ptr<LinePrimitive>
  operator*(Eigen::Matrix4f transform) const = 0;
  virtual ~LinePrimitive();
};
} // namespace GTypes
