#pragma once
#include "primitive.hpp"
#include "types.hpp"
namespace GTypes {
class Line : public LinePrimitive {
public:
  POINT_EGDE_3D a;
  POINT_EGDE_3D b;
  Line() = default;
  Line(const POINT_EGDE_3D &a, const POINT_EGDE_3D &b);
  virtual std::unique_ptr<LinePrimitive>
  operator*(Eigen::Matrix4f transform) const override;
};
} // namespace GTypes