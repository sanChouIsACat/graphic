#include "line.hpp"
using namespace GTypes;
GTypes::Line::Line(const POINT_EGDE_3D &a, const POINT_EGDE_3D &b)
    : a(a), b(b){};
std::unique_ptr<LinePrimitive>
GTypes::Line::operator*(Eigen::Matrix4f transform) const {
  auto ret = std::make_unique<Line>();
  auto &new_this = *ret.get();
  new_this.a = transform * a;
  new_this.b = transform * b;
  return ret;
}
