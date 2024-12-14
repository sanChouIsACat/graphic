//
// Created by LEI XU on 4/11/19.
//

#include "Triangle.hpp"
#include "types.hpp"
#include <algorithm>
#include <array>
using namespace GTypes;
Triangle::Triangle() {
  v[0] << 0, 0, 0, 1;
  v[1] << 0, 0, 0, 1;
  v[2] << 0, 0, 0, 1;

  color[0] << 0.0, 0.0, 0.0;
  color[1] << 0.0, 0.0, 0.0;
  color[2] << 0.0, 0.0, 0.0;

  tex_coords[0] << 0.0, 0.0, 0.0;
  tex_coords[1] << 0.0, 0.0, 0.0;
  tex_coords[2] << 0.0, 0.0, 0.0;
}

std::unique_ptr<LinePrimitive>
GTypes::Triangle::operator*(Eigen::Matrix4f transform) const {
  auto ret_p = std::make_unique<Triangle>();
  Triangle &ret = *ret_p;
  ret.v[0] = transform * this->v[0];
  ret.v[1] = transform * this->v[1];
  ret.v[2] = transform * this->v[2];
  return ret_p;
}

void Triangle::setVertex(int ind, POINT_EGDE_3D ver) { v[ind] = ver; }
void Triangle::setNormal(int ind, POINT_EGDE_3D n) { normal[ind] = n; }
void Triangle::setColor(int ind, float r, float g, float b) {
  if ((r < 0.0) || (r > 255.) || (g < 0.0) || (g > 255.) || (b < 0.0) ||
      (b > 255.)) {
    fprintf(stderr, "ERROR! Invalid color values");
    fflush(stderr);
    exit(-1);
  }

  color[ind] = Vector3f((float)r / 255., (float)g / 255., (float)b / 255.);
  return;
}
void Triangle::setTexCoord(int ind, POINT_EGDE_2D uv) { tex_coords[ind] = uv; }

std::array<POINT_EGDE_3D, 3> Triangle::toVector4() const {
  std::array<POINT_EGDE_3D, 3> res;
  std::transform(std::begin(v), std::end(v), res.begin(), [](auto &vec) {
    return POINT_EGDE_3D(vec.x(), vec.y(), vec.z(), 1.f);
  });
  return res;
}

void Triangle::setNormals(const std::array<POINT_EGDE_3D, 3> &normals) {
  normal[0] = normals[0];
  normal[1] = normals[1];
  normal[2] = normals[2];
}

void Triangle::setColors(const std::array<RGB, 3> &colors) {
  auto first_color = colors[0];
  setColor(0, colors[0][0], colors[0][1], colors[0][2]);
  setColor(1, colors[1][0], colors[1][1], colors[1][2]);
  setColor(2, colors[2][0], colors[2][1], colors[2][2]);
}