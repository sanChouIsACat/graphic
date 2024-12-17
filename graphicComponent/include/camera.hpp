#pragma once
#include "eigen3/Eigen/Eigen"
#include "mutex"
#include "types.hpp"
namespace GComponent {
class Camera {
private:
  using CameraPoint = Eigen::Matrix4f;

private:
  CameraPoint camera_point;

public:
  Camera();
  std::recursive_mutex mutex;
  // all movement calculations are in carema coordinates systems
  void move(Eigen::Matrix4f par);
  void rotateByX(float angle);
  void rotateByY(float angle);
  void rotateByZ(float angle);
  // get origin(the same as last time getAndUpdateViewTransformMatrix returned)
  // camera transform matrix
  Eigen::MatrixX4f getViewTransformMatrix();
  Eigen::MatrixX4f getViewTransformBackMatrix();
};
} // namespace GComponent