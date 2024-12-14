#include "camera.hpp"
#include "logger.hpp"
#include "transformation.hpp"
using namespace GComponent;
GComponent::Camera::Camera() {
  camera_point = Eigen::Matrix4f::Identity();
  camera_point.col(0) = Eigen::Vector4f{0, 0, -1, 0};
  camera_point.col(1) = Eigen::Vector4f{1, 0, 0, 0};
  camera_point.col(2) = Eigen::Vector4f{0, 1, 0, 0};
  old_view_transform = camera_point;
}
void Camera::move(Eigen::Matrix4f par) {
  camera_point.col(3) = par * camera_point.col(3);
}

void GComponent::Camera::rotateByX(float angle) {
  camera_point = transformation::generateArbitraryRotateMatrix(
                     {1, 0, 0, 0}, {0, 0, 0, 1}, angle) *
                 camera_point;
}

void GComponent::Camera::rotateByY(float angle) {
  camera_point = transformation::generateArbitraryRotateMatrix(
                     {0, 1, 0, 0}, {0, 0, 0, 1}, angle) *
                 camera_point;
}

void GComponent::Camera::rotateByZ(float angle) {
  camera_point = transformation::generateArbitraryRotateMatrix(
                     {0, 0, 1, 0}, {0, 0, 0, 1}, angle) *
                 camera_point;
}

Eigen::MatrixX4f GComponent::Camera::getAndUpdateViewTransformMatrix() {
  // G_LOGGER_INFO("camera move to %s",
  // EigenStructToString(old_view_transform).c_str());
  CameraPoint new_carema = camera_point * old_view_transform.transpose();
  old_view_transform = new_carema;
  camera_point = CameraPoint::Identity();
  return new_carema;
}

Eigen::MatrixX4f GComponent::Camera::getViewTransformMatrix() {
  return old_view_transform;
}
