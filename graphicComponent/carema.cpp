#include "camera.hpp"
#include "logger.hpp"
#include "transformation.hpp"

using namespace GComponent;
GComponent::Camera::Camera() {
  camera_point = Eigen::Matrix4f::Identity();
  camera_point.col(0) = Eigen::Vector4f{1, 0, 0, 0};
  camera_point.col(1) = Eigen::Vector4f{0, 1, 0, 0};
  camera_point.col(2) = Eigen::Vector4f{0, 0, -1, 0};
}
void Camera::move(Eigen::Matrix4f par) { camera_point = par * camera_point; }

void GComponent::Camera::rotateByX(float angle) {
  camera_point = transformation::generateArbitraryRotateMatrix(
                     {1, 0, 0, 0}, camera_point.col(3), angle) *
                 camera_point;
}

void GComponent::Camera::rotateByY(float angle) {
  camera_point = transformation::generateArbitraryRotateMatrix(
                     {0, 1, 0, 0}, camera_point.col(3), angle) *
                 camera_point;
}

void GComponent::Camera::rotateByZ(float angle) {
  camera_point = transformation::generateArbitraryRotateMatrix(
                     {0, 0, 1, 0}, camera_point.col(3), angle) *
                 camera_point;
}

Eigen::MatrixX4f GComponent::Camera::getViewTransformMatrix() {
  return camera_point.inverse();
}

Eigen::MatrixX4f GComponent::Camera::getViewTransformBackMatrix() {
  return camera_point;
}
