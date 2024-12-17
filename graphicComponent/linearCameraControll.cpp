#include "linearCameraControl.hpp"
#include "logger.hpp"
#include <corecrt_math_defines.h>
using namespace GComponent;

LinearCameraControl::LinearCameraControl(const GTypes::AABB &aabb,
                                         int screen_width, int screen_height,
                                         float ratio_z, float ratio_x,
                                         GComponent::Camera &camera)
    : aabb(aabb), box_x_len_buffer(std::abs(aabb.top_3.x() - aabb.top_1.x())),
      box_y_len_buffer(std::abs(aabb.top_1.y() - aabb.bottom_1.y())),
      box_z_len_buffer(std::abs(aabb.top_2.z() - aabb.top_4.z())),
      screen_width(screen_width), screen_height(screen_height),
      ratio_x(ratio_x), ratio_z(ratio_z), ratio_rorate_x(1), ratio_rorate_y(1),
      camera(camera), last_time_x_press(0), last_time_z_press(0),

      press_threshlod(10), last_time_mouse_coordiante(){};
void LinearCameraControl::keyBoardX(int x, long long timestamp) {
  float step = computeMoveStep(x, ratio_x, timestamp, last_time_x_press,
                               box_x_len_buffer, 0);

  Eigen::Matrix4f movement = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f middle = camera.getViewTransformBackMatrix();
  middle.col(3) = Eigen::Vector4f{0, 0, 0, 1};
  movement.col(3) = middle * Eigen::Vector4f{step, 0, 0, 1};
  // update camera
  std::lock_guard t{camera.mutex};
  camera.move(std::move(movement));
}
void LinearCameraControl::keyBoardZ(int y, long long timestamp) {
  float step = computeMoveStep(y, ratio_z, timestamp, last_time_z_press,
                               box_z_len_buffer, 2);

  Eigen::Matrix4f movement = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f middle = camera.getViewTransformBackMatrix();
  middle.col(3) = Eigen::Vector4f{0, 0, 0, 1};
  movement.col(3) = middle * Eigen::Vector4f{0, 0, -step, 1};
  // update camera
  std::lock_guard t{camera.mutex};
  camera.move(std::move(movement));
}
void LinearCameraControl::mouse(int x, int y) {
  float x_len = x - last_time_mouse_coordiante.x();
  float y_len = y - last_time_mouse_coordiante.y();
  float x_angle = x_len / screen_width * ratio_rorate_x * M_PI;
  float y_angle = y_len / screen_width * ratio_rorate_y * M_PI;

  std::lock_guard t{camera.mutex};
  camera.rotateByY(x_angle);
  camera.rotateByX(y_angle);
  last_time_mouse_coordiante = Eigen::Vector2i{x, y};
}

void GComponent::LinearCameraControl::resize(int width, int height) {
  this->screen_width = width;
  this->screen_height = height;
}

void GComponent::LinearCameraControl::setEnterCoords(int x, int y) {
  last_time_mouse_coordiante = Eigen::Vector2i{x, y};
};

float LinearCameraControl::computeMoveStep(int sign, long long &ratio,
                                           long long current_timestamp,
                                           long long &timestamp,
                                           unsigned int &box_len, int offset) {
  long long interval = current_timestamp - last_time_x_press;
  float step_len = sign * 1.0 * box_len *
                   (interval > press_threshlod ? 20 : interval) / ratio;
  timestamp = current_timestamp;
  return step_len;
}