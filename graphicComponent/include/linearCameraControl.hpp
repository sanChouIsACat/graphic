#pragma once
#include "camera.hpp"
#include "cameraControlI.hpp"
#include "transformation.hpp"
#include "types.hpp"

namespace GComponent {
// No Acceleration for rotate and movement
class LinearCameraControl : public CameraControlI {
public:
  LinearCameraControl(const GTypes::AABB &aabb, int screen_width,
                      int screen_height, float ratio_z, float ratio_x,
                      GComponent::Camera &camera);
  void keyBoardX(int x, long long timestamp) override;
  void keyBoardZ(int y, long long timestamp) override;
  void mouse(int x, int y);
  void resize(int width, int height);
  void setEnterCoords(int x, int y);

private:
  unsigned int box_x_len_buffer;
  unsigned int box_y_len_buffer;
  unsigned int box_z_len_buffer;
  float computeMoveStep(int sign, long long &ratio, long long current_timestamp,
                        long long &timestamp, unsigned int &screen_len,
                        int offset);

public:
  // Camera
  Camera &camera;

private:
  // the volumn of whole scene
  GTypes::AABB aabb;
  // pixel
  int screen_width;
  // pixel
  int screen_height;
  // how many millionseconds can user move from front to back.
  long long ratio_z;
  // how many millionseconds can user move from left to right.
  long long ratio_x;
  // how many ratio will rorate when mouse move from left to right
  float ratio_rorate_x;
  // how many ratio will rorate when mouse move from buttom to top
  float ratio_rorate_y;
  Eigen::Vector2i last_time_mouse_coordiante;
  // the timestamp when user press wasd
  long long last_time_x_press;
  long long last_time_z_press;
  // the interval that two press will be consider as dependent event
  long long press_threshlod;
};
} // namespace GComponent