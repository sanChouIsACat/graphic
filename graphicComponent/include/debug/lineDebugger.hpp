#pragma once
#include "Triangle.hpp"
#include "camera.hpp"
#include "cameraStateMachine.hpp"
#include "eventLoop.hpp"
#include "interpolation.hpp"
#include "line.hpp"
#include "linearCameraControl.hpp"
#include "primitive.hpp"
#include "types.hpp"
#include "window.hpp"
#include <list>
#include <mutex>
#include <vector>
// raster objs using lines with. And you can move camera
class LineDebugger {
private:
private:
  static RGB rgb;
  std::function<void(const POINT_EGDE_2D &, const RGB &)> set_pixel_f;

  GComponent::GameWindow<CameraStateMachine> gameWindow;
  // primitives coordinates in nature coordinates.
  std::mutex primitives_mutex;
  // Debugger dosen't response for gc
  std::list<GTypes::LinePrimitive *> primitives;
  // primitives that are mvpd. raster thread will use this directly.
  std::mutex mvp_primitives_mutex;
  std::vector<std::unique_ptr<GTypes::LinePrimitive>> mvp_primitives;

  // parameter of perssitive
  float far = -100;
  POINT_EGDE_3D down_left = {-50, -50, -1, 1};
  POINT_EGDE_3D top_right = {50, 50, -1, 1};

  // mp
  Eigen::Matrix4f prespective_transform;
  Eigen::Matrix4f view_transform_cache;

  // camera
  GComponent::Camera camera;
  // camera control
  GComponent::LinearCameraControl cameraController;

  // state machine
  CameraMachineContext state_machine_context;
  CameraStateMachine machine;

private:
  GComponent::EventLoop compute_event_loop;

private:
  void compute_mvp();
  // the following lines is response for raster
  void draw_tri(const GTypes::Triangle &tri);
  void draw_line(const GTypes::Line &line);
  void draw_f();

public:
  void addLinePrimitives(GTypes::LinePrimitive &pri);
  void start();
  LineDebugger(int w, int h, GTypes::AABB aabb, const std::string window_name,
               const POINT_EGDE_3D &origin);
};