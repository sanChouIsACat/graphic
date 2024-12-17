#define _USE_MATH_DEFINES
#include "debug/lineDebugger.hpp"
#include "line.hpp"
#include "transformation.hpp"
#include "types.hpp"
#include "view.hpp"
#include <cmath>

using namespace GTypes;
POINT_EGDE_2D from3dTo2d(const POINT_EGDE_3D &a) {
  return POINT_EGDE_2D{a.x(), a.y(), 1.0f};
}

void LineDebugger::compute_mvp() {
  compute_event_loop.add_event(
      (std::function<void()>)std::bind(&LineDebugger::compute_mvp, this),
      "compute");
  Eigen::Matrix4f view = camera.getViewTransformMatrix();
  if (view == view_transform_cache) {
    return;
  }

  std::lock_guard m1{mvp_primitives_mutex};
  // may use a lock at iterator level rather than linked-list level
  std::lock_guard m2{primitives_mutex};
  mvp_primitives.clear();
  mvp_primitives.resize(primitives.size());
  auto dst = mvp_primitives.begin();
  for (auto i = primitives.begin(); i != primitives.end(); i++) {
    LinePrimitive &cur = **i;
    std::lock_guard m4{cur.mutex};
    *dst = cur.operator*((prespective_transform * view));
  }

  view_transform_cache = view;
}

void LineDebugger::draw_tri(const Triangle &tri) {
  interpolation::tanx_line_draw(from3dTo2d(tri.v[0]), from3dTo2d(tri.v[1]),
                                set_pixel_f);
  interpolation::tanx_line_draw(from3dTo2d(tri.v[1]), from3dTo2d(tri.v[2]),
                                set_pixel_f);
  interpolation::tanx_line_draw(from3dTo2d(tri.v[2]), from3dTo2d(tri.v[0]),
                                set_pixel_f);
}

void LineDebugger::draw_line(const Line &line) {
  interpolation::tanx_line_draw(from3dTo2d(line.a), from3dTo2d(line.b),
                                set_pixel_f);
}

void LineDebugger::draw_f() {
  // raster primitives
  std::lock_guard t{mvp_primitives_mutex};
  auto it = mvp_primitives.begin();
  while (it != mvp_primitives.end()) {
    LinePrimitive &pri = **it;
    std::lock_guard cur_lock{pri.mutex};

    // just a debugger, may use other design mode that couple raster algo with
    // concreate primitive types
    const type_info &id = typeid(pri);
    if (id == typeid(GTypes::Triangle)) {
      draw_tri((GTypes::Triangle &)pri);
    } else if (id == typeid(pri)) {
      draw_line((GTypes::Line &)pri);
    }
    it++;
  }
}

void LineDebugger::start() {
  GComponent::GameWindow<CameraStateMachine>::DRAR_FUNCTION f =
      std::bind(&LineDebugger::draw_f, this);
  gameWindow.set_draw_functions(
      GComponent::GameWindow<CameraStateMachine>::DRAR_FUNCTIONS{f});
  compute_mvp();
  std::thread t1(std::bind(&GComponent::EventLoop::start, &compute_event_loop));
  gameWindow.run();
}
void LineDebugger::addLinePrimitives(GTypes::LinePrimitive &pri) {
  std::lock_guard m2{primitives_mutex};

  primitives.push_back(&pri);
}

LineDebugger::LineDebugger(int w, int h, GTypes::AABB aabb,
                           const std::string window_name,
                           const POINT_EGDE_3D &origin)
    : camera(), prespective_transform(view::generatePerspectiveProjection(
                    down_left, top_right, far)),
      cameraController(aabb, w, h, 10000, 10000, camera),
      state_machine_context{cameraController, compute_event_loop},
      machine(state_machine_context), compute_event_loop{1000, 10},
      gameWindow(w, h, window_name, machine) {
  std::function<void(const POINT_EGDE_2D &, const RGB &)> func =
      std::bind(&GComponent::FrameBuffer::set_pixel, &gameWindow,
                std::placeholders::_1, std::placeholders::_2);
  set_pixel_f = func;
}