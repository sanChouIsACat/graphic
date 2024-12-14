#include "window.hpp"
#include "interpolation.hpp"
#include "types.hpp"
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/transition.hpp>
#include <gtest/gtest.h>

using namespace GComponent;
namespace sc = boost::statechart;

POINT_EGDE_2D button_down_poz;
template <typename U> std::thread startLoop(GameWindow<U> &gameWindow) {
  return std::thread{&GameWindow<U>::run, &gameWindow};
}

struct InitState;
struct LButtonDown;
struct LButtonMoving;

struct Machine : public boost::statechart::state_machine<Machine, InitState> {};

struct InitState : public boost::statechart::state<InitState, Machine> {
  InitState(my_context ctx) : my_base(ctx) {}
  typedef sc::custom_reaction<MouseKeyboardEvent> reactions;

  boost::statechart::result react(const MouseKeyboardEvent &event) {
    if (event.id == cv::EVENT_LBUTTONDOWN) {
      button_down_poz = POINT_EGDE_2D{(float)event.x, (float)event.y, 1};
      return transit<LButtonDown>();
    }
    return discard_event();
  }
};

struct LButtonDown : public boost::statechart::state<LButtonDown, Machine> {
  LButtonDown(my_context ctx) : my_base(ctx) {}

  typedef sc::custom_reaction<MouseKeyboardEvent> reactions;

  boost::statechart::result react(const MouseKeyboardEvent &event) {
    if (event.id == cv::EVENT_MOUSEMOVE) {
      return transit<LButtonMoving>();
    } else if (event.id == cv::EVENT_LBUTTONUP) {
      return transit<InitState>();
    }
    return discard_event();
  }
};

struct LButtonMoving : public boost::statechart::state<LButtonMoving, Machine> {
  LButtonMoving(my_context ctx) : my_base(ctx) {}
  typedef sc::custom_reaction<MouseKeyboardEvent> reactions;

  boost::statechart::result react(const MouseKeyboardEvent &event) {
    if (event.id == cv::EVENT_LBUTTONUP) {
      POINT_EGDE_2D end = POINT_EGDE_2D{(float)event.x, (float)event.y, 1};
      FrameBuffer &buffer = *static_cast<FrameBuffer *>(event.gameWindow);
      std::function<void(const Eigen::Vector3f &, const Eigen::Vector3f &)>
          set_pixel_f = std::bind(&FrameBuffer::set_pixel, &buffer,
                                  std::placeholders::_1, std::placeholders::_2);

      interpolation::tanx_line_draw(button_down_poz, end, set_pixel_f);
      return transit<InitState>();
    }
    return discard_event();
  }
};
// 状态机类定义

TEST(GameWindowTest, normalStartTest) {
  Machine machine;
  GameWindow<Machine> gameWindow(700, 700, "test", machine);
  auto thread = startLoop(gameWindow);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  gameWindow.exitNow();
  thread.join();
}

TEST(GameWindowTest, eventTest) {
  Machine machine;
  GameWindow<Machine> gameWindow(700, 700, "test", machine);
  auto thread = startLoop(gameWindow);
  gameWindow.inject_gui_event(
      MouseKeyboardEvent{cv::EVENT_LBUTTONDOWN, 300, 300});
  gameWindow.inject_gui_event(
      MouseKeyboardEvent{cv::EVENT_MOUSEMOVE, 300, 301});
  gameWindow.inject_gui_event(
      MouseKeyboardEvent{cv::EVENT_LBUTTONUP, 300, 350});
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  gameWindow.exitNow();
  thread.join();
}