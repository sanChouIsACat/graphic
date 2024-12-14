#pragma once
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/transition.hpp>
#include <mutex>

#include "cameraControlI.hpp"
#include "eventLoop.hpp"
#include "types.hpp"
#include "window.hpp"

namespace sc = boost::statechart;
/*
 * unit: pixel.
 * The stateMachine only response for set input. Computing will be finished in
 * another code block.
 */
struct CameraMachineContext {
  GComponent::CameraControlI &cameraControlI;
  GComponent::EventLoop &taskEventLoop;
};
struct State;

struct CameraStateMachine
    : public boost::statechart::state_machine<CameraStateMachine, State> {
  explicit CameraStateMachine(CameraMachineContext &ctx) : ctx(ctx){};
  CameraMachineContext &ctx;
};

struct State : public boost::statechart::state<State, CameraStateMachine> {
  static long long getCurrentTimeStamp() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               now.time_since_epoch())
        .count();
  };
  typedef sc::custom_reaction<GComponent::MouseKeyboardEvent> reactions;
  State(my_context ctx)
      : boost::statechart::state<State, CameraStateMachine>(ctx) {}
  CameraMachineContext &getCtx() const { return outermost_context().ctx; }
  boost::statechart::result react(const GComponent::MouseKeyboardEvent &event) {
    // mouse move event
    CameraMachineContext &ctx = getCtx();
    if (event.id == cv::EVENT_MOUSEMOVE) {
      std::function<void()> mouse_f =
          std::bind(&GComponent::CameraControlI::mouse, &ctx.cameraControlI,
                    event.x, event.y);
      ctx.taskEventLoop.add_event(mouse_f, "mouse move");
    } else if (event.id == 'w') {
      std::function<void()> keyboard_w =
          std::bind(&GComponent::CameraControlI::keyBoardZ, &ctx.cameraControlI,
                    1, getCurrentTimeStamp());
      ctx.taskEventLoop.add_event(keyboard_w, "mouse move");
    } else if (event.id == 's') {
      std::function<void()> keyboard_s =
          std::bind(&GComponent::CameraControlI::keyBoardZ, &ctx.cameraControlI,
                    1, getCurrentTimeStamp());
      ctx.taskEventLoop.add_event(keyboard_s, "mouse move");
    } else if (event.id == 'a') {
      std::function<void()> keyboard_a =
          std::bind(&GComponent::CameraControlI::keyBoardZ, &ctx.cameraControlI,
                    -1, getCurrentTimeStamp());
      ctx.taskEventLoop.add_event(keyboard_a, "mouse move");
    } else if (event.id == 'd') {
      std::function<void()> keyboard_d =
          std::bind(&GComponent::CameraControlI::keyBoardZ, &ctx.cameraControlI,
                    +1, getCurrentTimeStamp());
      ctx.taskEventLoop.add_event(keyboard_d, "mouse move");
    }
    return discard_event();
  }
};
