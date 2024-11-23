#pragma once
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <shared_mutex>
#include "window.hpp"
#include "types.hpp"
#include "algebra.hpp"

namespace sc = boost::statechart;
struct Context
{
    std::vector<POINT_EGDE_2D>* control_points;
    std::shared_mutex vec_mutex;
    // control points check radio
    int radio;
    // currently moving point
    std::vector<POINT_EGDE_2D>::iterator cached_moving_point;
    Context(int radio, std::vector<POINT_EGDE_2D>* contorl_points = nullptr) :
        radio(radio), control_points(contorl_points) {};
};

// stateMachine
struct InitState;
struct CreatingOrDoNothing;
struct ModifyingControlPoints;
struct CreatingPoint;
struct TrivialEvent : public sc::event<TrivialEvent> {};

struct Machine : public boost::statechart::state_machine<Machine, InitState> {
	explicit Machine(Context& ctx) : ctx(ctx){};
	Context& ctx;
};

struct InitState : public boost::statechart::state<InitState, Machine> {
    InitState(my_context ctx) : my_base(ctx) {
    }
    typedef sc::custom_reaction<GComponent::MouseKeyboardEvent> reactions;
    
    boost::statechart::result react(const GComponent::MouseKeyboardEvent& event) {
        Context& ctx = outermost_context().ctx;
        if (event.id == cv::EVENT_LBUTTONDOWN) {
            POINT_EGDE_2D toggle_point{ (float)event.x, (float)event.y,1 };
            
            //check whether inside an exist point
            std::shared_lock lock(ctx.vec_mutex);
            auto iterator = std::find_if(
                ctx.control_points->begin(),
                ctx.control_points->end(),
                [&event,&toggle_point,&ctx](const POINT_EGDE_2D& a) {
                    return algebra::insideCircle(toggle_point, a, ctx.radio);
                }
            );
            lock.unlock();

            // not inside the circle
            if (iterator == ctx.control_points->end()) {
                return transit<CreatingOrDoNothing>();
            }
            else {
                ctx.cached_moving_point = iterator;
                return transit<ModifyingControlPoints>();
            }
        }
        return discard_event();
    }
};

struct ModifyingControlPoints : public boost::statechart::state<ModifyingControlPoints, Machine> {
    ModifyingControlPoints(my_context ctx) : my_base(ctx) {
    }
    typedef sc::custom_reaction<GComponent::MouseKeyboardEvent> reactions;

    boost::statechart::result react(const GComponent::MouseKeyboardEvent& event) {
        Context& ctx = outermost_context().ctx;
        if (event.id == cv::EVENT_MOUSEMOVE) {
            *ctx.cached_moving_point = POINT_EGDE_2D{ (float)event.x, (float)event.y, 1 };
            return discard_event();
        }
        else if (event.id == cv::EVENT_LBUTTONUP) {
            return transit<InitState>();
        }

        return discard_event();
    }
};

struct CreatingOrDoNothing : public boost::statechart::state<CreatingOrDoNothing, Machine> {
    CreatingOrDoNothing(my_context ctx) : my_base(ctx) {
    }
    typedef sc::custom_reaction<GComponent::MouseKeyboardEvent> reactions;

    boost::statechart::result react(const GComponent::MouseKeyboardEvent& event) {
        if (event.id != cv::EVENT_LBUTTONUP && event.id != cv::EVENT_MOUSEMOVE) {
            return discard_event();
        }

        Context& ctx = outermost_context().ctx;
        if (event.id == cv::EVENT_LBUTTONUP) {
            POINT_EGDE_2D current_point { (float)event.x,(float)event.y,1 };
            Context& my_ctx = outermost_context().ctx;
            auto& control_points = *my_ctx.control_points;
            std::unique_lock lock(my_ctx.vec_mutex);
            int size = control_points.size();
            if (size < 2) {
                control_points.push_back(std::move(current_point));
            }
            else {
                RGB last = control_points[size - 1];
                control_points[size - 1] = current_point;
                control_points.push_back(last);
            }
        }
        return transit<InitState>();
    }
};