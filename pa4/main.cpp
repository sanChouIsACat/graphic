#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Curves.hpp"
#include "types.hpp"
#include "stateMachine.hpp"
#include "bezierCurve.hpp"
#include "window.hpp"
#include "functional"
#include "algo.hpp"
#include "bSplineCurve.hpp"

using namespace GAlgo;
using namespace GComponent;
constexpr int width = 700;
constexpr int height = 700;
constexpr int control_point_check_radio = 20;
constexpr int control_point_render_radio = 5;
constexpr float sample_rate = 0.0001;
constexpr char* window_name = "pa4";

int main() 
{
    /*
    * global variables.All stateful algos like curves/state macheins
    * are not responsible for resource management.
    * Because some resources maybe shared by more than two algos.
    * Using Ioc way of springboot maybe a good way.
    * I'm using simpler way.
    */ 
    using CurrentWindowT = GameWindow<Machine>;
    std::vector<POINT_EGDE_2D> control_points;
    Context ctx{ control_point_check_radio };
    Machine machine{ ctx };
    CurrentWindowT window{width, height, window_name, machine};
    auto f = [&window](int x, int y, const RGB& rgb) {
        window.set_pixel(POINT_EGDE_2D{ (float)x, (float)y, 1 }, rgb);
        };
    ctx.control_points = &control_points;

    // BSplineCurve definiton
    BSplineCurve bSplineCurve{ f,4 };
    GameWindow<std::any>::DRAR_FUNCTION draw_curve = [&window, &bSplineCurve, &control_points, &ctx]() {
        std::shared_lock lock(ctx.vec_mutex);
        bSplineCurve.drawCurve(sample_rate, control_points, generateRainbowColor);
        };

    // To use bezier curve, uncomment these and comment previous BSplineCurve definition
    //BezierCurve bezierCurve{ f };
    //GameWindow<std::any>::DRAR_FUNCTION draw_curve = [&window, &bezierCurve, &control_points, &ctx]() {
    //    std::shared_lock lock(ctx.vec_mutex);
    //    bezierCurve.drawCurve(sample_rate, control_points, generateRainbowColor);
    //    };

    GameWindow<std::any>::DRAR_FUNCTION draw_control_point = [&ctx, &window]() {
        std::shared_lock lock(ctx.vec_mutex);
        for (int i = 0; i < ctx.control_points->size(); i++)
        {
            auto& x = ctx.control_points[i];
            GAlgo::renderCircle((*ctx.control_points)[i],
                control_point_render_radio,
                std::bind(&FrameBuffer::set_pixel, &window, std::placeholders::_1, std::placeholders::_2),
                [](const RGB& whatever) {return RGB{ 255,255,255 }; });
        }
        };
    GameWindow<std::any>::DRAR_FUNCTIONS drawFunctions{ draw_control_point, draw_curve };
    window.set_draw_functions(std::move(drawFunctions));
    window.run();
    return 0;
}
