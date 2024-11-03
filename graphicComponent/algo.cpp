#include "algo.hpp"

std::tuple<float, float, float, float> g_algo::getRoundingBox(const Triangle& tri)
{
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::min();
    float max_y = std::numeric_limits<float>::min();

    for (const Vector4f& edge : tri.v) {

        float x = edge.x();
        float y = edge.y();
        min_x = min_x < x ? min_x : x;
        min_y = min_y < y ? min_y : y;
        max_x = max_x > x ? max_x : x;
        max_y = max_y > y ? max_y : y;
    }
	return std::make_tuple(min_x, min_y, max_x, max_y);
}
