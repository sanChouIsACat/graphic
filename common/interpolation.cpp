#include "interpolation.hpp"
#include "logger.hpp"
#include "gasserts.hpp"

using namespace Eigen;

bool interpolation::insideTriangle(float x, float y, const std::array<Eigen::Vector4f, 3>& edges)
{
    int res = 0;
    for (int i = 0; i < 3; i++)
    {
		G_ASSERTS_TRUE(edges[i].w() == 1,"should be point but vector");

        float ab_x = edges[i].x() - edges[(i + 1) % 3].x();
        float ab_y = edges[i].y() - edges[(i + 1) % 3].y();
        float qa_x = x - edges[i].x();
        float qa_y = y - edges[i].y();
        // ab cross qa
		float cross_res = ab_x * qa_y - ab_y * qa_x;
		G_LOGGER_TRACE("ab(%.2f,%.2f) corss qa(%.2f,%.2f) equals %f", ab_x, ab_y, qa_x, qa_y, cross_res);
        res += cross_res < 0 ? -1 : 1;
    }

    return std::abs(res) == 3;
}
