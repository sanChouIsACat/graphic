#include "view.hpp"
#include "transformation.hpp"
#include "gasserts.hpp"

using namespace Eigen;
Eigen::Matrix4f view::generateOrthogonalProjection(const Eigen::Vector4f& down_left, const Eigen::Vector4f& top_right)
{
	float near = down_left.z();
	float far = top_right.z();
	G_ASSERTS_TRUE(near < 0 && far < 0, "n and f should be nagetive");
	G_ASSERTS_TRUE(near > far, "near should be bigger than far");

	return transformation::generateWindowingMatrix(down_left,
		top_right,
		Vector4f(-1, -1, 1, 1),
		Vector4f(1, 1, -1, 1)
		);
}

Eigen::Matrix4f view::generatePerspectiveProjection(const Eigen::Vector4f& down_left, const Eigen::Vector4f& top_right, float far)
{
	float n = down_left.z();

	G_ASSERTS_TRUE(n < 0 && far < 0, "n and f should be nagetive");
	G_ASSERTS_TRUE(n > far, "near should be bigger than far");
	G_ASSERTS_TRUE(std::abs(down_left.z() - top_right.z()) < 1e-9, "near should be bigger than far");
	Matrix4f p = Eigen::Matrix4f::Zero();
	p << n, 0, 0, 0,
		0, n, 0, 0,
		0, 0, n + far, -n * far,
		0, 0, 1, 0;
	G_LOGGER_TRACE("p:\n%s", EigenStructToString(p).c_str());
	Vector4f new_top_right(top_right);
	new_top_right.z() = far;
	G_LOGGER_TRACE("new_top_right :\n%s", EigenStructToString(new_top_right).c_str());
	Matrix4f res = generateOrthogonalProjection(down_left, new_top_right)* p;
	G_LOGGER_TRACE("perspective matrix:\n%s", EigenStructToString(res).c_str());
	return res;
}
