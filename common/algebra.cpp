#include "algebra.hpp"
#include "logger.hpp"
#include "gasserts.hpp"

using namespace Eigen;
using namespace algebra;

Eigen::Matrix4f algebra::generateOrthogonalBasis(
    const Eigen::Vector4f& vector,
    const Eigen::Vector4f& point,
    int vector_offset)
{
    Matrix4f basis = Matrix4f::Zero();
    Vector4f normed_v = vector.normalized();
    Vector3f v = homogeneousToNormalCoordinate(normed_v);
    G_ASSERTS_TRUE(vector_offset >= 0 && vector_offset <=2 , "wrong vector offset");
    G_ASSERTS_TRUE(vector.w() == 0, "consider an vector rather point");
    G_ASSERTS_TRUE(v.norm() != 0, "consider a none zero");

    // a_1 * x_1 + a_2 * x_2 + a_3 * x_3 = 0
    // find the first none ele
    int next = (vector_offset + 1) %3;
    basis.col(vector_offset) = normed_v;
    basis.col(3) = point;

	Vector4f solution = Vector4f::Zero();
    //a2 * x2 + a3 * x3 = 0
    if (normed_v[0] == 0) {
        solution[0] = 1;
    }
    else {
        solution[1] = 1;
		solution[0] = normed_v[1] / - normed_v[0];
        solution.normalize();
    }
    basis.col(next) = solution;
    next = (next + 1) % 3;

    basis.col(next) = Eigen::Map<Vector3f>(basis.col(vector_offset).head<3>().data())
        .cross(Eigen::Map<Vector3f>(basis.col((vector_offset + 1) %3).head<3>().data()))
        .normalized()
        .homogeneous();
	basis.col(next)[3] = 0;

    return basis;
}

Eigen::Matrix4f algebra::generateBasisTransformationMatrix(const Eigen::Vector4f& vector,
    const Eigen::Vector4f& point,
    int vectorOffset)
{
	return generateOrthogonalBasis(vector, point, vectorOffset).inverse();
}

Eigen::Vector4f algebra::homogeneousNormalize(Eigen::Vector4f& point)
{
    float w = point.w();
    G_ASSERTS_TRUE(w != 0, "using point rather vector");

    if (w == 1) return point;
    
    point.x() /= w;
    point.y() /= w;
    point.z() /= w;
    point.w() = 1;
    return point;
}
