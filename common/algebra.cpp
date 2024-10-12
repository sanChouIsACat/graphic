#include "algebra.hpp"
#include "logger.hpp"
#include "gasserts.hpp"

using namespace Eigen;
using namespace algebra;

Eigen::Matrix4f algebra::generateOrthogonal3DBasis(
    const Eigen::Vector4f & vector, const Eigen::Vector4f& point)
{
    Matrix4f basis = Matrix4f::Identity();
    Vector4f normedV = vector.normalized();
    Vector3f v = homogeneousToNormal(normedV);
    G_ASSERTS_TRUE(vector.w() == 0, "consider an vector rather point");
    G_ASSERTS_TRUE(v.norm() != 0, "consider a none zero");

    int j = 0;
    for (j = 0; j < 3 && !v[j]; j++);
    basis.col(j) = normedV;
    basis.col(3) = point;
    
    for (int i = 0; i < 3; i++) {
        if (i == j) continue;
        Vector4f e = Vector4f::Zero();
		e[i] = 1;
		e[j] = - v[i] / v[j];
        e.normalize();
		basis.col(i) = e;
    }

    return basis;
}
