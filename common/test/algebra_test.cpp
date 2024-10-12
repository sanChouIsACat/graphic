#include"algebra.hpp"
#include<gtest/gtest.h>

using namespace algebra;
using namespace Eigen;

void checkOrthogonal3DBasis(const Matrix4f& basis,
    const Vector4f& originV,
    const Vector4f& point) {
    bool sameLine = false;
    for (int i = 0; i < 3; i++)
    {
        Vector3f a = homogeneousToNormal(basis.col(i));
		sameLine |= a.dot(homogeneousToNormal(originV)) == 0; // 检查现在的基是否有和原来的向量在同一直线上
        EXPECT_FLOAT_EQ(1, a.norm()); // 每个向量是否单位化
        for (int j = i; j < i; j++) {
            Vector3f b = homogeneousToNormal(basis.col(j));
            EXPECT_EQ(0, a.dot(b)); // 每个向量是否正交
        }
    }

    EXPECT_EQ(true, point.isApprox(basis.col(3))); // 点是否平移
    EXPECT_EQ(true, sameLine); // 检查现在的基是否有和原来的向量在同一直线上
}

TEST(generateOrthogonal3DBasisTest, originBasis) {
    // 输入齐次坐标系下的原点以及1,0,0
   Vector4f origin(0, 0, 0, 1);
   Vector4f x(1, 0, 0, 0);

    // 调用 generateOrthogonal3DBasis 函数生成正交基
    const Matrix4f& basis = generateOrthogonal3DBasis(x, origin);
    GTEST_LOG_(INFO) << "basis: " << "\n" << basis;
	checkOrthogonal3DBasis(basis, x, origin);
}

TEST(generateOrthogonal3DBasisTest, originBasis2) {
	// 输入齐次坐标系下的原点以及0,1,0
	Vector4f origin(1, -0.5, -0.655, 1);
	Vector4f y(0, 1, 0, 0);

	// 调用 generateOrthogonal3DBasis 函数生成正交基
	const Matrix4f& basis = generateOrthogonal3DBasis(y, origin);
	GTEST_LOG_(INFO) << "basis: " << "\n" << basis;
	checkOrthogonal3DBasis(basis, y, origin);
}

TEST(generateOrthogonal3DBasisTest, originBasis3) {
	// 输入齐次坐标系下的原点以及0,0,1
	Vector4f origin(1, -0.5, -0.655, 1);
	Vector4f z(0, 0, 1, 0);

	// 调用 generateOrthogonal3DBasis 函数生成正交基
	const Matrix4f& basis = generateOrthogonal3DBasis(z, origin);
	GTEST_LOG_(INFO) << "basis: " << "\n" << basis;
	checkOrthogonal3DBasis(basis, z, origin);
}

TEST(generateOrthogonal3DBasisTest, originBasis4) {
    // 输入齐次坐标系下的原点以及1,1,1
    Vector4f origin(0, 0, 0, 1);
    Vector4f v(1, 1, 1, 0);

    // 调用 generateOrthogonal3DBasis 函数生成正交基
    const Matrix4f& basis = generateOrthogonal3DBasis(v, origin);
    GTEST_LOG_(INFO) << "basis: " << "\n" << basis;
    checkOrthogonal3DBasis(basis, v, origin);
}

TEST(generateOrthogonal3DBasisTest, originBasis5) {
    // 输入齐次坐标系下的原点以及-1,2,3
    Vector4f origin(0, 0, 0, 1);
    Vector4f v(-1, 2, 3, 0);

    // 调用 generateOrthogonal3DBasis 函数生成正交基
    const Matrix4f& basis = generateOrthogonal3DBasis(v, origin);
    GTEST_LOG_(INFO) << "basis: " << "\n" << basis;
    checkOrthogonal3DBasis(basis, v, origin);
}

TEST(generateOrthogonal3DBasisTest, originBasis6) {
    // 输入齐次坐标系下的原点以及-1,0,1
    Vector4f origin(0, 0, 0, 1);
    Vector4f v(-1, 0, 1, 0);

    // 调用 generateOrthogonal3DBasis 函数生成正交基
    const Matrix4f& basis = generateOrthogonal3DBasis(v, origin);
    GTEST_LOG_(INFO) << "basis: " << "\n" << basis;
    checkOrthogonal3DBasis(basis, v, origin);
}

TEST(generateOrthogonal3DBasisTest, originBasis7) {
    // 输入齐次坐标系下的原点以及0,-1,1
    Vector4f origin(0, 0, 0, 1);
    Vector4f v(0, -1, 1, 0);

    // 调用 generateOrthogonal3DBasis 函数生成正交基
    const Matrix4f& basis = generateOrthogonal3DBasis(v, origin);
    GTEST_LOG_(INFO) << "basis: " << "\n" << basis;
    checkOrthogonal3DBasis(basis, v, origin);
}

TEST(generateOrthogonal3DBasisTest, originBasis8) {
    // 输入齐次坐标系下的原点以及0,-1,1
    Vector4f origin(3.3, 4.4, 9999, 1);
    Vector4f v(10000, 2, 5000, 0);

    // 调用 generateOrthogonal3DBasis 函数生成正交基
    const Matrix4f& basis = generateOrthogonal3DBasis(v, origin);
    GTEST_LOG_(INFO) << "basis: " << "\n" << basis;
    checkOrthogonal3DBasis(basis, v, origin);
}
