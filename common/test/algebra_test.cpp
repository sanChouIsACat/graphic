#include"algebra.hpp"
#include<gtest/gtest.h>

using namespace algebra;
using namespace Eigen;

void checkOrthogonal3DBasis(const Matrix4f& basis,
    const Vector4f& origin_v,
    const Vector4f& point, int offset) {
    for (int i = 0; i < 3; i++)
    {
        Vector3f a = homogeneousToNormalCoordinate(basis.col(i));
        EXPECT_FLOAT_EQ(1, a.norm()); // 每个向量是否单位化
        for (int j = i + 1; j < 3; j++) {
            Vector3f b = homogeneousToNormalCoordinate(basis.col(j));
            EXPECT_EQ(0, a.dot(b)); // 每个向量是否正交
        }
    }
    EXPECT_EQ(basis.col(3), point); // 检查点是否正确
    EXPECT_FLOAT_EQ(1, basis.col(offset).dot(origin_v.normalized())); // 检查原来的向量是否在z轴上

}

TEST(generateOrthogonal3DBasisTest, falseArgument) {
    // 输入齐次坐标系下的原点以及1,0,0
    Vector4f origin(0, 0, 0, 1);
    Vector4f x(1, 0, 0, 0);

    // 调用 generateOrthogonal3DBasis 函数生成正交基
    ASSERT_DEATH({
    int* p = nullptr;
    *p = 1;  // 这会导致崩溃
        }, "");
    ASSERT_THROW({
        const Matrix4f& basis = generateOrthogonalBasis(origin, x, 2);
},std::runtime_error);
}

TEST(generateOrthogonal3DBasisTest, originBasis) {
    // 输入齐次坐标系下的原点以及1,0,0
   Vector4f origin(0, 0, 0, 1);
   Vector4f x(1, 0, 0, 0);

    // 调用 generateOrthogonal3DBasis 函数生成正交基
    const Matrix4f& basis = generateOrthogonalBasis(x, origin, 2);
    GTEST_LOG_(INFO) << "basis: " << "\n" << basis;
	checkOrthogonal3DBasis(basis, x, origin, 2);
}

TEST(generateOrthogonal3DBasisTest, originBasis2) {
	// 输入齐次坐标系下的原点以及0,1,0
	Vector4f origin(1, -0.5, -0.655, 1);
	Vector4f y(0, 1, 0, 0);

	// 调用 generateOrthogonal3DBasis 函数生成正交基
	const Matrix4f& basis = generateOrthogonalBasis(y, origin, 2);
	GTEST_LOG_(INFO) << "basis: " << "\n" << basis;
	checkOrthogonal3DBasis(basis, y, origin, 2);
}

TEST(generateOrthogonal3DBasisTest, originBasis3) {
	// 输入齐次坐标系下的原点以及0,0,1
	Vector4f origin(1, -0.5, -0.655, 1);
	Vector4f z(0, 0, 1, 0);

	// 调用 generateOrthogonal3DBasis 函数生成正交基
	const Matrix4f& basis = generateOrthogonalBasis(z, origin, 2);
	GTEST_LOG_(INFO) << "basis: " << "\n" << basis;
	checkOrthogonal3DBasis(basis, z, origin, 2);
}

TEST(generateOrthogonal3DBasisTest, originBasis4) {
    // 输入齐次坐标系下的原点以及1,1,1
    Vector4f origin(0, 0, 0, 1);
    Vector4f v(1, 1, 1, 0);

    // 调用 generateOrthogonal3DBasis 函数生成正交基
    const Matrix4f& basis = generateOrthogonalBasis(v, origin, 2);
    GTEST_LOG_(INFO) << "basis: " << "\n" << basis;
    checkOrthogonal3DBasis(basis, v, origin, 2);
}

TEST(generateOrthogonal3DBasisTest, originBasis5) {
    // 输入齐次坐标系下的原点以及-1,2,3
    Vector4f origin(0, 0, 0, 1);
    Vector4f v(-1, 2, 3, 0);

    // 调用 generateOrthogonal3DBasis 函数生成正交基
    const Matrix4f& basis = generateOrthogonalBasis(v, origin, 2);
    GTEST_LOG_(INFO) << "basis: " << "\n" << basis;
    checkOrthogonal3DBasis(basis, v, origin, 2);
}

TEST(generateOrthogonal3DBasisTest, originBasis6) {
    // 输入齐次坐标系下的原点以及-1,0,1
    Vector4f origin(0, 0, 0, 1);
    Vector4f v(-1, 0, 1, 0);

    // 调用 generateOrthogonal3DBasis 函数生成正交基
    const Matrix4f& basis = generateOrthogonalBasis(v, origin, 2);
    GTEST_LOG_(INFO) << "basis: " << "\n" << basis;
    checkOrthogonal3DBasis(basis, v, origin, 2);
}

TEST(generateOrthogonal3DBasisTest, originBasis7) {
    // 输入齐次坐标系下的原点以及0,-1,1
    Vector4f origin(0, 0, 0, 1);
    Vector4f v(0, -1, 1, 0);

    // 调用 generateOrthogonal3DBasis 函数生成正交基
    const Matrix4f& basis = generateOrthogonalBasis(v, origin, 2);
    GTEST_LOG_(INFO) << "basis: " << "\n" << basis;
    checkOrthogonal3DBasis(basis, v, origin, 2);
}

TEST(generateOrthogonal3DBasisTest, originBasis8) {
    // 输入齐次坐标系下的原点以及0,-1,1
    Vector4f origin(3.3, 4.4, 9999, 1);
    Vector4f v(10000, 2, 5000, 0);

    // 调用 generateOrthogonal3DBasis 函数生成正交基
    const Matrix4f& basis = generateOrthogonalBasis(v, origin, 2);
    GTEST_LOG_(INFO) << "basis: " << "\n" << basis;
    checkOrthogonal3DBasis(basis, v, origin, 2);
}

TEST(homogeneousNormalizeTest, twoToOne) {
	Vector4f point(2, 4, 6, 2);
	Vector4f result = homogeneousNormalize(point);
	GTEST_LOG_(INFO) << "result: " << result;
	EXPECT_FLOAT_EQ(1, result.w());
	EXPECT_FLOAT_EQ(1, result.x());
	EXPECT_FLOAT_EQ(2, result.y());
	EXPECT_FLOAT_EQ(3, result.z());
}
