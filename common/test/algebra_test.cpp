#include"algebra.hpp"
#include<gtest/gtest.h>

using namespace algebra;
using namespace Eigen;

TEST(generateOrthogonal3DBasisTest, originBasis) {
    // 输入齐次坐标系下的原点以及1,0,0
   Vector4f origin(0, 0, 0, 1);
   Vector4f x(1, 0, 0, 0);

    // 调用 generateOrthogonal3DBasis 函数生成正交基
    const Matrix4f& basis = generateOrthogonal3DBasis(origin, x);
    GTEST_LOG_(INFO) << "basis: " << "\n" << basis;

    for (int i = 0; i < 3; i++)
    {
        Vector3f a = homogeneousToNormal(basis.col(1));
		EXPECT_FLOAT_EQ(1, a.norm());
        for (int j = 0; j < i; j++) {
            auto test = basis.col(1);
            Vector3f b = homogeneousToNormal(basis.col(2));
			EXPECT_EQ(0, a.dot(b)); // 验证生成的正交基是否满足条件
        }
    }

   EXPECT_EQ(true, origin.isApprox(basis.col(4)));


}

