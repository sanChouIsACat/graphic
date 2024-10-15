#include "view.hpp"
#include "transformation.hpp"
#include "algebra.hpp"
#include "gtest/gtest.h"

using namespace Eigen;

// Orthogonal projection
TEST(OrthogonalProjectionTest, translateScaleTest) {
	Vector4f down_left(0, 0, -100, 1);
	Vector4f top_right(200, 200, -200, 1);

	Matrix4f projection = view::generateOrthogonalProjection(down_left, top_right);
	GTEST_LOG_(INFO) << "projection:\n" << projection;

	Vector4f origin(100, 100, -150, 1);
	EXPECT_EQ(projection * down_left, Vector4f(-1, -1, 1, 1));
	EXPECT_EQ(projection * top_right, Vector4f(1, 1, -1, 1));
	EXPECT_EQ(projection * origin, Vector4f(0,0,0,1));
}

// Perspective projection
TEST(PerspectiveProjectionTest, perspectiveTest) {
	Vector4f down_left(1, 1, -1, 1);
	Vector4f top_right(200, 200, -1, 1);

	Matrix4f projection = view::generatePerspectiveProjection(down_left, top_right, -200);
	GTEST_LOG_(INFO) << "projection:\n" << projection;


	Vector4f p(2, 2, -10, 1);
	Vector4f q(2, 2, -20, 1);
	Vector4f transformed_p = projection * p;
	algebra::homogeneousNormalize(transformed_p);
	Vector4f transformed_q = projection * q;
	algebra::homogeneousNormalize(transformed_q);
	EXPECT_EQ(1 , std::abs(transformed_p.y()-transformed_q.y()) < 1e-2);
	Vector4f far_point(200, 200, -200, 1);
	Vector4f transformed_far_point = projection * far_point;
	algebra::homogeneousNormalize(transformed_far_point);
	GTEST_LOG_(INFO) << "transformed_far_point:\n" << transformed_far_point;
	EXPECT_EQ(-1, transformed_far_point.z());

	Vector4f transformed_near_point = projection * Vector4f(10,10,-1,1);
	algebra::homogeneousNormalize(transformed_near_point);
	GTEST_LOG_(INFO) << "transformed_near_point:\n" << transformed_near_point;
	EXPECT_EQ(transformed_near_point.z(), 1);
}