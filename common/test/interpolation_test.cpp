#include "interpolation.hpp"
#include <gtest/gtest.h>

using namespace interpolation;
using namespace Eigen;

TEST(interpolationTest, insideTest) {
	std::array<Vector4f, 3> edges{
		Vector4f{-2, 0, 123, 1},
		Vector4f{2, 0, 123, 1},
		Vector4f{0, 3, 123, 1},
	};

	ASSERT_TRUE(insideTriangle(0, 1, edges));
	ASSERT_TRUE(insideTriangle(0, 2, edges));
}

TEST(interpolationTest, onEdgeTest) {
	std::array<Vector4f, 3> edges{
		Vector4f{-2, 0, 123, 1},
		Vector4f{2, 0, 123, 1},
		Vector4f{0, 2, 123, 1},
	};

	ASSERT_FALSE(insideTriangle(1, 0, edges));
}

TEST(interpolationTest, notInsideTest) {
	std::array<Vector4f, 3> edges{
		Vector4f{-2, -2, 123, 1},
		Vector4f{2, 2, 123, 1},
		Vector4f{0, 2, 123, 1},
	};

	ASSERT_FALSE(insideTriangle(2, 3, edges));
}
