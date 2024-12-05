#include "interpolation.hpp"
#include <gtest/gtest.h>
#include "g_type_traits.hpp"

using namespace interpolation;
using namespace Eigen;

TEST(insideTriangleTEst, insideTest) {
	std::array<Vector4f, 3> edges{
		Vector4f{-2, 0, 123, 1},
		Vector4f{2, 0, 123, 1},
		Vector4f{0, 3, 123, 1},
	};

	ASSERT_TRUE(insideTriangle(0, 1, edges));
	ASSERT_TRUE(insideTriangle(0, 2, edges));
}

TEST(insideTriangleTEst, onEdgeTest) {
	std::array<Vector4f, 3> edges{
		Vector4f{-2, 0, 123, 1},
		Vector4f{2, 0, 123, 1},
		Vector4f{0, 2, 123, 1},
	};

	ASSERT_FALSE(insideTriangle(1, 0, edges));
}

TEST(insideTriangleTEst, notInsideTest) {
	std::array<Vector4f, 3> edges{
		Vector4f{-2, -2, 123, 1},
		Vector4f{2, 2, 123, 1},
		Vector4f{0, 2, 123, 1},
	};

	ASSERT_FALSE(insideTriangle(2, 3, edges));
}

TEST(computeBarycentric2DTest, edgeTest) {
	std::array<Vector4f, 3> edges{
		Vector4f{-2, 0, 0, 1},
		Vector4f{2, 0, 0, 1},
		Vector4f{-2, 3, 0, 1},
	};
	auto [c1, c2, c3] = computeBarycentric2D(1, 0, edges);
	ASSERT_FLOAT_EQ(c1, 0.25);
	ASSERT_FLOAT_EQ(c2, 0.75);
	ASSERT_FLOAT_EQ(c3, 0);
}

TEST(computeBarycentric2DTest, edgeTest2) {
	std::array<Vector4f, 3> edges{
		Vector4f{-2, 0, 0, 1},
		Vector4f{2, 0, 0, 1},
		Vector4f{-2, 3, 0, 1},
	};

	auto [c1, c2, c3] = computeBarycentric2D(-2, 1.5, edges);
	ASSERT_FLOAT_EQ(c1, 0.5);
	ASSERT_FLOAT_EQ(c2, 0);
	ASSERT_FLOAT_EQ(c3, 0.5);
}

TEST(computeBarycentric2DTest, insideTest) {
	std::array<Vector4f, 3> edges{
		Vector4f{-2, 0, 0, 1},
		Vector4f{2, 0, 0, 1},
		Vector4f{-2, 3, 0, 1},
	};

	auto [c1, c2, c3] = computeBarycentric2D(0, 1.5, edges);
	ASSERT_FLOAT_EQ(c1, 0);
	ASSERT_FLOAT_EQ(c2, 0.5);
	ASSERT_FLOAT_EQ(c3, 0.5);
}

TEST(computeBarycentricTest, insideTest) {
	Vector4f a = Vector4f{ -2, 0, 124, 1 };
	Vector4f b = Vector4f{ 2, 0, 145, 1 };
	Vector4f c = Vector4f{ -2, 3, 82, 1 };
	Vector4f edges[3]{a,b,c};
	Vector4f point = 0.6 * (b - a) + 0.6 * (c - a) + a;
	GTEST_LOG_(INFO) <<"da:\n" << point - a;
	GTEST_LOG_(INFO) <<"ba:\n" << b - a;
	GTEST_LOG_(INFO) <<"ca:\n" << c - a;
	auto [c1, c2, c3] = computeBarycentric(point, edges);
	ASSERT_NEAR(c1, -0.2, 1e-03);
	ASSERT_NEAR(c2, 0.6, 1e-03);
	ASSERT_NEAR(c3, 0.6, 1e-03);
}

TEST(computeBarycentricTest, notInsideTest) {
	Vector4f a = Vector4f{ -2, 0, 124, 1 };
	Vector4f b = Vector4f{ 2, 0, 145, 1 };
	Vector4f c = Vector4f{ -2, 3, 82, 1 };
	Vector4f edges[3]{ a,b,c };
	Vector4f point = (b - a).cross3(c - a) + a;
	auto [c1, c2, c3] = computeBarycentric(point, edges);
	ASSERT_NEAR(c1, -1, 1e-03);
	ASSERT_NEAR(c2, -1, 1e-03);
	ASSERT_NEAR(c3, -1, 1e-03);
}


//line draw testing
void draw_frame(int* frame, int height, int width) {
	std::string raster_buffer = "\n";

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (frame[i * width + j] == 1) {
				raster_buffer.append("\033[32m*\033[0m");
			}
			else {
				raster_buffer.append("*");
			}
		}
		raster_buffer.append("\n");
	}
	GTEST_LOG_(INFO) << raster_buffer;
}

TEST(tanxDrawLineTest, horizontalTest) {
	Vector3f begin{ 1,2,0 };
	Vector3f end{ 7,2,0 };
	int constexpr width = 10;
	int constexpr height = 10;
	int frame[width * height];
	tanx_line_draw(begin, end, [&frame](const Vector3f& a,const Vector3f& b){
		frame[(int)(a.y() * 10 + a.x())] = 1;
	});

	draw_frame(frame, height, width);
	for (int i = 1; i < 8; i++)
	{
		ASSERT_TRUE(frame[(int)(2 * height + i)]);
	}
	
}

TEST(tanxDrawLineTest, vertialTest) {
	Vector3f begin{ 0,0,0 };
	Vector3f end{ 0,9,0 };
	int constexpr width = 10;
	int constexpr height = 10;
	int frame[width * height];
	tanx_line_draw(begin, end, [&frame](const Vector3f& a, const Vector3f& b) {
		frame[(int)(a.y() * 10 + a.x())] = 1;
		});

	draw_frame(frame, height, width);
	for (int i = 0; i < 10; i++)
	{
		ASSERT_TRUE(frame[(int)(i * height)]);
	}
}

TEST(tanxDrawLineTest, normalTest1) {
	Vector3f begin{ 0,0,0 };
	Vector3f end{ 9,9,0 };
	int constexpr width = 10;
	int constexpr height = 10;
	int frame[width * height];
	tanx_line_draw(begin, end, [&frame](const Vector3f& a, const Vector3f& b) {
		frame[(int)(a.y() * 10 + a.x())] = 1;
		});

	draw_frame(frame, height, width);
}
