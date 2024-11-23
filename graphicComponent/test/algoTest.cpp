#include <gtest/gtest.h>
#include "algo.hpp"
#include "bezierCurve.hpp"

using namespace GAlgo;
std::string getAnsiColorCode(const RGB& r) {
    // ANSI ÑÕÉ«Âë¸ñÊ½: "\033[38;2;<r>;<g>;<b>m"
	return "\033[38;2;" + std::to_string((int)r.x()) +
		";" + std::to_string((int)r.y()) +
		";" + std::to_string((int)r.z()) + "m*\033[0m";
}


void printScreen(const std::vector<RGB>& screen, int width, int height) {
	std::string line = "";
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			line += getAnsiColorCode(screen[i * width + j]);
		}
		line += "\n";
	}
	GTEST_LOG_(INFO) << "buffer(ASNI):\n" << line;
};

constexpr int width = 50;
constexpr int height = 5;
RGB white{ 255,255,255 };
bool isApproxEqual(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, float epsilon = 1e-5) {
	return (v1 - v2).norm() < epsilon;
}
std::vector<RGB> drawBezierCurve(const std::vector<POINT_EGDE_2D>& control_points) {
	std::vector<RGB> screen{ width * height,RGB{255,255,255} };
	auto drawPixel = [&screen](int x, int y, const RGB& RGB) {
		int poz = y * width + x;
		ASSERT_TRUE(poz < width * height);
		screen[poz] = RGB;
		};
	BezierCurve curveGenerator{ drawPixel };
	curveGenerator.drawCurve(0.01, control_points, generateRainbowColor);
	return screen;
}

TEST(BezierCurveTest, LineTest) {
	std::vector control_points{ POINT_EGDE_2D{0,0,1}, POINT_EGDE_2D{20,0,1} };
	std::vector<RGB> screen = drawBezierCurve(control_points);
	printScreen(screen, width, height);
	for (int i = 0; i < 21; ++i) {
		ASSERT_FALSE(isApproxEqual(screen[i],white)) << "differ at idx" << i;
	}
	for (int i = 21; i < width * height; i++) {
		ASSERT_TRUE(isApproxEqual(screen[i], white)) << "differ at idx" << i;
	}
}

TEST(BezierCurveTest, FourBezierCurveTest) {
	std::vector control_points{ POINT_EGDE_2D{0,4,1}, POINT_EGDE_2D{0,0,1}, POINT_EGDE_2D{49,0,1}, POINT_EGDE_2D{49,4,1} };
	std::vector<RGB> screen = drawBezierCurve(control_points);
	printScreen(screen, width, height);
	std::vector<Eigen::Vector2i> standard_coors{ {0, 4}, {0, 3}, {1, 3}, {2, 3}, {3, 3}, {3, 2}, {4, 2}, {5, 2}, {6, 2}, {7, 2}, {8, 2}, {9, 2}, {10, 2}, {11, 1}, {12, 1}, {13, 1}, {14, 1}, {15, 1}, {16, 1}, {17, 1}, {18, 1}, {19, 1}, {20, 1}, {21, 1}, {22, 1}, {23, 1}, {24, 1}, {25, 1}, {26, 1}, {27, 1}, {28, 1}, {29, 1}, {30, 1}, {31, 1}, {32, 1}, {33, 1}, {34, 1}, {35, 1}, {36, 1}, {37, 1}, {38, 1}, {39, 2}, {40, 2}, {41, 2}, {42, 2}, {43, 2}, {44, 2}, {45, 2}, {46, 2}, {46, 3}, {47, 3}, {48, 3}, {49, 3}, {49, 4} };
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			auto iterator = std::find(standard_coors.begin(),
				standard_coors.end(),
				Eigen::Vector2i{j, i});
			int idx = i * width + j;
			if (standard_coors.end() == iterator) {
				ASSERT_TRUE(isApproxEqual(screen[idx], white)) << "differ at idx" << idx;
			}
			else {
				ASSERT_FALSE(isApproxEqual(screen[idx], white)) << "differ at idx" << idx;
			}
		}
	}
	
}