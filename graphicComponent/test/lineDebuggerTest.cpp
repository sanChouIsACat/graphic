#include "lineDebugger.hpp"
#include <gtest/gtest.h>
#include "Triangle.hpp"
#include "Line.hpp"
TEST(lineDebuggerTest, start) {
	;
	LineDebugger lineDebugger{ 800,
		800,
		GTypes::AABB{ {50,-50,0,0},{-50,50,-100,1} },
		"test2",
		{0,0,0,1} };
	GTypes::Line line{ POINT_EGDE_3D{-1,-1,-1,1},POINT_EGDE_3D{-45,-45,-99,1} };
	lineDebugger.addLinePrimitives(line);
	lineDebugger.start();

}