#include "transformation.hpp"
#include "algebra.hpp"
#include <gtest/gtest.h>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace Eigen;
using namespace transformation;

// scale matrix
TEST(ScaleTransformationTest, simpleScaleMatrix) {
	Matrix4f scale_matrix = generateScaleMatrix(1, 2, 3);
	GTEST_LOG_(INFO) << "scale_matrix: \n" << scale_matrix;
	Matrix4f expected_matrix;
	expected_matrix << 1, 0, 0, 0,
		0, 2, 0, 0,
		0, 0, 3, 0,
		0, 0, 0, 1;
	ASSERT_EQ(scale_matrix, expected_matrix);
}

TEST(ScaleTransformationTest, decimalScaleMatrix) {
	Matrix4f scale_matrix = generateScaleMatrix(1.33333, 2.44444, 3.77777);
	GTEST_LOG_(INFO) << "scale_matrix: \n" << scale_matrix;
	Matrix4f expected_matrix;
	expected_matrix << 1.33333, 0, 0, 0,
		0, 2.44444, 0, 0,
		0, 0, 3.77777, 0,
		0, 0, 0, 1;
	ASSERT_EQ(scale_matrix, expected_matrix);
}

TEST(ScaleTransformationTest, bigNumberDecimalScaleMatrix) {
	Matrix4f scale_matrix = generateScaleMatrix(1000000.33333, 2.44444, 3.77777);
	GTEST_LOG_(INFO) << "scale_matrix: \n" << scale_matrix;
	Matrix4f expected_matrix;
	expected_matrix << 1000000.33333, 0, 0, 0,
		0, 2.44444, 0, 0,
		0, 0, 3.77777, 0,
		0, 0, 0, 1;
	ASSERT_EQ(scale_matrix, expected_matrix);
}

// for rotate
TEST(RotateTransformationTest, simpleRotateByZMatrix) {
	const float angle = 0.5;
	Matrix4f rotate_matrix = generateRotateByZMatrix(angle);
	GTEST_LOG_(INFO) << "rotate_matrix: \n" << rotate_matrix;
	
	const float angle_cos = std::cos(angle);
	const float angle_sin = std::sin(angle);
	Matrix4f expected_matrix;
	expected_matrix << angle_cos, -angle_sin, 0, 0,
		angle_sin, angle_cos, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	ASSERT_EQ(rotate_matrix, expected_matrix);
}

TEST(RotateTransformationTest, arbitraryRotateByZeroMatrix) {
	const float angle = 0.5;
	ASSERT_THROW({
	Matrix4f rotate_matrix = generateArbitraryRotateMatrix(Vector4f(0, 0, 0, 0), Vector4f(0, 0, 0, 0), angle);

		}, std::runtime_error);
}

TEST(RotateTransformationTest, arbitraryRotateByPointMatrix) {
	const float angle = 0.5;
	ASSERT_THROW({
	Matrix4f rotate_matrix = generateArbitraryRotateMatrix(Vector4f(0, 0, 0, 1), Vector4f(0, 0, 0, 1), angle);

		}, std::runtime_error);
}

TEST(RotateTransformationTest, arbitraryRotateByVectorMatrix) {
	const float angle = 0.5;
	ASSERT_THROW({
	Matrix4f rotate_matrix = generateArbitraryRotateMatrix(Vector4f(1, 0, 0, 0), Vector4f(1, 0, 0, 0), angle);

		}, std::runtime_error);
}

TEST(RotateTransformationTest, arbitraryRotateByZMatrix) {
	const float angle = 0.5;
	Matrix4f rotate_matrix = generateArbitraryRotateMatrix(Vector4f(0,0,1,0), Vector4f(0, 0, 0, 1), angle);
	GTEST_LOG_(INFO) << "rotate_matrix: \n" << rotate_matrix;

	const float angle_cos = std::cos(angle);
	const float angle_sin = std::sin(angle);
	Matrix4f expected_matrix;
	expected_matrix << angle_cos, -angle_sin, 0, 0,
		angle_sin, angle_cos, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	ASSERT_EQ(rotate_matrix, expected_matrix);
}

TEST(RotateTransformationTest, simpleArbitraryRotateMatrix) {
	const Vector4f direction(0, 0, 1, 0);
	const Vector4f rorate_point(0, 1, 0, 1);
	const float angle = M_PI / 2;
	Matrix4f rotate_matrix = generateArbitraryRotateMatrix(direction, rorate_point, angle);
	GTEST_LOG_(INFO) << "rotate_matrix: \n" << rotate_matrix;

	Vector4f v(0, 0, 0, 1);
	// Vector4f v(0.333, 0.333, 0.333, 0);
	const Matrix4f p = algebra::generateBasisTransformationMatrix(direction, Vector4f(0, 1, 0, 1), 2);
	Matrix4f expected_matrix = p.inverse() * generateRotateByZMatrix(angle) * p;
	
	// use isApprox rather == operator dut to float precision
	// ASSERT_EQ(rotate_matrix, p.inverse() * generateRotateByZMatrix(angle) * p);
	ASSERT_EQ(true, expected_matrix.isApprox(rotate_matrix));
}

TEST(RotateTransformationTest, bigNumberArbitraryRotateMatrix) {
	const Vector4f direction(100034, 0.334, 123, 0);
	const Vector4f rorate_point(33333, 55555, 0.1234, 1);
	const float angle = M_PI / 2;
	Matrix4f rotate_matrix = generateArbitraryRotateMatrix(direction, rorate_point, angle);
	GTEST_LOG_(INFO) << "rotate_matrix: \n" << rotate_matrix;

	const Vector4f v(33333, 55555, 0.1234, 1);
	ASSERT_EQ(true, (rotate_matrix * v).isApprox(v));
}

TEST(RotateTransformationTest, complicatedArbitraryRotateMatrix) {
	const Vector4f direction(1, 1, 1, 0);
	const Vector4f rorate_point(1, 3, 0, 1);
	const float angle = M_PI;
	Matrix4f rotate_matrix = generateArbitraryRotateMatrix(direction, rorate_point, angle);
	GTEST_LOG_(INFO) << "rotate_matrix: \n" << rotate_matrix;

	Vector4f v(1, 3, 0, 1);
	// it should identical dut to point is the origin in new coordinate system
	ASSERT_EQ(true, (rotate_matrix * v).isApprox(v));
}

TEST(RotateTransformationTest, nagtiveComplicatedArbitraryRotateMatrix) {
	const Vector4f direction(1, 1, 1, 0);
	const Vector4f rorate_point(1, -3, 0, 1);
	const float angle = M_PI;
	Matrix4f rotate_matrix = generateArbitraryRotateMatrix(direction, rorate_point, angle);
	GTEST_LOG_(INFO) << "rotate_matrix: \n" << rotate_matrix;

	Vector4f v(1, -3, 0, 1);
	// it should identical dut to point is the origin in new coordinate system
	ASSERT_EQ(true, (rotate_matrix * v).isApprox(v));
}



// translate 
TEST(TranslateTransformationTest, simpleTranslateMatrix) {
	Matrix4f translate_matrix = generateTranslateMatrix(1, 1, 1);
	GTEST_LOG_(INFO) << "rotate_matrix: \n" << translate_matrix;

	Matrix4f expected_matrix;
	expected_matrix << 0, 0, 0, 1,
		0, 0, 0, 1,
		0, 0, 0, 1,
		0, 0, 0, 1;
	ASSERT_EQ(true, expected_matrix.isApprox(translate_matrix));
}

TEST(TranslateTransformationTest, complicatedTranslateMatrix) {
	Matrix4f translate_matrix = generateTranslateMatrix(12345.567, 0.0001234, 4444.5567);
	GTEST_LOG_(INFO) << "rotate_matrix: \n" << translate_matrix;

	Matrix4f expected_matrix;
	expected_matrix << 0, 0, 0, 12345.567,
		0, 0, 0, 0.0001234,
		0, 0, 0, 4444.5567,
		0, 0, 0, 1;

	ASSERT_EQ(true, expected_matrix.isApprox(translate_matrix));
}



 

