#pragma once

#include <eigen3/Eigen/Eigen>
namespace transformation {
    Eigen::Matrix4d generateScaleMatrix(double scale_x,
        double scale_y,
        double scale_z)
    {
        Eigen::Matrix4d scale_matrix;
        scale_matrix << scale_x, 0, 0, 0,
            0, scale_y, 0, 0,
            0, 0, scale_z, 0,
            0, 0, 0, 1;
        return scale_matrix;
    }
 //   Eigen::Matrix4d generateRotateMatrix(double angle)
	//{
	//	Eigen::Matrix4d translation_matrix;
	//	translation_matrix << 1, 0, 0, x,
	//		0, 1, 0, y,
	//		0, 0, 1, z,
	//		0, 0, 0, 1;
	//	return translation_matrix;
	//}
}

