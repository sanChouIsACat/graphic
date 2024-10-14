#pragma once

#include <eigen3/Eigen/Eigen>

namespace transformation {
    // generate a scale matrix
    Eigen::Matrix4f generateScaleMatrix(float scale_x, float scale_y, float scale_z);

    /*
    * generate a translation matrix, the angle using pi
    */
    Eigen::Matrix4f generateRotateByZMatrix(float angle);

    /*
    *the direction is the axis of rotation, and the angle is the angle of rotation. the angle using pi
    */
    Eigen::Matrix4f generateArbitraryRotateMatrix(const Eigen::Vector4f direction, const Eigen::Vector4f point, float angle);

    // generate a translation matrix
    Eigen::Matrix4f generateTranslateMatrix(float translate_x, float translate_y, float translate_z);
}
