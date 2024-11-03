//
// Created by LEI XU on 4/11/19.
//

#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H

#include <eigen3/Eigen/Eigen>
#include "Texture.hpp"
#include "types.hpp"

using namespace Eigen;
class Triangle{

public:
    POINT_EGDE_3D v[3]; /*the original coordinates of the triangle, v0, v1, v2 in counter clockwise order*/
    /*Per vertex values*/
    RGB color[3]; //color at each vertex;
    POINT_EGDE_2D tex_coords[3]; //texture u,v
    POINT_EGDE_3D normal[3]; //normal vector for each vertex

    const Texture * tex= nullptr;
    Triangle();

    Eigen::Vector4f a() const { return v[0]; }
    Eigen::Vector4f b() const { return v[1]; }
    Eigen::Vector4f c() const { return v[2]; }

    void setVertex(int ind, POINT_EGDE_3D ver); /*set i-th vertex coordinates */
    void setNormal(int ind, POINT_EGDE_3D n); /*set i-th vertex normal vector*/
    void setColor(int ind, float r, float g, float b); /*set i-th vertex color*/

    void setNormals(const std::array<POINT_EGDE_3D, 3>& normals);
    void setColors(const std::array<RGB, 3>& colors);
    void setTexCoord(int ind,POINT_EGDE_2D uv ); /*set i-th vertex texture coordinate*/
    void setTexture(const Texture& texture) {
        tex = &texture;
    }
    std::array<POINT_EGDE_3D, 3> toVector4() const;
};






#endif //RASTERIZER_TRIANGLE_H
