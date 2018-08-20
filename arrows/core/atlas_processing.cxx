#include "atlas_processing.h"

#include <vital/types/camera_rpc.h>
#include <vital/types/geodesy.h>
#include <vital/types/matrix.h>
#include <Eigen/Dense>

using namespace kwiver::vital;

namespace kwiver {
namespace arrows {
namespace core {


bool is_point_inside_triangle(const vector_2d& p,
                              const vector_2d& a,
                              const vector_2d& b,
                              const vector_2d& c)
{
    vector_2d AB = b - a;
    vector_2d AC = c - a;
    vector_2d AP = p - a;

    double inv_total_area = 1.0 / (AB[0]*AC[1]-AB[1]*AC[0]);
    double area_1 = inv_total_area * (AB[0]*AP[1]-AB[1]*AP[0]);
    double area_2 = inv_total_area * (AP[0]*AC[1]-AP[1]*AC[0]);

    return area_1>=0 && area_2>=0 && (area_1+area_2)<=1;
}

vector_3d barycentric_coordinates(const vector_2d& p,
                                  const vector_2d& a,
                                  const vector_2d& b,
                                  const vector_2d& c)
{
    matrix_3x3d abc;
    abc << a, b, c, 1, 1, 1;
    double det_inv = 1.0 / abc.determinant();

    vector_3d res;
    res(0) = ((b(1)-c(1)) * (p(0)-c(0)) - (b(0) - c(0)) * (p(1) - c(1))) * det_inv;
    res(1) = ((c(1)-a(1)) * (p(0)-c(0)) - (c(0) - a(0)) * (p(1) - c(1))) * det_inv;
    res(2) = 1.0 - res(0) - res(1);
    return res;
}

vital::image_container_sptr generate_triangles_map(kwiver::vital::mesh_sptr mesh,
                                                   unsigned int width, unsigned int height)
{

    // can handle separate triangles or adjacent triangles (with weak and strong assignment)

    // status_grid contains the status of each pixel:
    // 0: do not intersect any triangle
    // 1: the pixel center is inside a triangle (the pixel is be strongly assigned to this triangle, it will not changed)
    // 2: the pixel intersects partly with a triangle (the pixel is weakly assigned ti this triangle, it may change depending on the next triangles)
    // id_grid contains the current the triangle id assigned to each pixel

    image_of<unsigned char> status_grid(width, height);
    image_of<int> id_grid(width, height);
    for (int i=0; i < height; ++i)
    {
        for (int j=0; j < width; ++j)
        {
            status_grid(j, i) = 0;
            id_grid(j, i) = -1;
        }
    }

    const int number_of_faces = mesh->num_faces();
    const std::vector<vector_2d>& tcoords = mesh->tex_coords();
    for (int f=0; f < number_of_faces; ++f)
    {
        // get the face uv coordinates
        const vector_2d& a_uv = tcoords[f * 3 + 0];
        const vector_2d& b_uv = tcoords[f * 3 + 1];
        const vector_2d& c_uv = tcoords[f * 3 + 2];

        // get the 2d bounding box
        int u_min = static_cast<int>(std::floor(std::min(a_uv[0], std::min(b_uv[0], c_uv[0]))));
        int u_max = static_cast<int>(std::ceil(std::max(a_uv[0], std::max(b_uv[0], c_uv[0]))));
        int v_min = static_cast<int>(std::floor(std::min(a_uv[1], std::min(b_uv[1], c_uv[1]))));
        int v_max = static_cast<int>(std::ceil(std::max(a_uv[1], std::max(b_uv[1], c_uv[1]))));

        // map pixels to triangles
        for (int v=v_min; v<=v_max; ++v)
        {
            for (int u=u_min; u<=u_max; ++u)
            {
                vector_2d center(u, v);
                if (is_point_inside_triangle(center, a_uv, b_uv, c_uv))
                {
                    id_grid(u, v) = f;
                    status_grid(u, v) = 1;   // strong assignment

                }
                else
                {
                    bool partial_intersection = false;
                    // check partial intersections (done by checking 25 points
                    // regularly placed on the pixel)
                    for (float dy=-0.5; dy <= 0.5; dy+=0.25)
                    {
                        for (float dx=-0.5; dx <= 0.5; dx+=0.25)
                        {
                            if (is_point_inside_triangle(vector_2d(center[0]+dx, center[1]+dy), a_uv, b_uv, c_uv))
                            {
                                partial_intersection = true;
                                break;
                            }
                        }
                    }
                    if (partial_intersection && status_grid(u, v) != 1)
                    {   // if partial intersection and this pixel was not strongly assigned before
                        id_grid(u, v) = f;
                        status_grid(u, v) = 2;
                    }
                }
            }
        }
    }
    return image_container_sptr(new simple_image_container(id_grid));
}




}
}
}
