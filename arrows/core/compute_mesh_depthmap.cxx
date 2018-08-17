#include "compute_mesh_depthmap.h"

#include <arrows/core/atlas_processing.h>
#include <vital/types/camera_perspective.h>
#include <vital/types/camera_rpc.h>
#include <vital/types/geodesy.h>
#include <vital/types/image.h>

namespace kwiver {
namespace arrows {
namespace core {

compute_mesh_depthmap::compute_mesh_depthmap()
{

}

std::pair<vital::image_container_sptr, vital::image_container_sptr>
compute_mesh_depthmap::compute(vital::mesh_sptr mesh, kwiver::vital::camera_sptr camera,
                               int width, int height, int utm_zone)
{
    unsigned int nb_vertices = mesh->num_verts();

    kwiver::vital::mesh_vertex_array<3>& vertices = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(mesh->vertices());

    // project all points on image
    std::vector<vector_2d> points_uvs(nb_vertices);
    if (utm_zone > 0)
    {
        // if utm is valid, the mesh points are first transformed to lat/long coordinates
        int i=0;
        for (auto pt3d : vertices)
        {
            vital::vector_2d pt2d_latlong = vital::geo_conv({pt3d[0], pt3d[1]},
                                                            kwiver::vital::SRID::UTM_WGS84_north + utm_zone,
                                                            kwiver::vital::SRID::lat_lon_WGS84);
            points_uvs[i++] = camera->project({pt2d_latlong[0], pt2d_latlong[1], pt3d[2]});
        }
    }
    else
    {
        int i=0;
        for (auto pt3d : vertices)
        {
            points_uvs[i++] = camera->project(pt3d);
        }
    }

    // Compute the points depth
    std::vector<double> points_depth(nb_vertices);
    for (unsigned int i=0; i < vertices.size(); ++i)
    {
        points_depth[i] = camera->depth(vertices[i]);
    }

    // Initialize z_buffer with max double and id_buffer with -1
    kwiver::vital::image_of<double> z_buffer(width, height, 1);
    kwiver::vital::image_of<int> id_map(width, height, 1);
    for (int i=0; i < height; ++i)
    {
        for (int j=0; j < width; ++j)
        {
            z_buffer(j, i) = std::numeric_limits<double>::max();
            id_map(j, i) = -1;
        }
    }

    // Write faces on z_buffer and id_map with depth test
    vital::mesh_regular_face_array<3>& faces = dynamic_cast< vital::mesh_regular_face_array<3>& >(mesh->faces());
    for (unsigned int f_id=0; f_id < faces.size(); ++f_id)
    {
        vital::mesh_regular_face<3> f = faces[f_id];
        const vector_2d& a_uv = points_uvs[f[0]];
        const vector_2d& b_uv = points_uvs[f[1]];
        const vector_2d& c_uv = points_uvs[f[2]];

        // skip the face if the three points are outside the image
        if ((a_uv[0] < 0 || a_uv[0] >= width || a_uv[1] < 0 || a_uv[1] >= height) &&
            (b_uv[0] < 0 || b_uv[0] >= width || b_uv[1] < 0 || b_uv[1] >= height) &&
            (c_uv[0] < 0 || c_uv[0] >= width || c_uv[1] < 0 || c_uv[1] >= height))
            continue;

        double a_depth = points_depth[f[0]];
        double b_depth = points_depth[f[1]];
        double c_depth = points_depth[f[2]];

        // the rasterization is done over the face bounding box
        int u_min = static_cast<int>(std::floor(std::min(a_uv[0], std::min(b_uv[0], c_uv[0]))));
        int u_max = static_cast<int>(std::ceil(std::max(a_uv[0], std::max(b_uv[0], c_uv[0]))));
        int v_min = static_cast<int>(std::floor(std::min(a_uv[1], std::min(b_uv[1], c_uv[1]))));
        int v_max = static_cast<int>(std::ceil(std::max(a_uv[1], std::max(b_uv[1], c_uv[1]))));
        for (int v=v_min; v<=v_max; ++v)
        {
            for (int u=u_min; u<=u_max; ++u)
            {

                vector_2d p(u, v);
                // only compute depth for points inside the image and inside the triangle
                if (u >= 0 && u < width && v >= 0 && v < height && is_point_inside_triangle(p, a_uv, b_uv, c_uv))
                {
                    vector_3d bary_coords = barycentric_coordinates(p, a_uv, b_uv, c_uv);
                    double depth = bary_coords[0] * a_depth
                                   + bary_coords[1] * b_depth
                                   + bary_coords[2] * c_depth;
                    if (depth < z_buffer(u, v))
                    {
                        z_buffer(u, v) = depth;
                        id_map(u, v) = f_id;
                    }
                }
            }
        }
    }
//    vital::simple_image_container z_buffer_containercontainer(z_buffer);
    vital::image_container_sptr z_buffer_container(new vital::simple_image_container(z_buffer));
    vital::image_container_sptr id_map_container(new vital::simple_image_container(id_map));

    return std::pair<vital::image_container_sptr, vital::image_container_sptr>(z_buffer_container,
                                                                               id_map_container);
}

}
}
}
