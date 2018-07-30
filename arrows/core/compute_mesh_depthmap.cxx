#include "compute_mesh_depthmap.h"

#include <vital/types/camera_perspective.h>
#include <vital/types/image.h>
#include <vital/plugin_loader/plugin_manager.h>

#include <Eigen/Dense>
#include <typeinfo>


namespace kwiver {
namespace arrows {
namespace core {

compute_mesh_depthmap::compute_mesh_depthmap()
{

}

bool is_point_inside_triangle(const Eigen::Vector2d& p,
                              const Eigen::Vector2d& a,
                              const Eigen::Vector2d& b,
                              const Eigen::Vector2d& c)
{
    Eigen::Vector2d AB = b - a;
    Eigen::Vector2d AC = c - a;
    Eigen::Vector2d AP = p - a;

    double inv_total_area = 1.0 / (AB[0]*AC[1]-AB[1]*AC[0]);
    double area_1 = inv_total_area * (AB[0]*AP[1]-AB[1]*AP[0]);
    double area_2 = inv_total_area * (AP[0]*AC[1]-AP[1]*AC[0]);

    return area_1>=0 && area_2>=0 && (area_1+area_2)<=1;
}


Eigen::Vector3d barycentric_coordinates(const Eigen::Vector2d& p,
                                        const Eigen::Vector2d& a,
                                        const Eigen::Vector2d& b,
                                        const Eigen::Vector2d& c)
{
    Eigen::Matrix3d abc;
    abc << a, b, c, 1, 1, 1;
    double det_inv = 1.0 / abc.determinant();

    Eigen::Vector3d res;
    res(0) = ((b(1)-c(1)) * (p(0)-c(0)) - (b(0) - c(0)) * (p(1) - c(1))) * det_inv;
    res(1) = ((c(1)-a(1)) * (p(0)-c(0)) - (c(0) - a(0)) * (p(1) - c(1))) * det_inv;
    res(2) = 1.0 - res(0) - res(1);
    return res;
}

std::pair<vital::image_container_sptr, vital::image_container_sptr>
compute_mesh_depthmap::compute(vital::mesh_sptr mesh, kwiver::vital::camera_sptr camera,
                               int width, int height)
{
    kwiver::vital::plugin_manager::instance().load_all_plugins();

    int utm_zone = 0;
    // --- project points on camera ---
    // create a vector of mesh points
    unsigned int nb_vertices = mesh->num_verts();
//    std::vector<Eigen::Vector3d> vertices(nb_vertices);
//    double tmp[3];
//    for (unsigned int i=0; i < nb_vertices; ++i)
//    {
//        mesh->GetPoint(i, tmp);
//        vertices[i] = Eigen::Map<Eigen::Vector3d>(tmp);
//    }

    kwiver::vital::mesh_vertex_array<3>& vertices = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(mesh->vertices());

    // project all points
    std::vector<Eigen::Vector2d> points_uvs(nb_vertices);
    if (utm_zone > 0)
    {
//        // it utm is valid, the mesh points are first transformed to latlong coordinates
//        std::vector<Eigen::Vector3d> vertices_latlong(nb_vertices);
//        convert_points_from_UTM_to_WGS84(vertices, vertices_latlong, utm_zone);
//        camera.project_points(vertices_latlong, points_uvs);
    }
    else
    {
        int i=0;
        for (auto pt3d : vertices)
        {
            points_uvs[i++] = camera->project(pt3d);
            std::cout << points_uvs[i-1] << std::endl;
        }
    }

    // Compute the points depth
    std::vector<double> points_depth(nb_vertices);
//    std::pair<Eigen::Vector3d, Eigen::Vector3d> axis_and_position = camera.get_optical_axis_and_position(utm_zone);
//    Eigen::Vector3d axis = axis_and_position.first;
//    Eigen::Vector3d position = axis_and_position.second;    // a possible camera position
    for (int i=0; i < vertices.size(); ++i)
    {
        points_depth[i] = dynamic_cast<kwiver::vital::camera_perspective*>(camera.get())->depth(vertices[i]);
        std::cout << "depth " << points_depth[i] << std::endl;
    }

    // Initialize the z_buffer with max double and id_map with -1
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

    vital::mesh_regular_face_array<3>& faces = dynamic_cast< vital::mesh_regular_face_array<3>& >(mesh->faces());

    for (int f_id=0; f_id < faces.size(); ++f_id)
    {
        vital::mesh_regular_face<3> f = faces[f_id];
        const Eigen::Vector2d& a_uv = points_uvs[f[0]];
        const Eigen::Vector2d& b_uv = points_uvs[f[1]];
        const Eigen::Vector2d& c_uv = points_uvs[f[2]];

        double a_depth = points_depth[f[0]];
        double b_depth = points_depth[f[1]];
        double c_depth = points_depth[f[2]];


        int u_min = static_cast<int>(std::floor(std::min(a_uv[0], std::min(b_uv[0], c_uv[0]))));
        int u_max = static_cast<int>(std::ceil(std::max(a_uv[0], std::max(b_uv[0], c_uv[0]))));
        int v_min = static_cast<int>(std::floor(std::min(a_uv[1], std::min(b_uv[1], c_uv[1]))));
        int v_max = static_cast<int>(std::ceil(std::max(a_uv[1], std::max(b_uv[1], c_uv[1]))));

        for (int v=v_min; v<=v_max; ++v)
        {
            for (int u=u_min; u<=u_max; ++u)
            {

                Eigen::Vector2d p(u, v);
                // only compute depth for points inside the image and inside the triangle
                if (u >= 0 && u < width && v >= 0 && v < height && is_point_inside_triangle(p, a_uv, b_uv, c_uv))
                {
                    Eigen::Vector3d bary_coords = barycentric_coordinates(p, a_uv, b_uv, c_uv);
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
