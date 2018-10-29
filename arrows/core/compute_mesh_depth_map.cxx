#include "compute_mesh_depth_map.h"

#include <vital/types/camera_perspective.h>
#include <vital/types/camera_rpc.h>
#include <vital/types/geodesy.h>
#include <vital/types/image.h>
#include <vital/types/vector.h>


namespace {

/// Helper function to check if a pont is inside a triangle
bool is_point_inside_triangle(const kwiver::vital::vector_2d& p,
                              const kwiver::vital::vector_2d& a,
                              const kwiver::vital::vector_2d& b,
                              const kwiver::vital::vector_2d& c)
{
  kwiver::vital::vector_2d AB = b - a;
  kwiver::vital::vector_2d AC = c - a;
  kwiver::vital::vector_2d AP = p - a;
  double inv_total_area = 1.0 / (AB[0] * AC[1] - AB[1] * AC[0]);
  double area_1 = inv_total_area * (AB[0] * AP[1] - AB[1] * AP[0]);
  double area_2 = inv_total_area * (AP[0] * AC[1] - AP[1] * AC[0]);
  return area_1 >= 0 && area_2 >= 0 && (area_1 + area_2) <= 1;
}

/// Helper function to compute barycentric coordinates of p w.r.t the triangle abc
kwiver::vital::vector_3d barycentric_coordinates(const kwiver::vital::vector_2d& p,
                                                 const kwiver::vital::vector_2d& a,
                                                 const kwiver::vital::vector_2d& b,
                                                 const kwiver::vital::vector_2d& c)
{
  kwiver::vital::matrix_3x3d abc;
  abc << a, b, c, 1, 1, 1;
  double det_inv = 1.0 / abc.determinant();
  kwiver::vital::vector_3d res;
  res(0) = ((b(1) - c(1)) * (p(0) - c(0)) - (b(0) - c(0)) * (p(1) - c(1))) * det_inv;
  res(1) = ((c(1) - a(1)) * (p(0) - c(0)) - (c(0) - a(0)) * (p(1) - c(1))) * det_inv;
  res(2) = 1.0 - res(0) - res(1);
  return res;
}

}

namespace kwiver {
namespace arrows {

std::pair<vital::image_container_sptr, vital::image_container_sptr>
compute_mesh_depth_map(vital::mesh_sptr mesh, vital::camera_sptr camera)
{
  unsigned int nb_vertices = mesh->num_verts();
  unsigned int width = camera->image_width();
  unsigned int height = camera->image_height();
  vital::mesh_vertex_array<3>& vertices = dynamic_cast< vital::mesh_vertex_array<3>& >(mesh->vertices());

  // project all points on image
  std::vector<vital::vector_2d> points_2d(nb_vertices);
  vital::camera_rpc *rpc_camera = dynamic_cast<vital::camera_rpc*>(camera.get());
  std::vector<double> points_depth(nb_vertices);
  if (rpc_camera != nullptr)
  {
    std::vector<vital::vector_2d> latlong_points(nb_vertices);
    // For rpc cameras, the mesh coordinates are first transformed to lat/long coordinates
    vital::vector_3d center(0.0, 0.0, 0.0);
    for (int i = 0; i < vertices.size(); ++i)
    {
      latlong_points[i] = vital::geo_conv(vertices[i].head<2>(),
                                                      vital::SRID::UTM_WGS84_north + rpc_camera->determine_utm_zone(),
                                                      vital::SRID::lat_lon_WGS84);
      center[0] += latlong_points[i][0];
      center[1] += latlong_points[i][1];
      center[2] += vertices[i][2];
    }
    center /= vertices.size();
    vital::camera_affine_sptr affine_approx_cam = rpc_camera->approximate_affine_camera(center);
    std::cout << "Camera center =  " << affine_approx_cam->get_center() << std::endl;
    for (int i = 0; i < vertices.size(); ++i)
    {
      points_2d[i] = affine_approx_cam->project({latlong_points[i][0], latlong_points[i][1], vertices[i](2)});
      points_depth[i] = affine_approx_cam->depth({latlong_points[i][0], latlong_points[i][1], vertices[i](2)});
    }
    // Compute the points depth
//    for (unsigned int i=0; i < vertices.size(); ++i)
//    {
//      points_depth[i] = affine_approx_cam->depth(vertices[i]);
//    }
  }
  else
  {
    for (int i = 0; i < vertices.size(); ++i)
    {
      points_2d[i] = camera->project(vertices[i]);
    }
    // Compute the points depth
    for (unsigned int i=0; i < vertices.size(); ++i)
    {
      points_depth[i] = std::dynamic_pointer_cast<vital::camera_perspective>(camera)->depth(vertices[i]);
    }
  }

  // Initialize z_buffer with max double and id_buffer with -1
  vital::image_of<double> z_buffer(width, height, 1);
  vital::image_of<int> id_map(width, height, 1);
  for (int i=0; i < height; ++i)
  {
    for (int j=0; j < width; ++j)
    {
      z_buffer(j, i) = std::numeric_limits<double>::max();
      id_map(j, i) = -1;
    }
  }
  // Write faces on z_buffer and id_map with depth test
  vital::mesh_face_array& faces = dynamic_cast< vital::mesh_face_array& >(mesh->faces());
  for (unsigned int f_id = 0; f_id < faces.size(); ++f_id)
  {
    const vital::vector_2d& a_uv = points_2d[faces(f_id, 0)];
    const vital::vector_2d& b_uv = points_2d[faces(f_id, 1)];
    const vital::vector_2d& c_uv = points_2d[faces(f_id, 2)];
    // skip the face if the three points are outside the image
    if ((a_uv[0] < 0 || a_uv[0] >= width || a_uv[1] < 0 || a_uv[1] >= height) &&
        (b_uv[0] < 0 || b_uv[0] >= width || b_uv[1] < 0 || b_uv[1] >= height) &&
        (c_uv[0] < 0 || c_uv[0] >= width || c_uv[1] < 0 || c_uv[1] >= height))
      continue;
    double a_depth = points_depth[faces(f_id, 0)];
    double b_depth = points_depth[faces(f_id, 1)];
    double c_depth = points_depth[faces(f_id, 2)];
    // the rasterization is done over the face axis-aligned bounding box
    int u_min = std::max(0, static_cast<int>(std::round(std::min(a_uv[0], std::min(b_uv[0], c_uv[0])))));
    int u_max = std::min(static_cast<int>(width) - 1, static_cast<int>(std::round(std::max(a_uv[0], std::max(b_uv[0], c_uv[0])))));
    int v_min = std::max(0, static_cast<int>(std::round(std::min(a_uv[1], std::min(b_uv[1], c_uv[1])))));
    int v_max = std::min(static_cast<int>(height) - 1, static_cast<int>(std::round(std::max(a_uv[1], std::max(b_uv[1], c_uv[1])))));
    for (int v = v_min; v <= v_max; ++v)
    {
      for (int u = u_min; u <= u_max; ++u)
      {
        vital::vector_2d p(u, v);
        // only compute depth for points inside the triangle
        if (is_point_inside_triangle(p, a_uv, b_uv, c_uv))
        {
          vital::vector_3d bary_coords = barycentric_coordinates(p, a_uv, b_uv, c_uv);
          // interpolate depth
          double depth = bary_coords[0] * a_depth + bary_coords[1] * b_depth + bary_coords[2] * c_depth;
          if (depth >= 0 && depth < z_buffer(u, v))
          {
            z_buffer(u, v) = depth;
            id_map(u, v) = f_id;
          }
        }
      }
    }
  }
  return std::make_pair(std::make_shared<vital::simple_image_container>(z_buffer),
                        std::make_shared<vital::simple_image_container>(id_map));
}

}
}
