#include "generate_texture.h"
#include "render_mesh_depth_map.h"
#include "uv_unwrap_mesh.h"

#include <vital/util/transform_image.h>

namespace kwiver {
namespace arrows{
namespace core {

double check_neighbouring_pixels_depth_map(const kwiver::vital::image& img, double x, double y)
{
  /// check the four closest pixels and return the average of the finite values
//  int x1 = static_cast<int>(x);
//  int y1 = static_cast<int>(y);
//   double v1;
//  double sum = 0.0;
//  int nb = 0;
//   if (x1 >= 0 && x1 < static_cast<int>(image.width()) && y1 >= 0 && y1 < static_cast<int>(image.height()))
//  {
//    v1 = image.at<double>(x1, y1);
//    if (!std::isinf(v1))
//    {
//      sum += v1;
//      nb++;
//    }
//  }
//  ++x1;
//  if (x1 >= 0 && x1 < static_cast<int>(image.width()) && y1 >= 0 && y1 < static_cast<int>(image.height()))
//  {
//    v1 = image.at<double>(x1, y1);
//    if (!std::isinf(v1))
//    {
//      sum += v1;
//      nb++;
//    }
//  }
//  ++y1;
//  if (x1 >= 0 && x1 < static_cast<int>(image.width()) && y1 >= 0 && y1 < static_cast<int>(image.height()))
//  {
//    v1 = image.at<double>(x1, y1);
//    if (!std::isinf(v1))
//    {
//      sum += v1;
//      nb++;
//    }
//  }
//  --x1;
//  if (x1 >= 0 && x1 < static_cast<int>(image.width()) && y1 >= 0 && y1 < static_cast<int>(image.height()))
//  {
//    v1 = image.at<double>(x1, y1);
//    if (!std::isinf(v1))
//    {
//      sum += v1;
//      nb++;
//    }
//  }
//  if (nb > 0)
//    return sum / nb;
//  else
//    return std::numeric_limits<double>::infinity();


//  if (x < 0 || y < 0 || x > img.width() - 1|| y > img.height() - 1)
//    return 0.0;

//  int p1x = static_cast<int>(x);
//  double normx = x - p1x;
//  int p1y = static_cast<int>(y);
//  double normy = y - p1y;

//  ptrdiff_t w_step = img.w_step(), h_step = img.h_step();
//  const double* pix1 = reinterpret_cast<const double*>(img.first_pixel()) + h_step * p1y + w_step * p1x;

//  if (normx < 1e-5 && normy < 1e-5) return *pix1;


//  if (normx < 1e-5)
//  {
//    if (!std::isinf(pix1[0]) && !std::isinf(pix1[h_step]))
//    {
//      return pix1[0] + (pix1[h_step] - pix1[0]) * normy;
//    }
//    else
//      return std::min(pix1[0], pix1[h_step]);
//  }
//  if (normy < 1e-5)
//  {
//    if (!std::isinf(pix1[0]) && !std::isinf(pix1[w_step]))
//    {
//      return pix1[0] + (pix1[w_step] - pix1[0]) * normx;
//    }
//    else
//      return std::min(pix1[0], pix1[w_step]);
//  }



//  double i1, i2;
//  if (!std::isinf(pix1[0]) && !std::isinf(pix1[h_step]))
//  {
//    i1 = pix1[0] + (pix1[h_step] - pix1[0]) * normy;
//  }
//  else
//    i1 = std::min(pix1[0], pix1[h_step]);

//  if (!std::isinf(pix1[w_step]) && !std::isinf(pix1[w_step + h_step]))
//  {
//    i2 = pix1[w_step] + (pix1[w_step + h_step] - pix1[w_step]) * normy;
//  }
//  else
//    i2 = std::min(pix1[w_step], pix1[w_step + h_step]);



//  if (!std::isinf(i1) && !std::isinf(i2))
//  {
//    return i1 + (i2 - i1) * normx;
//  }
//  else
//    return std::min(i1, i2);



  if (x < 0 || y < 0 || x > img.width() - 1|| y > img.height() - 1)
    return 0.0;


  int p1x = static_cast<int>(x);
  double normx = x - p1x;
  int p1y = static_cast<int>(y);
  double normy = y - p1y;

  ptrdiff_t w_step = img.w_step(), h_step = img.h_step();
  const double* pix1 = reinterpret_cast<const double*>(img.first_pixel()) + h_step * p1y + w_step * p1x;



  if (std::isinf(pix1[0]) || std::isinf(pix1[w_step]) || std::isinf(pix1[h_step]) || std::isinf(pix1[w_step+h_step]))
  {
    std::cout << "at least one is inf" << std::endl;
    std::cout << pix1[0] << " " << pix1[w_step] << " " << pix1[w_step+h_step] << " " << pix1[h_step] << std::endl;

    int nb = 0;
    double sum = 0.0;
    if (!std::isinf(pix1[0]))
    {
      sum += pix1[0];
      ++nb;
    }
    if (!std::isinf(pix1[w_step]))
    {
      sum += pix1[w_step];
      ++nb;
    }
    if (!std::isinf(pix1[h_step]))
    {
      sum += pix1[h_step];
      ++nb;
    }
    if (!std::isinf(pix1[w_step + h_step]))
    {
      sum += pix1[w_step + h_step];
      ++nb;
    }
    if (nb > 0)
    {
      std::cout << "ret " << sum/nb << std::endl;
      return sum / nb;
     }
    else
    {std::cout << "ret inf " << std::endl;
      return std::numeric_limits<double>::infinity();
    }
  }

  if (normx == 0 && normy == 0) return *pix1;
  if (normx == 0) return pix1[0] + (pix1[h_step] - pix1[0]) * normy;
  if (normy == 0) return pix1[0] + (pix1[w_step] - pix1[0]) * normx;

  double i1 = pix1[0] + (pix1[h_step] - pix1[0]) * normy;
  double i2 = pix1[w_step] + (pix1[w_step + h_step] - pix1[w_step]) * normy;

  return i1 + (i2 - i1) * normx;
}

vital::image_container_sptr generate_texture(vital::mesh_sptr mesh, std::vector<vital::camera_perspective_sptr> const& cameras, std::vector<vital::image> const& images)
{
  if (mesh->faces().regularity() != 3)
  {
    LOG_ERROR(vital::get_logger("arrows.core.generate_texture" ), "The mesh has to be triangular.");
    return nullptr;
  }

  kwiver::vital::mesh_vertex_array<3>& vertices = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(mesh->vertices());
  auto const& triangles = static_cast< const kwiver::vital::mesh_regular_face_array<3>& >(mesh->faces());

  /// Mesh uv parameterization
  double resolution = 0.0005;   // mesh unit/pixel

  // Unwrap the mesh
  if (mesh->has_tex_coords() == 0)
  {
    uv_unwrap_mesh unwrap;
    vital::config_block_sptr config = unwrap.get_configuration();
    config->set_value<double>("spacing", 0.005);
    unwrap.set_configuration(config);
    unwrap.unwrap(mesh);
  }

  auto tcoords = mesh->tex_coords();


  size_t factor = 1;
  // Rescale tcoords to real pixel values
  for (unsigned int f = 0; f < mesh->num_faces(); ++f)
  {
    auto const& tc1 = tcoords[f * 3 + 0];
    auto const& tc2 = tcoords[f * 3 + 1];
    auto const& tc3 = tcoords[f * 3 + 2];
    vital::vector_2d a2 = tc2 - tc1;
    vital::vector_2d b2 = tc3 - tc1;
    double area_2d = a2(0) * b2(1) - a2(1) * b2(0);

    auto const& v1 = vertices[triangles(f, 0)];
    auto const& v2 = vertices[triangles(f, 1)];
    auto const& v3 = vertices[triangles(f, 2)];
    vital::vector_3d a3 = v2 - v1;
    vital::vector_3d b3 = v3 - v1;
    double area_3d = a3.cross(b3).norm();

    if (!std::isinf(area_2d) && !std::isinf(area_3d))
    {
      factor = static_cast<size_t>(std::ceil(sqrt(area_3d / area_2d) / resolution));
    }
  }


  // Render the depth maps of the mesh seen by the different cameras
  std::vector<vital::image> depth_maps(images.size());
  for (unsigned int i = 0; i < images.size(); ++i)
  {
    depth_maps[i] = render_mesh_depth_map(mesh, cameras[i])->get_image();
  }


  for (auto& tc : tcoords)
  {
    tc.y() = 1.0 - tc.y();
    tc *= factor;
  }
  vital::image_of<unsigned char> texture(factor, factor, 3); /// unsigned char and 3 should be templte parameters
  vital::transform_image(texture, [](unsigned char){ return 0; });


  /// Compute the depth of each points w.r.t each camera
  std::vector< std::vector<double> > per_camera_point_depth(mesh->num_verts(), std::vector<double>(cameras.size(), 0.0));
  for (unsigned int v = 0; v < mesh->num_verts(); ++v)
  {
    for (unsigned int c = 0; c < cameras.size(); ++c)
    {
      per_camera_point_depth[v][c] = cameras[c]->depth(vertices[v]);
    }
  }

  std::vector<vital::camera_sptr> cameras_base(cameras.size());
  for (unsigned int k = 0; k < cameras.size(); ++k)
  {
    cameras_base[k] = cameras[k];
  }

  for (unsigned int f = 0; f < mesh->num_faces(); ++f)
  {
    unsigned int p1 = triangles(f, 0);
    unsigned int p2 = triangles(f, 1);
    unsigned int p3 = triangles(f, 2);
    vital::vector_3d const& pt_0 = vertices[p1];
    vital::vector_3d const& pt_1 = vertices[p2];
    vital::vector_3d const& pt_2 = vertices[p3];

    render_triangle_from_image<unsigned char>(tcoords[f * 3], tcoords[f * 3 + 1], tcoords[f * 3 + 2],
                                              pt_0, pt_1, pt_2, cameras_base, images,
                                              per_camera_point_depth[p1],
                                              per_camera_point_depth[p2],
                                              per_camera_point_depth[p3],
                                              depth_maps, texture, 0.01);

  }

  return std::make_shared<vital::simple_image_container>(texture);
}

}
}
}
