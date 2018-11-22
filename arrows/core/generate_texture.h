#ifndef KWIVER_ARROWS_CORE_GENERATE_TEXTURE_H
#define KWIVER_ARROWS_CORE_GENERATE_TEXTURE_H

#include <arrows/core/kwiver_algo_core_export.h>
#include <arrows/core/render_mesh_depth_map.h>
#include <arrows/core/triangle_scan_iterator.h>
#include <arrows/core/uv_unwrap_mesh.h>
#include <vital/types/camera.h>
#include <vital/types/camera_perspective.h>
#include <vital/types/image.h>
#include <vital/types/image_container.h>
#include <vital/types/mesh.h>
#include <vital/types/vector.h>
#include <vital/util/transform_image.h>
#include <iostream>

namespace kwiver {
namespace arrows {
namespace core {


template <class T>
void dilate_atlas(vital::image& atlas, vital::image_of<bool> _mask, int nb_iter);


/// This function samples an image at a non-integer location with bilinear interpolation
/**
* \param img [in] image
* \param x [in] location
* \param y [in] location
* \param d [in] depth
*/
template <class T>
inline double bilinear_interp_safe(const vital::image& img, double x, double y, int d=0)
{
  if (x < 0 || y < 0 || x > img.width() - 1|| y > img.height() - 1)
    return 0.0;

  int p1x = static_cast<int>(x);
  double normx = x - p1x;
  int p1y = static_cast<int>(y);
  double normy = y - p1y;

  ptrdiff_t w_step = img.w_step(), h_step = img.h_step();
  const T* pix1 = reinterpret_cast<const T*>(img.first_pixel()) + img.d_step() * d + h_step * p1y + w_step * p1x;

  if (normx < 1e-9 && normy < 1e-9) return *pix1;
  if (normx < 1e-9) return static_cast<T>(pix1[0] + (static_cast<double>(pix1[h_step]) - static_cast<double>(pix1[0])) * normy);
  if (normy < 1e-9) return static_cast<T>(pix1[0] + (static_cast<double>(pix1[w_step]) - static_cast<double>(pix1[0])) * normx);

  double i1 = pix1[0] + (static_cast<double>(pix1[h_step]) - static_cast<double>(pix1[0])) * normy;
  double i2 = pix1[w_step] + (static_cast<double>(pix1[w_step + h_step]) - static_cast<double>(pix1[w_step])) * normy;

  return i1 + (i2 - i1) * normx;
}

/// This function render a triangle and fill it from different image sources
/**
 * \param v1 [in] 2D triangle point
 * \param v2 [in] 2D triangle point
 * \param v3 [in] 2D triangle point
 * \param pt1 [in] 3D triangle point
 * \param pt2 [in] 3D triangle point
 * \param pt3 [in] 3D triangle point
 * \param cameras [in] list of cameras
 * \param images [in] list of images
 * \param depths_pt1 [in] depths of pt1 w.r.t cameras
 * \param depths_pt2 [in] depths of pt2 w.r.t cameras
 * \param depths_pt3 [in] depths of pt3 w.r.t cameras
 * \param depth_maps [in] list of depthmaps
 * \param texture [out] texture where the triangle is rendered
 * \param depth_theshold [in] threshold used for depth test
 */
template <class T>
void render_triangle_from_image(const vital::vector_2d& v1, const vital::vector_2d& v2, const vital::vector_2d& v3,
                                const vital::vector_3d& pt1, const vital::vector_3d& pt2, const vital::vector_3d& pt3,
                                const std::vector<vital::camera_sptr> &cameras, const std::vector<vital::image>& images,
                                std::vector<double> const& depths_pt1,
                                std::vector<double> const& depths_pt2,
                                std::vector<double> const& depths_pt3,
                                const std::vector<vital::image>& depth_maps,
                                vital::image& texture, double depth_theshold)
{
  assert(images[0].depth() == texture.depth());

  std::vector<double> scores(images.size(), 0.0);
  std::vector<vital::matrix_2x3d> points(images.size());
  for (size_t i = 0; i < images.size(); ++i)
  {
    points[i].col(0) = cameras[i]->project(pt1);
    points[i].col(1) = cameras[i]->project(pt2);
    points[i].col(2) = cameras[i]->project(pt3);

    vital::vector_2d ab = points[i].col(1) - points[i].col(0);
    vital::vector_2d ac = points[i].col(2) - points[i].col(0);
    double score = -(ab(0)*ac(1) - ab(1)*ac(0));
    scores[i] = std::max(score, 0.0);
  }

  triangle_scan_iterator tsi(v1, v2, v3);
  for (tsi.reset(); tsi.next(); )
  {
    int y = tsi.scan_y();
    if (y < 0 || y >= static_cast<int>(texture.height()))
      continue;
    int min_x = std::max(0, tsi.start_x());
    int max_x = std::min(static_cast<int>(texture.width()) - 1, tsi.end_x());

    vital::vector_2d v1v2 = v2 - v1;
    vital::vector_2d v1v3 = v3 - v1;

    vital::vector_3d pt1pt2 = pt2 - pt1;
    vital::vector_3d pt1pt3 = pt3 - pt1;
    for (int x = min_x; x <= max_x; ++x)
    {
      vital::vector_2d coord(x, y);
      coord -= v1;
      double a = (v1v3(0) * coord(1) - v1v3(1) * coord(0)) / (v1v3(0) * v1v2(1) - v1v3(1) * v1v2(0));
      double b = (v1v2(0) * coord(1) - v1v2(1) * coord(0)) / (v1v2(0) * v1v3(1) - v1v2(1) * v1v3(0));

      // Corresponding 3d point
      vital::vector_3d pt3d = pt1 + a * pt1pt2 + b * pt1pt3;
      double score_max = 0.0;
      int max_score_index = -1;
      vital::vector_2d max_score_img_pt = {0, 0};
      for (size_t i = 0; i < images.size(); ++i)
      {
        // Corresponding point in image i
        vital::vector_2d pt_img = cameras[i]->project(pt3d);
        if (pt_img(0) < 0 || pt_img(0) >= images[i].width() || pt_img(1) < 0 || pt_img(1) >= images[i].height())
          continue;

        // visibility test from the camera i
        double interpolated_depth = depths_pt1[i] + a * (depths_pt2[i] - depths_pt1[i]) + b * (depths_pt3[i] - depths_pt1[i]);
        if (std::abs(interpolated_depth -  bilinear_interp_safe<double>(depth_maps[i], pt_img(0), pt_img(1))) > depth_theshold)
          continue;

        if (scores[i] > score_max)
        {
          score_max = scores[i];
          max_score_index = i;
          max_score_img_pt = pt_img;
        }
      }
      // Fill the pixel with data from the image with the highest score
      if (max_score_index >= 0)
      {
        for (size_t d = 0; d < texture.depth(); ++d)
          texture.at<T>(x, y, d) = bilinear_interp_safe<T>(images[max_score_index], max_score_img_pt(0), max_score_img_pt(1), d);
      }
    }
  }
}

/// This functions dilates the texture atlas based on a binary mask
/**
 * \param texture [in/out] image to dilate
 * \param mask [in/out] binary mask used for dilation
 * \param nb_iter [in] the dilation is repeated nb_iter times
 */
template <class T>
void dilate_atlas(vital::image& texture, vital::image_of<char> mask, int nb_iter)
{
  int height = static_cast<int>(texture.height());
  int width = static_cast<int>(texture.width());

  auto copy_pixel = [&texture](unsigned int x_d, unsigned int y_d, unsigned int x_s, unsigned int y_s)
  {
    T* dest = &texture.at<T>(x_d, y_d, 0);
    T* src = &texture.at<T>(x_s, y_s, 0);
    for (int i = 0; i < texture.depth(); ++i, src += texture.d_step(), dest += texture.d_step())
    {
      *dest = *src;
    }
  };
  // horizontal
  for (int n = 0; n < nb_iter; ++n)
  {
    for (unsigned int y = 0; y < height; ++y)
    {
      for (unsigned int x = 0; x < width - 1; ++x)
      {
        char d = mask(x + 1, y) - mask(x, y);
        if (d == 1)
        {
          mask(x, y) = 1;
          copy_pixel(x, y, x + 1, y);
        }
        else if (d == -1)
        {
          mask(x + 1, y) = 1;
          copy_pixel(x + 1, y, x, y);
          ++x;
        }
      }
    }
    // vertical
    for (unsigned int x = 0; x < width; ++x)
    {
      for (unsigned int y = 0; y < height - 1; ++y)
      {
        char d = mask(x, y + 1) - mask(x, y);
        if (d == 1)
        {
          mask(x, y) = 1;
          copy_pixel(x, y, x, y + 1);
        }
        else if (d == -1)
        {
          mask(x, y + 1) = 1;
          copy_pixel(x, y + 1, x, y);
          ++y;
        }
      }
    }
  }
}


/// This function generates a texture from a set of images and maps it on the mesh
/**
 * \param mesh [in/out] the mesh to texture.
 * \param cameras [in] a list of cameras
 * \param images [in] a list of images
 * \param resolution [in] resolution of the texture ??? or the size of the texture
 */
template <class T, int N>
vital::image_container_sptr
generate_texture(vital::mesh_sptr mesh, std::vector<vital::camera_perspective_sptr> const& cameras,
                 std::vector<vital::image> const& images, double resolution)
{
  if (mesh->faces().regularity() != 3)
  {
    LOG_ERROR(vital::get_logger("arrows.core.generate_texture" ), "The mesh has to be triangular.");
    return nullptr;
  }

  kwiver::vital::mesh_vertex_array<3>& vertices = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(mesh->vertices());
  auto const& triangles = static_cast< const kwiver::vital::mesh_regular_face_array<3>& >(mesh->faces());

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
  vital::image_of<T> texture(factor, factor, N); /// unsigned char and 3 should be templte parameters
  vital::transform_image(texture, [](T){ return 0; });
  kwiver::vital::image_of<char> texture_label(factor, factor, 1);
  vital::transform_image(texture_label, [](char){ return 0; });

  // Compute the depth of each points w.r.t each camera
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

    render_triangle_from_image<T>(tcoords[f * 3], tcoords[f * 3 + 1], tcoords[f * 3 + 2],
                                              pt_0, pt_1, pt_2, cameras_base, images,
                                              per_camera_point_depth[p1],
                                              per_camera_point_depth[p2],
                                              per_camera_point_depth[p3],
                                              depth_maps, texture, 0.1);

    kwiver::arrows::core::render_triangle<bool>(tcoords[f * 3], tcoords[f * 3 + 1], tcoords[f * 3 + 2], true, texture_label);
  }

  kwiver::arrows::core::dilate_atlas<T>(texture, texture_label, 4);

  // Update texture coordinates
  for (auto& tc : tcoords)
  {
    tc[0] += 0.5;
    tc[1] += 0.5;
    tc /= factor;
    tc[1] = 1.0 - tc[1];
  }

  mesh->set_tex_coords(tcoords);
  return std::make_shared<vital::simple_image_container>(texture);
}



}
}
}





#endif // KWIVER_ARROWS_CORE_GENERATE_TEXTURE_H
