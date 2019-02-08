/*ckwg +29
 * Copyright 2018 by Kitware, SAS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * \brief Header for core generate_texture functions
 */

#ifndef KWIVER_ARROWS_CORE_GENERATE_TEXTURE_H
#define KWIVER_ARROWS_CORE_GENERATE_TEXTURE_H

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


namespace kwiver {
namespace arrows {
namespace core {


/// This function samples an image at a non-integer location with bilinear interpolation
/**
* \param img [in] image
* \param x [in] location
* \param y [in] location
* \param d [in] depth
*/
template <class T>
inline double bilinear_interp_safe(vital::image_of<T> const& img, double x, double y, int d=0)
{
  if (x < 0 || y < 0 || x >= img.width() - 1|| y >= img.height() - 1)
    return 0.0;

  int p1x = static_cast<int>(x);
  double normx = x - p1x;
  int p1y = static_cast<int>(y);
  double normy = y - p1y;

  ptrdiff_t w_step = img.w_step(), h_step = img.h_step();
  T const* pix1 = reinterpret_cast<T const*>(img.first_pixel()) + img.d_step() * d + h_step * p1y + w_step * p1x;

  if (normx < 1e-9 && normy < 1e-9) return *pix1;
  if (normx < 1e-9) return static_cast<T>(pix1[0] + (pix1[h_step] - pix1[0]) * normy);
  if (normy < 1e-9) return static_cast<T>(pix1[0] + (pix1[w_step] - pix1[0]) * normx);

  double i1 = pix1[0] + (pix1[h_step] - pix1[0]) * normy;
  double i2 = pix1[w_step] + (pix1[w_step + h_step] - pix1[w_step]) * normy;

  return i1 + (i2 - i1) * normx;
}


/// Base functor used to fuse multiple images. It adjusts the contribution of each image.
struct KWIVER_ALGO_CORE_EXPORT image_fusion_method
{
  virtual ~image_fusion_method() {}
  virtual void operator ()(double *scores, int nb_scores) const = 0;
};


/// This functor sets the highest score to 1 and the other to 0
struct KWIVER_ALGO_CORE_EXPORT select_max_score : image_fusion_method
{
  virtual void operator ()(double *scores, int nb_scores) const;
};


/// This functor normalizes the scores
struct KWIVER_ALGO_CORE_EXPORT normalize_scores : image_fusion_method
{
  virtual void operator ()(double *scores, int nb_scores) const;
};


/// This function renders the scores of a triangle into texture coordinate images. These score
/// images for the different cameras are stacked. The images points corresponding to the texture
/// pixel are also saved in points_image.
/**
 * \param v1 [in] 2D triangle point
 * \param v2 [in] 2D triangle point
 * \param v3 [in] 2D triangle point
 * \param pt1 [in] 3D triangle point
 * \param pt2 [in] 3D triangle point
 * \param pt3 [in] 3D triangle point
 * \param cameras [in] list of cameras
 * \param depths_pt1 [in] depths of pt1 w.r.t cameras
 * \param depths_pt2 [in] depths of pt2 w.r.t cameras
 * \param depths_pt3 [in] depths of pt3 w.r.t cameras
 * \param depth_maps [in] list of depthmaps
 * \param depth_threshold [in] threshold used for depth test
 * \param scores_image [out] image containing scores
 * \param points_image [out] image continaing 2d positions
 */
KWIVER_ALGO_CORE_EXPORT
void render_triangle_scores(vital::vector_2d const& v1, vital::vector_2d const& v2, vital::vector_2d const& v3,
                            vital::vector_3d const& pt1, vital::vector_3d const& pt2, vital::vector_3d const& pt3,
                            std::vector<vital::camera_sptr> const& cameras,
                            std::vector<double> const& depths_pt1,
                            std::vector<double> const& depths_pt2,
                            std::vector<double> const& depths_pt3,
                            const std::vector< vital::image_of<double> > &depth_maps,
                            double depth_threshold,
                            vital::image_of<double>& scores_image,
                            vital::image_of<double>& points_image);


/// This function adjusts the contributions (scores) of each image
/**
 * @param tex_coords [in] coordinates of the triangle in the image
 * @param method [in] method used
 * @param scores_image [out] image containing the scores
 */
KWIVER_ALGO_CORE_EXPORT
void adjust_cameras_contributions(vital::vector_2d const& v1,
                                  vital::vector_2d const& v2,
                                  vital::vector_2d const& v3,
                                  image_fusion_method const& method,
                                  vital::image_of<double>& scores_image);


/// This function finds the dimenson of the largest face
/**
 * @param coords [in] the coordinates of the vertices. Each group of three vertices represents a face
 * @param nb_faces [in] the number of faces
 * @return the largest face dimension
 */
KWIVER_ALGO_CORE_EXPORT
vital::vector_2d find_largest_face_dimensions(std::vector<vital::vector_2d> const& coords, unsigned int nb_faces);


/// This function finds the scale to apply to the texture in order to reach the required resolution
/**
 * @param vertices [in] mesh vertices
 * @param tcoords [in] texture coordinates of the mesh
 * @param faces [in] list of faces
 * @param resolution [in] required resolution (mesh unit/pixexl)
 * @return texture scaling
 */
KWIVER_ALGO_CORE_EXPORT
size_t find_texture_scaling(vital::mesh_vertex_array<3> const& vertices,
                            std::vector<vital::vector_2d> const& tcoords,
                            vital::mesh_regular_face_array<3> const& faces,
                            double resolution);


/// This function dilates the texture atlas using a binary mask
/**
 * \param texture [in/out] image to dilate
 * \param mask [in/out] binary mask used for dilation
 * \param nb_iter [in] the dilation is repeated nb_iter times
 */
template <class T>
void dilate_atlas(vital::image_of<T>& texture, vital::image_of<char>& mask, int nb_iter)
{
  int height = static_cast<int>(texture.height());
  int width = static_cast<int>(texture.width());

  // lambda to copy a pixel with any depth
  auto copy_pixel = [&texture](unsigned int x_d, unsigned int y_d, unsigned int x_s, unsigned int y_s)
  {
    T* dest = &texture(x_d, y_d, 0);
    T* src = &texture(x_s, y_s, 0);
    for (int i = 0; i < texture.depth(); ++i, src += texture.d_step(), dest += texture.d_step())
    {
      *dest = *src;
    }
  };
  for (int n = 0; n < nb_iter; ++n)
  {
    // horizontal dilate
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
    // vertical dilate
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


/// This function renders a triangle in the texture from weighted source images
/**
 * \param v1 [in] 2D triangle point
 * \param v2 [in] 2D triangle point
 * \param v3 [in] 2D triangle point
 * \param images [in] list of images
 * \param scores_image [in] image of scores
 * \param points_image [in] image of 2d positions
 * \param texture [out] texture where the triangle is rendered
 * \param shift [in] shift of the face bounding box between the local coordinates
 *        in scores_image (or points_image) and the global texture coordinates
 */
template<class T>
void render_texture_from_images(vital::vector_2d const& v1, vital::vector_2d const& v2, vital::vector_2d const& v3,
                                std::vector< vital::image_of<T> > const& images,
                                vital::image_of<double> const& scores_image,
                                vital::image_of<double> const& points_image,
                                vital::vector_2i const& shift,
                                vital::image_of<T>& texture)
{
  triangle_bb_iterator tsi(v1, v2,v3);
  for (tsi.reset(); tsi.next(); )
  {
    int y = tsi.scan_y();
    if (y < 0 || y >= static_cast<int>(texture.height()))
      continue;
    int min_x = std::max(0, tsi.start_x());
    int max_x = std::min(static_cast<int>(texture.width()) - 1, tsi.end_x());
    for (int x = min_x; x <= max_x; ++x)
    {
      std::vector<double> values(texture.depth(), 0.0);
      vital::vector_2i local_point(x, y);
      local_point -= shift;
      for (int i = 0; i < images.size(); ++i)
      {
        double score = scores_image(local_point.x(), local_point.y(), i);
        if (score > 0)
        {
          vital::vector_2d pt_img(points_image(local_point.x(), local_point.y(), i*2),
                                  points_image(local_point.x(), local_point.y(), i*2+1));
          for (int d = 0; d < texture.depth(); ++d)
          {
            values[d] += score * bilinear_interp_safe<T>(images[i], pt_img(0), pt_img(1), d);
          }
        }
      }
      for (int d = 0; d < texture.depth(); ++d)
      {
        texture(x, y, d) = static_cast<T>(values[d]);
      }
    }
  }
}

KWIVER_ALGO_CORE_EXPORT
vital::vector_2d find_largest_face_dimensions(std::vector<vital::vector_2d> const& tcoords, unsigned int nb_faces);



/// This function generates a texture from a set of images and maps it on the mesh
/**
 * \param mesh [in/out] the mesh to texture.
 * \param cameras [in] a list of cameras perspective
 * \param images [in] a list of images
 * \param resolution [in] resolution of the texture (mesh unit/pixel)
 * \return the generated texture
 */
template <class T>
vital::image_container_sptr
generate_texture(vital::mesh_sptr mesh, std::vector<vital::camera_perspective_sptr> const& cameras,
                 std::vector< vital::image_of<T> > const& images, double resolution)
{
  if (mesh->faces().regularity() != 3)
  {
    LOG_ERROR(vital::get_logger("arrows.core.generate_texture" ), "The mesh has to be triangular.");
    return nullptr;
  }

  vital::mesh_vertex_array<3>& vertices = dynamic_cast< vital::mesh_vertex_array<3>& >(mesh->vertices());
  auto const& triangles = static_cast< vital::mesh_regular_face_array<3> const& >(mesh->faces());

  // Unwrap the mesh
  if (mesh->has_tex_coords() == 0)
  {
    uv_unwrap_mesh unwrap;
    unwrap.unwrap(mesh);
  }

  // Find the texture scaling
  std::vector<vital::vector_2d> tcoords = mesh->tex_coords();
  size_t scale = find_texture_scaling(vertices, tcoords, triangles, resolution);

  // Adjust the coordinates that are used to fill the texture image
  for (auto& tc : tcoords)
  {
    // flip Y because vital::image has origin at the top-left corner
    tc[1] = 1.0 - tc[1];
    tc *= scale;
    // subtract 0.5 because integer coordinates are at the center of the pixels
    tc[0] -= 0.5;
    tc[1] -= 0.5;
  }

  // Render the depth maps of the mesh seen by the different cameras
  std::vector< vital::image_of<double> > depth_maps(images.size());
  for (unsigned int i = 0; i < images.size(); ++i)
  {
    depth_maps[i] = vital::image_of<double>(render_mesh_depth_map(mesh, cameras[i])->get_image());
  }

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

  // Find the bounding box dimensions of the largest face
  vital::vector_2d largest_dim = find_largest_face_dimensions(tcoords, mesh->num_faces());
  int face_patch_width = std::ceil(largest_dim.x()) + 2;
  int face_patch_height = std::ceil(largest_dim.y()) + 2;

  // create and initialize a texture, a mask, and two images to cache scores and positions
  vital::image_of<T> texture(scale, scale, images[0].depth());
  vital::transform_image(texture, [](T){ return 0; });

  kwiver::vital::image_of<char> texture_mask(scale, scale, 1);
  vital::transform_image(texture_mask, [](char){ return 0; });

  // scores_image and points_image only contains the current face. A shift is used to adjust the texture coordinates
  kwiver::vital::image_of<double> scores_image(face_patch_width, face_patch_height, cameras.size(), true);
  vital::transform_image(scores_image, [](double){ return 0; });

  kwiver::vital::image_of<double> points_image(face_patch_width, face_patch_height, cameras.size() * 2, true);
  vital::transform_image(points_image, [](double){ return 0; });


  for (unsigned int f = 0; f < mesh->num_faces(); ++f)
  {
    vital::vector_2d const& tc1 = tcoords[f * 3];
    vital::vector_2d const& tc2 = tcoords[f * 3 + 1];
    vital::vector_2d const& tc3 = tcoords[f * 3 + 2];

    unsigned int p1 = triangles(f, 0);
    unsigned int p2 = triangles(f, 1);
    unsigned int p3 = triangles(f, 2);

    // shift of the face bounding box
    vital::vector_2d shift;
    shift.x() = std::floor(std::min(tc1.x(), std::min(tc2.x(), tc3.x())));
    shift.y() = std::floor(std::min(tc1.y(), std::min(tc2.y(), tc3.y())));

    // compute the scores in a texture coordinates image and cache image points
    render_triangle_scores(tc1 - shift, tc2 - shift, tc3 - shift,
                           vertices[p1], vertices[p2], vertices[p3],
                           cameras_base,
                           per_camera_point_depth[p1],
                           per_camera_point_depth[p2],
                           per_camera_point_depth[p3],
                           depth_maps, 0.1, scores_image,
                           points_image);

    // update the triangles mask
    kwiver::arrows::core::render_triangle<char>(tc1, tc2, tc3, 1, texture_mask);

    // adjust camera contribution
    adjust_cameras_contributions(tc1 - shift, tc2 - shift, tc3 - shift,
                                 select_max_score(), scores_image);

    // render the texture from images
    render_texture_from_images<T>(tc1, tc2, tc3, images, scores_image, points_image,
                                  shift.cast<int>(), texture);
  }

  kwiver::arrows::core::dilate_atlas<T>(texture, texture_mask, 1);

  return std::make_shared<vital::simple_image_container>(texture);
}



}
}
}

#endif // KWIVER_ARROWS_CORE_GENERATE_TEXTURE_H
