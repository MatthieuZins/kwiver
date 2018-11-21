#ifndef KWIVER_ARROWS_CORE_GENERATE_TEXTURE_H
#define KWIVER_ARROWS_CORE_GENERATE_TEXTURE_H

#include <arrows/core/kwiver_algo_core_export.h>
#include <arrows/core/triangle_scan_iterator.h>
#include <vital/types/camera.h>
#include <vital/types/camera_perspective.h>
#include <vital/types/image.h>
#include <vital/types/image_container.h>
#include <vital/types/mesh.h>
#include <vital/types/vector.h>
#include <iostream>

namespace kwiver {
namespace arrows {
namespace core {

KWIVER_ALGO_CORE_EXPORT
vital::image_container_sptr generate_texture(vital::mesh_sptr mesh, const std::vector<vital::camera_perspective_sptr> &cameras,
                                             std::vector<vital::image> const& images);

KWIVER_ALGO_CORE_EXPORT
double check_neighbouring_pixels_depth_map(const kwiver::vital::image& image, double x, double y);

template <class T>
double bilinear_interp_safe(const vital::image& img, double x, double y, int d=0);


/// This renders a triangle filled with label
template<class T>
void render_triangle(const vital::vector_2d& v1, const vital::vector_2d& v2,
                     const vital::vector_2d& v3, const T& label, vital::image& img)
{
  triangle_scan_iterator tsi(v1, v2, v3);
  for (tsi.reset(); tsi.next(); )
  {
    int y = tsi.scan_y();
    if (y < 0 || y >= static_cast<int>(img.height()))
      continue;
    int min_x = tsi.start_x();
    int max_x = tsi.end_x();
    if (max_x < 0 || min_x >= static_cast<int>(img.width()))
      continue;
    if (min_x < 0)
      min_x = 0;
    if (max_x >= static_cast<int>(img.width()))
      max_x = static_cast<int>(img.width()) - 1;

    for (int x = min_x; x <= max_x; ++x)
    {
      img.at<T>(x, y) = label;
    }
  }
}





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

// TODO add a check on orientation of the face w.r.t. the camera view point (maybe remove ? or put outside the function)
// check the projected triangle orientation
    vital::vector_2d ab = points[i].col(1) - points[i].col(0);
    vital::vector_2d ac = points[i].col(2) - points[i].col(0);
    double score = -(ab(0)*ac(1) - ab(1)*ac(0));
    scores[i] = std::max(score, 0.0);
  }

  // if mean score => normalize scores


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
        // if outside continue
        if (pt_img(0) < 0 || pt_img(0) >= images[i].width() || pt_img(1) < 0 || pt_img(1) >= images[i].height())
          continue;

        // interpolate depth of this 3d point
        double interpolated_depth = depths_pt1[i] + a * (depths_pt2[i] - depths_pt1[i]) + b * (depths_pt3[i] - depths_pt1[i]);

        // check if correspond to the depth map
        if (std::abs(interpolated_depth -  check_neighbouring_pixels_depth_map(depth_maps[i], pt_img(0), pt_img(1))) > depth_theshold)
        {
          continue;
        }

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


template <class T>
inline double bilinear_interp_safe(const vital::image& img, double x, double y, int d)
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
  if (normx < 1e-9) return pix1[0] + (pix1[h_step] - pix1[0]) * normy;
  if (normy < 1e-9) return pix1[0] + (pix1[w_step] - pix1[0]) * normx;

  double i1 = pix1[0] + (pix1[h_step] - pix1[0]) * normy;
  double i2 = pix1[w_step] + (pix1[w_step + h_step] - pix1[w_step]) * normy;

  return i1 + (i2 - i1) * normx;
}



/**
 * @brief dilate_texture This functions performs a dilation horizontally
 * and vertically
 * @param texture [in/out] image to dilate
 * @param mask [in] binary mask used for dilation
 * @param nb_iter [in] the dilation is repeated nb_iter times
 */
//template <class T>
//void dilate_atlas(vital::image& atlas, vital::image_of<bool> _mask, int nb_iter)
//{
//  vital::image_of<bool> mask;
//  mask.copy_from(_mask);

//  size_t height = atlas.height();
//  size_t width = atlas.width();
//  size_t depth = atlas.depth();

//  for (int n=0; n<nb_iter; ++n)
//  {
//    vital::image_of<bool> tmp;
//    tmp.copy_from(mask);

//    // vertically
//    for (int r=0; r < height; ++r)
//    {
//      for (int c=0; c < width; ++c)
//      {
//        if (mask(c, r) == false)
//        {
//          std::vector<T> values(depth, 0);
//          unsigned int nb = 0;
//          if ((r-1) >= 0 && mask(c, r-1))
//          {
//            for (int d=0; d < depth; ++d)
//            {
//              values[d] += atlas.at<T>(c, r-1, d);
//            }
//            nb++;
//          }
//          if ((r+1) < height && mask(c, r+1))
//          {
//            for (int d=0; d < depth; ++d)
//            {
//              values[d] += atlas.at<T>(c, r+1, d);
//            }
//            nb++;
//          }
//          if (nb > 0)
//          {
//            for (int d=0; d < depth; ++d)
//            {
//              atlas.at<T>(c, r, d) = static_cast<float>(values[d]) / nb;
//            }
//            tmp(c, r) = true;
//          }
//        }
//      }
//    }

//    // horizontally
//    for (int r=0; r < height; ++r)
//    {
//      for (int c=0; c < width; ++c)
//      {
//        if (mask(c, r) == false)
//        {
//          std::vector<T> values(depth, 0);
//          unsigned int nb = 0;
//          if ((c-1) >= 0 && mask(c-1, r))
//          {
//            for (int d=0; d < depth; ++d)
//            {
//              values[d] += atlas.at<T>(c-1, r, d);
//            }
//            nb++;
//          }
//          if ((c+1) < width && mask(c+1, r))
//          {
//            for (int d=0; d < depth; ++d)
//            {
//              values[d] += atlas.at<T>(c+1, r, d);
//            }
//            nb++;
//          }
//          if (nb > 0)
//          {
//            for (int d=0; d < depth; ++d)
//            {
//              atlas.at<T>(c, r, d) = static_cast<float>(values[d]) / nb;
//            }
//            tmp(c, r) = true;
//          }
//        }
//      }
//    }
//    mask.copy_from(tmp);
//  }
//}


}
}
}





#endif // KWIVER_ARROWS_CORE_GENERATE_TEXTURE_H
