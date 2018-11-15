#ifndef KWIVER_ARROWS_CORE_RENDER_H
#define KWIVER_ARROWS_CORE_RENDER_H

#include <arrows/core/kwiver_algo_core_export.h>
#include <vital/types/camera.h>
#include <vital/types/camera_perspective.h>
#include <vital/types/image.h>
#include <vital/types/triangle_scan_iterator.h>
#include <vital/types/vector.h>
#include <iostream>

namespace kwiver {
namespace arrows {

KWIVER_ALGO_CORE_EXPORT
double check_neighbouring_pixels_depth_map(const kwiver::vital::image& image, double x, double y);

template <class T>
double bilinear_interp_safe(const vital::image& img, double x, double y, int d=0);

/// This function render a triangle
template<class T>
void render_triangle(const vital::vector_2d& v1, const vital::vector_2d& v2, const vital::vector_2d& v3,
                     double depth_v1, double depth_v2, double depth_v3,
                     T attrib_v1, T attrib_v2, T attrib_v3,
                     vital::image_of<double>& depth_img,
                     vital::image_of<T>& img,
                     bool use_perspect_correct)
{
  assert(depth_v1 != 0 && depth_v2 != 0 && depth_v3 != 0);

  vital::triangle_scan_iterator tsi(v1, v2, v3);
  double i1 = static_cast<double>(attrib_v1);
  double i2 = static_cast<double>(attrib_v2);
  double i3 = static_cast<double>(attrib_v3);

  if (use_perspect_correct)
  {
    i1 /= depth_v1;
    i2 /= depth_v2;
    i3 /= depth_v3;

    depth_v1 = 1.0 / depth_v1;
    depth_v2 = 1.0 / depth_v2;
    depth_v3 = 1.0 / depth_v3;
  }
  // Linear interpolation attributes
  vital::vector_3d b1(v2.x()-v1.x(), v2.y()-v1.y(), i2 - i1);
  vital::vector_3d b2(v3.x()-v1.x(), v3.y()-v1.y(), i3 - i1);
  vital::vector_3d n = b1.cross(b2);
  double A = -n.x()/n.z();
  double B = -n.y()/n.z();
  double C = (v1.x() * n.x() + v1.y() * n.y() + i1 * n.z()) / n.z();
  // Linear interpolation depth
  vital::vector_3d b1_d(v2.x()-v1.x(), v2.y()-v1.y(), depth_v2 - depth_v1);
  vital::vector_3d b2_d(v3.x()-v1.x(), v3.y()-v1.y(), depth_v3 - depth_v1);
  vital::vector_3d n_d = b1_d.cross(b2_d);
  double A_d = -n_d.x()/n_d.z();
  double B_d = -n_d.y()/n_d.z();
  double C_d = (v1.x() * n_d.x() + v1.y() * n_d.y() + depth_v1 * n_d.z()) / n_d.z();

  for (tsi.reset(); tsi.next(); )
  {
    int y = tsi.scan_y();
    if (y < 0 || y >= img.height())
      continue;
    int min_x = tsi.start_x();
    int max_x = tsi.end_x();
    if (max_x < 0 || min_x >= img.width())
      continue;
    if (min_x < 0)
      min_x = 0;
    if (max_x >= img.width())
      max_x = img.width() - 1;

    double new_i = B * y + C;
    double new_i_d = B_d * y + C_d;
    for (int x = min_x; x <= max_x; ++x)
    {
      double attrib = new_i + A * x;
      double depth = new_i_d + A_d * x;
      if (use_perspect_correct)
      {
        depth = 1.0 / depth;
        attrib *= depth;
      }
      if (depth < depth_img(x, y))
      {
        img(x, y) =  static_cast<T>(attrib);
        depth_img(x, y) = depth;
      }
    }
  }
}

/// This renders a triangle filled with label
template<class T>
void render_triangle(const vital::vector_2d& v1, const vital::vector_2d& v2,
                     const vital::vector_2d& v3, const T& label, vital::image& img)
{
  vital::triangle_scan_iterator tsi(v1, v2, v3);
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
                                const std::vector<vital::vector_3d>& depths,
                                const std::vector<vital::image>& depth_maps,
                                vital::image& texture, double depth_theshold)
{
  assert(images[0].depth() == texture.depth());

  std::vector<double> scores(images.size(), 0.0);
  std::vector<vital::matrix_2x3d> points(images.size());
  for (int i = 0; i < images.size(); ++i)
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


  vital::triangle_scan_iterator tsi(v1, v2, v3);
  for (tsi.reset(); tsi.next(); )
  {
    int y = tsi.scan_y();
    if (y < 0 || y >= texture.height())
      continue;
    int min_x = tsi.start_x();
    int max_x = tsi.end_x();
    if (max_x < 0 || min_x >= texture.width())
      continue;
    if (min_x < 0)
      min_x = 0;
    if (max_x >= texture.width())
      max_x = texture.width() - 1;


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

      vital::vector_3d pt3d = pt1 + a * pt1pt2 + b * pt1pt3;


//      std::cout << interpolated_depth << " - " << depth.at<double>(pt_img(0), pt_img(1)) << std::endl;
      double score_max = 0.0;
      for (int i = 0; i < images.size(); ++i)
      {
        vital::vector_2d pt_img = cameras[i]->project(pt3d);
        if (pt_img(0) < 0 || pt_img(0) >= images[i].width() || pt_img(1) < 0 || pt_img(1) >= images[i].height())
          continue;
        double interpolated_depth = depths[i](0) + a * (depths[i](1)-depths[i](0)) + b * (depths[i](2)-depths[i](0));
        if (std::abs(interpolated_depth -  check_neighbouring_pixels_depth_map(depth_maps[i], pt_img(0), pt_img(1))) > depth_theshold)
        {
//          std::cout << "expected " << interpolated_depth << std::endl;
//          std::cout << "expected " << pt3d.z() << std::endl;
          continue;
        }
//        if (std::abs(interpolated_depth -  bilinear_interp_safe<double>(depth_maps[i], pt_img(0), pt_img(1))) > depth_theshold)
//            continue;

        if (scores[i] >= score_max && scores[i] > 0)
        {
          score_max = scores[i];
          for (int d = 0; d < texture.depth(); ++d)
            texture.at<T>(x, y, d) = bilinear_interp_safe<T>(images[i], pt_img(0), pt_img(1), d);
          // if mean score => texture... += scores[i] * bilin...
        }
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

  if (normx == 0 && normy == 0) return *pix1;
  if (normx == 0) return pix1[0] + (pix1[h_step] - pix1[0]) * normy;
  if (normy == 0) return pix1[0] + (pix1[w_step] - pix1[0]) * normx;

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
template <class T>
void dilate_atlas(vital::image& atlas, vital::image_of<bool> _mask, int nb_iter)
{
  vital::image_of<bool> mask;
  mask.copy_from(_mask);

  size_t height = atlas.height();
  size_t width = atlas.width();
  size_t depth = atlas.depth();

  for (int n=0; n<nb_iter; ++n)
  {
    vital::image_of<bool> tmp;
    tmp.copy_from(mask);

    // vertically
    for (int r=0; r < height; ++r)
    {
      for (int c=0; c < width; ++c)
      {
        if (mask(c, r) == false)
        {
          std::vector<T> values(depth, 0);
          unsigned int nb = 0;
          if ((r-1) >= 0 && mask(c, r-1))
          {
            for (int d=0; d < depth; ++d)
            {
              values[d] += atlas.at<T>(c, r-1, d);
            }
            nb++;
          }
          if ((r+1) < height && mask(c, r+1))
          {
            for (int d=0; d < depth; ++d)
            {
              values[d] += atlas.at<T>(c, r+1, d);
            }
            nb++;
          }
          if (nb > 0)
          {
            for (int d=0; d < depth; ++d)
            {
              atlas.at<T>(c, r, d) = static_cast<float>(values[d]) / nb;
            }
            tmp(c, r) = true;
          }
        }
      }
    }

    // horizontally
    for (int r=0; r < height; ++r)
    {
      for (int c=0; c < width; ++c)
      {
        if (mask(c, r) == false)
        {
          std::vector<T> values(depth, 0);
          unsigned int nb = 0;
          if ((c-1) >= 0 && mask(c-1, r))
          {
            for (int d=0; d < depth; ++d)
            {
              values[d] += atlas.at<T>(c-1, r, d);
            }
            nb++;
          }
          if ((c+1) < width && mask(c+1, r))
          {
            for (int d=0; d < depth; ++d)
            {
              values[d] += atlas.at<T>(c+1, r, d);
            }
            nb++;
          }
          if (nb > 0)
          {
            for (int d=0; d < depth; ++d)
            {
              atlas.at<T>(c, r, d) = static_cast<float>(values[d]) / nb;
            }
            tmp(c, r) = true;
          }
        }
      }
    }
    mask.copy_from(tmp);
  }
}


}
}






#endif // KWIVER_ARROWS_CORE_RENDER_H
