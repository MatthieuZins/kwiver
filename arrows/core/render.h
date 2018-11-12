#ifndef KWIVER_ARROWS_CORE_RENDER_H
#define KWIVER_ARROWS_CORE_RENDER_H

#include <vital/types/camera.h>
#include <vital/types/camera_perspective.h>
#include <vital/types/image.h>
#include <vital/types/triangle_scan_iterator.h>
#include <vital/types/vector.h>
#include <iostream>

namespace kwiver {
namespace arrows {


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
    for (int x = tsi.start_x(); x <= tsi.end_x(); ++x)
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



template <class T>
void render_triangle_from_image(const vital::vector_2d& v1, const vital::vector_2d& v2, const vital::vector_2d& v3,
                                const vital::vector_3d& pt1, const vital::vector_3d& pt2, const vital::vector_3d& pt3,
                                const std::vector<vital::camera_sptr> &cameras, const std::vector<vital::image>& images,
                                const std::vector<vital::vector_3d>& depths,
                                const std::vector<vital::image>& depth_maps,
                                vital::image& texture)
{
  assert(img.depth() == texture.depth());

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
    for (int x = tsi.start_x(); x <= tsi.end_x(); ++x)
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
        double interpolated_depth = depths[i](0) + a * (depths[i](1)-depths[i](0)) + b * (depths[i](2)-depths[i](0));
        if (std::abs(interpolated_depth - depth_maps[i].at<double>(pt_img(0), pt_img(1))) > 0.1 )
          continue;
        if (scores[i] > score_max)
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
double bilinear_interp_safe(const vital::image& img, double x, double y, int d)
{
  if (x <0 || y < 0 || x > img.width() || y >= img.height())
    return 0.0;

  int p1x = static_cast<int>(x);
  double normx = x - p1x;
  int p1y = static_cast<int>(y);
  double normy = y - p1y;

  img.at<T>(p1x, p1y);

  if (normx == 0 && normy == 0) return img.at<T>(p1x, p1y, d);
  if (normx == 0) return img.at<T>(p1x, p1y, d) + (img.at<T>(p1x, p1y + 1, d) - img.at<T>(p1x, p1y, d)) * normy;
  if (normy == 0) return img.at<T>(p1x, p1y, d) + (img.at<T>(p1x + 1, p1y, d) - img.at<T>(p1x, p1y, d)) * normx;

  double i1 = img.at<T>(p1x, p1y, d) + (img.at<T>(p1x, p1y + 1, d) - img.at<T>(p1x, p1y , d)) * normy;
  double i2 = img.at<T>(p1x + 1, p1y, d) + (img.at<T>(p1x + 1, p1y + 1, d) - img.at<T>(p1x + 1, p1y, d)) * normy;

  return i1 + (i2 - i1) * normx;
}




}
}






#endif // KWIVER_ARROWS_CORE_RENDER_H
