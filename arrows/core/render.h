#ifndef KWIVER_ARROWS_CORE_RENDER_H
#define KWIVER_ARROWS_CORE_RENDER_H

#include <vital/types/image.h>
#include <vital/types/triangle_scan_iterator.h>
#include <vital/types/vector.h>
#include <iostream>
namespace kwiver {
namespace arrows {


/// This function render a triangle
template<class T>
void render_triangle(const vital::vector_2d& v1, const vital::vector_2d& v2, const vital::vector_2d& v3,
                     double depth_v1, double depth_v2, double depth_v3,
                     T attrib_v1, T attrib_v2, T attrib_v3,
                     const vital::image_of<double>& depth_img,
                     vital::image_of<T>& img,
                     bool use_perspect_correct)
{
  assert(depth_v1 != 0 && depth_v2 != 0 && depth_v3 != 0);

  vital::triangle_scan_iterator tsi(v1, v2, v3);

  vital::vector_3d b1(v2.x()-v1.x(), v2.y()-v1.y(), attrib_v2 - attrib_v1);
  vital::vector_3d b2(v3.x()-v1.x(), v3.y()-v1.y(), attrib_v3 - attrib_v1);
  vital::vector_3d n = b1.cross(b2);
  double A = -n.x()/n.z();
  double B = -n.y()/n.z();
  double C = (v1.x() * n.x() + v1.y() * n.y() + attrib_v1 * n.z()) / n.z();

//  if (use_perspect_correct)
//  {
//    depth_v1 = 1.0 / depth_v1;
//    depth_v2 = 1.0 / depth_v2;
//    depth_v3 = 1.0 / depth_v3;
//  }

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
    for (int x = tsi.start_x(); x <= tsi.end_x(); ++x)
    {
      double depth = new_i + A * x;
//      if (depth < depth_img(x, y))
      {
        img(x, y) = depth;
      }
    }
  }
}

}
}






#endif // KWIVER_ARROWS_CORE_RENDER_H
