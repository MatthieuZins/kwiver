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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
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
 * \brief Header for kwiver::arrows::triangle_scan_iterator
 */

#ifndef KWIVER_ARROWS_CORE_TRIANGLE_SCAN_ITERATOR_H
#define KWIVER_ARROWS_CORE_TRIANGLE_SCAN_ITERATOR_H

#include <arrows/core/kwiver_algo_core_export.h>
#include <vital/types/vector.h>

namespace kwiver {
namespace arrows {
namespace core {


namespace {
bool is_point_inside_triangle(const Eigen::Vector2d& p,
                              const Eigen::Vector2d& a,
                              const Eigen::Vector2d& b,
                              const Eigen::Vector2d& c)
{
    Eigen::Vector2d AB = b - a;
    Eigen::Vector2d AC = c - a;
    Eigen::Vector2d AP = p - a;

    double inv_total_area = 1.0 / (AB[0] * AC[1] - AB[1] * AC[0]);
    double area_1 = inv_total_area * (AB[0] * AP[1] - AB[1] * AP[0]);
    double area_2 = inv_total_area * (AP[0] * AC[1] - AP[1] * AC[0]);

    return area_1 >= 0 && area_2 >= 0 && (area_1+area_2) <= 1;
}

double dist_point_line(vital::vector_2d const& p, vital::vector_2d const& x0, vital::vector_2d const& x1)
{
  vital::vector_2d v = x1 - x0;
  vital::vector_2d n(-v[1], v[0]);
  n.normalize();
  vital::vector_2d x0p = p - x0;
  return std::abs(n.dot(x0p));
}

}

class KWIVER_ALGO_CORE_EXPORT triangle_square_iterator
{
public:
  triangle_square_iterator(vital::vector_2d const& pt1,
                           vital::vector_2d const& pt2,
                           vital::vector_2d const& pt3) :
    a(pt1), b(pt2), c(pt3)
  {
    reset();
  }

  void update_xrange()
  {
    int left = tl_corner[0];
    int right = br_corner[0];

    while (left < right) {
      if (is_point_inside_triangle({left, cur_line}, a, b, c))
      {
        break;
      }
      else if (dist_point_line({left, cur_line}, a, b) <= 0.707 ||
               dist_point_line({left, cur_line}, b, c) <= 0.707 ||
               dist_point_line({left, cur_line}, c, a) <= 0.707)
      {
         break;
      }
      else
      {
        ++left;
      }
    }
    while (right > left)
    {
      if (is_point_inside_triangle({right, cur_line}, a, b, c))
      {
        break;
      }
      else if (dist_point_line({right, cur_line}, a, b) <= 0.707 ||
               dist_point_line({right, cur_line}, b, c) <= 0.707 ||
               dist_point_line({right, cur_line}, c, a) <= 0.707)
      {
        break;
      }
      else
      {
        --right;
      }
    }
    x_min = left;
    x_max = right;
  }

  void reset() {
    tl_corner[0] = std::floor(std::min(std::min(a[0], b[0]), c[0]));
    tl_corner[1] = std::floor(std::min(std::min(a[1], b[1]), c[1]));

    br_corner[0] = std::ceil(std::max(std::max(a[0], b[0]), c[0]));
    br_corner[1] = std::ceil(std::max(std::max(a[1], b[1]), c[1]));

    cur_line = tl_corner[1]-1;
  }

  bool next() {
    if (cur_line < br_corner[1]) {
      cur_line++;
      update_xrange();
      return true;
    } else {
      return false;
    }
  }

  int scan_y() const { return cur_line; }

  int start_x() const { return x_min; }

  int end_x() const { return x_max; }

private:
  vital::vector_2d a, b, c;
  vital::vector_2i tl_corner, br_corner;
  int cur_line;
  int x_min, x_max;
};


/// Provides access to the pixels of a triangle using scanlines
class KWIVER_ALGO_CORE_EXPORT triangle_scan_iterator
{
public:
  triangle_scan_iterator(vital::vector_2d const & pt1,
                         vital::vector_2d const & pt2,
                         vital::vector_2d const & pt3) :
    a(pt1), b(pt2), c(pt3)
  {
    reset();
  }

  /// Reset the iterator state
  void reset();

  /// Update the iterator to the next scanline
  bool next();

  /// Current scanline index
  int scan_y() const { return scan_y_; }

  /// Index of the first pixel of the current scanline
  int start_x() const { return start_x_; }

  /// Index of the last pixel of the current scanline
  int end_x() const { return end_x_; }

private:
  vital::vector_2d const &a, &b, &c;
  vital::vector_2d g;
  int scan_y_;
  int start_x_, end_x_;
  int x0, y0, x1, y1;
  double data[3][3];
};



}
}
}


#endif // KWIVER_ARROWS_CORE_TRIANGLE_SCAN_ITERATOR_H
