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


/// Provides access to the pixels of a triangle using scanlines and possible bounds
class KWIVER_ALGO_CORE_EXPORT triangle_scan_iterator
{
public:

  /// Provides access to the pixels on a scanline
  class scan_line_iterator
  {
  public:
    /// Constructor from start, stop (not included) and scanline coordinate
    scan_line_iterator(int x_start, int x_stop, int y) :
      start(x_start), stop(x_stop), cur(x_start), scan_y(y)
    {
      if (stop < start)
        stop = start;
    }

    /// Constructor from start, stop (not included), scanline coordinate and current
    scan_line_iterator(int x_start, int x_stop, int y, int x_cur) :
      start(x_start), stop(x_stop), cur(x_cur), scan_y(y)
    {
      stop = std::max(stop, start);
      cur = std::max(cur, start);
      cur = std::min(cur, stop);
    }

    /// Iterator to the first pixel
    scan_line_iterator begin()
    {
      return scan_line_iterator(start, stop, scan_y);
    }

    /// Iterator to the end of the scanline
    scan_line_iterator end()
    {
      return scan_line_iterator(start, stop, scan_y, stop);
    }

    /// Update the iterator to the next pixel
    scan_line_iterator const& operator++()
    {
      ++cur;
      return *this;
    }

    bool operator ==(scan_line_iterator const& other)
    {
      return start == other.start && stop == other.stop && cur == other.cur && scan_y == other.scan_y;
    }

    bool operator !=(scan_line_iterator const& other)
    {
      return !(*this == other);
    }

    /// Provide access to the current x
    int operator*()
    {
      return cur;
    }

    /// Provide access to the scanline coordinate
    int y() const
    {
      return scan_y;
    }

  private:
    int start, stop, cur;
    int scan_y;
  };

  /// Constructor from 3 points and optional bounds
  triangle_scan_iterator(vital::vector_2d const & pt1,
                          vital::vector_2d const & pt2,
                          vital::vector_2d const & pt3,
                          vital::vector_4i const& image_bounds = vital::vector_4i(std::numeric_limits<int>::min(),
                                                                                  std::numeric_limits<int>::min(),
                                                                                  std::numeric_limits<int>::max()-1,
                                                                                  std::numeric_limits<int>::max()-1));

  /// Return an iterator to the first scanline of the triangle (inside the bounds)
  triangle_scan_iterator begin();

  /// Return an iterator pointing to the end
  triangle_scan_iterator end();

  /// Update the iterator to the next scanline
  triangle_scan_iterator const& operator ++();

  /// Access to the current scanline
  scan_line_iterator operator*()
  {
    return scan_line_iterator(std::max(start_x_, bounds[0]),
                              std::min(end_x_+1, bounds[2]+1),
                              scan_y_);
  }

  bool operator ==(triangle_scan_iterator const& other)
  {
    return a == other.a && b == other.b && c == other.c && scan_y_ == other.scan_y_;
  }
  bool operator !=(triangle_scan_iterator const& other)
  {
    return !(*this == other);
  }

private:
  vital::vector_2d const &a, &b, &c;
  vital::vector_2d g;
  int scan_y_;
  int start_x_, end_x_;
  int x0, y0, x1, y1;
  double data[3][3];
  vital::vector_4i bounds;

  /// Update the current scanline range
  void update_scan_line_range();
};

}
}
}


#endif // KWIVER_ARROWS_CORE_TRIANGLE_SCAN_ITERATOR_H
