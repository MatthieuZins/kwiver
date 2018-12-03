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
 * \brief core triangle_scan_iterator class tests
 */

#include <arrows/core/triangle_scan_iterator.h>
#include <vital/types/vector.h>
#include <gtest/gtest.h>

using namespace kwiver::vital;

// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ::testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}

namespace
{
const vector_2d pt1(0, 0);
const vector_2d pt2(10, 0);
const vector_2d pt3(5, 10);
const int x_pos[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 1, 2, 3, 4,
                     5, 6, 7, 8, 9, 2, 3, 4, 5, 6, 7, 8, 2, 3, 4, 5, 6, 7, 8, 3, 4, 5, 6, 7,
                     3, 4, 5, 6, 7, 4, 5, 6, 4, 5, 6, 5, 5};
const int y_pos[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2,
                     2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6,
                     6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 10};
const kwiver::vital::vector_4i bounds(3, 3, 7, 7);
const int x_pos_bounded[] = {3, 4, 5, 6, 7, 3, 4, 5, 6, 7, 3, 4, 5, 6, 7, 3, 4, 5, 6, 7, 4, 5, 6};
const int y_pos_bounded[] = {3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7};
}

// ----------------------------------------------------------------------------
TEST(triangle_scan_iterator, iterate)
{
  int i = 0;
  for (auto it : kwiver::arrows::core::triangle_scan_iterator(pt1, pt2, pt3))
  {
    int y = it.y();
    for (auto x : it)
    {
      EXPECT_EQ(y, y_pos[i]);
      EXPECT_EQ(x, x_pos[i]);
      ++i;
    }
  }
}

// ----------------------------------------------------------------------------
TEST(triangle_scan_iterator, iterate_bounded)
{
  int i = 0;
  for (auto it : kwiver::arrows::core::triangle_scan_iterator(pt1, pt2, pt3, bounds))
  {
    int y = it.y();
    for (auto x : it)
    {
      EXPECT_EQ(y, y_pos_bounded[i]);
      EXPECT_EQ(x, x_pos_bounded[i]);
      ++i;
    }
  }
}
