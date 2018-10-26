/*ckwg +29
 * Copyright 2014-2018 by Kitware, Inc.
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
 * \brief test core camera class
 */

#include <test_eigen.h>

#include <vital/types/camera_affine.h>
#include <vital/io/camera_io.h>

#include <iostream>

using namespace kwiver::vital;

// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ::testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}

// ----------------------------------------------------------------------------
TEST(camera_affine, clone)
{
  simple_camera_affine cam(vector_3d(0, 0, -1), vector_3d(0, 1, 0),
                           vector_3d(0, 0, 0), vector_2d(50, 50),
                           vector_2d(1, 1), 100, 100);


  vector_3d pt(50, -50, 0);

  auto cam_clone = std::dynamic_pointer_cast<camera_affine>( cam.clone() );
  EXPECT_MATRIX_EQ( cam.get_matrix(), cam_clone->get_matrix() );
}

// ----------------------------------------------------------------------------
TEST(camera_affine, projection)
{
  double scale = 4.0;
  simple_camera_affine cam(vector_3d(1, 1, -1), vector_3d(0, 0, 1),
                           vector_3d(5, 5, 0), vector_2d(50, 50),
                           vector_2d(scale, scale), 100, 100);


  auto res = cam.project({5, 5, 0});
  EXPECT_NEAR(res(0), 50, 1e-12);
  EXPECT_NEAR(res(1), 50, 1e-12);
  res = cam.project({0, 0, 0});
  EXPECT_NEAR(res(0), 50.0, 1e-12);

  double d = sin(0.6154797086703873) * sqrt(2.0) * 5;
  EXPECT_NEAR(res(1), 50 + d * scale, 1e-5);
}

// ----------------------------------------------------------------------------
TEST(camera_affine, depth)
{
  matrix_3x4d mat;
  mat << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 0, 1;

  simple_camera_affine cam(mat, 100, 100);

  cam.set_viewing_distance(-10);

  double res = cam.depth({0, 0, 30});
  EXPECT_NEAR(res, 40.0, 1e-12);

  simple_camera_affine cam2(vector_3d(1, 1, 1), vector_3d(0, 0, 1),
                            vector_3d(10, 10, 10), vector_2d(50, 50),
                            vector_2d(1, 1), 100, 100);

  cam2.set_viewing_distance(-10);
  res = cam2.depth({10, 10, 10});
  EXPECT_NEAR(res, 10 + 10*sqrt(3), 1e-12);
}
