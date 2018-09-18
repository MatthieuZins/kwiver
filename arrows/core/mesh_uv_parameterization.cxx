/*ckwg +29
 * Copyright 2017-2018 by Kitware, Inc.
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
 * \brief Core mesh uv parameterization implementation
 */

#include "mesh_uv_parameterization.h"
#include <vital/util/cpu_timer.h>
#include <algorithm>
#include <iostream>

using namespace kwiver::vital;

namespace {
struct triangle_t
{
  kwiver::vital::vector_2d a;
  kwiver::vital::vector_2d b;
  kwiver::vital::vector_2d c;
  int face_id;
  double height;
  void get_bounds(double bounds[0])
  {
    bounds[0] = std::min(a[0], std::min(b[0], c[0]));
    bounds[1] = std::max(a[0], std::max(b[0], c[0]));

    bounds[2] = std::min(a[1], std::min(b[1], c[1]));
    bounds[3] = std::max(a[1], std::max(b[1], c[1]));
  }
};
}

namespace kwiver {
namespace arrows {
namespace core {


/// Compute a UV parameterization of the mesh in the form of a rectangular texture atas and set texture coordinates
std::pair<unsigned int, unsigned int> parameterize(kwiver::vital::mesh_sptr mesh, double resolution,
                                                   int max_width, int interior_margin, int exterior_margin)
{
  std::vector<triangle_t> triangles(mesh->num_faces());
  const mesh_face_array_base& faces = mesh->faces();
  const mesh_vertex_array_base& vertices = mesh->vertices();

  // Transform each 3d triangular face to a 2d triangle so
  // that the longest edge is horizontal and its left point is (0, 0)
  for (int f=0; f < static_cast<int>(mesh->num_faces()); ++f)
  {
    // get the face points
    vector_3d pt1 = dynamic_cast<const mesh_vertex_array<3>&>(vertices)[faces(f, 0)];
    vector_3d pt2 = dynamic_cast<const mesh_vertex_array<3>&>(vertices)[faces(f, 1)];
    vector_3d pt3 = dynamic_cast<const mesh_vertex_array<3>&>(vertices)[faces(f, 2)];

    vector_3d pt1pt2 = pt2 - pt1;
    vector_3d pt1pt3 = pt3 - pt1;
    vector_3d pt2pt3 = pt3 - pt2;

    if (pt1pt2.norm() == 0 || pt1pt3.norm() == 0 || pt2pt3.norm() == 0)
    {
      triangles[f] = {{0,0}, {0,0}, {0,0}, f, 0};
    }
    else
    {
      // find the longest edge and assign it to AB
      vector_3d AB, AC;
      int longest_edge;
      if (pt1pt2.norm() >= pt1pt3.norm() && pt1pt2.norm() >= pt2pt3.norm())
      {
        // pt1 is A, pt2 is B, pt3 is C
        AB = pt1pt2;
        AC = pt1pt3;
        longest_edge = 0;
      }
      else if (pt2pt3.norm() >= pt1pt3.norm())
      {
        // pt1 is C, pt2 is A, pt3 is B
        AB = pt2pt3;
        AC = -pt1pt2;
        longest_edge = 1;
      }
      else
      {
        // pt1 is B, pt2 is C, pt3 is A
        AB = -pt1pt3;
        AC = -pt2pt3;
        longest_edge = 2;
      }
      // Transform the face to 2d
      vector_2d a(0.0, 0.0);
      vector_2d b(AB.norm(), 0.0);
      double proj = AC.dot(AB) / AB.norm();
      vector_2d c(proj, (AC - proj * AB.normalized()).norm());
      // Scale B and C by the resolution
      b /= resolution;
      c /= resolution;

      if (longest_edge == 0)
      {
        // pt1 is A, pt2 is B, pt3 is C
        triangles[f] = {a, b, c, f, c[1]};
      }
      else if (longest_edge == 1)
      {
        // pt1 is C, pt2 is A, pt3 is B
        triangles[f] = {c, a, b, f, c[1]};
      }
      else if (longest_edge == 2)
      {
        // pt1 is B, pt2 is C, pt3 is A
        triangles[f] = {b, c, a, f, c[1]};
      }
    }
  }

  // sort triangles by height
  std::sort(triangles.begin(), triangles.end(), [](const triangle_t& t1,
            const triangle_t& t2)
  {
    return t1.height < t2.height;
  });


  // pack triangles
  std::vector<vector_2d> tcoords(mesh->num_faces() * 3);

  double current_x = 0.0;
  double current_y = 0.0;
  double next_y = 0.0;

  current_x = -interior_margin;  // initialized to -interior_margin so that the first triangle is at x=0
  double bounds[4];
  for (int i=0; i < static_cast<int>(triangles.size()); ++i)
  {
    triangles[i].get_bounds(bounds);
    current_x += -bounds[0] + interior_margin;
    if (current_x + bounds[1] > max_width)
    {
      current_x = -bounds[0];
      current_y = next_y + interior_margin;
    }
    vector_2d position(current_x, current_y);
    tcoords[triangles[i].face_id * 3 + 0] = triangles[i].a + position;
    tcoords[triangles[i].face_id * 3 + 1] = triangles[i].b + position;
    tcoords[triangles[i].face_id * 3 + 2] = triangles[i].c + position;

    current_x += bounds[1];
    next_y = std::max(next_y, current_y + (bounds[3]-bounds[2]));
  }

  // add margin at the top and left borders
  vector_2d margin(exterior_margin, exterior_margin);
  for (int it=0; it < static_cast<int>(tcoords.size()); ++it)
  {
    tcoords[it] += margin;
  }

  // add margin at the bottom and right borders
  double max_x = (*std::max_element(tcoords.begin(), tcoords.end(), [](const vector_2d& v1, const vector_2d& v2) {
    return v1[0] < v2[0];
  }))[0];
  double max_y = (*std::max_element(tcoords.begin(), tcoords.end(), [](const vector_2d& v1, const vector_2d& v2) {
    return v1[1] < v2[1];
  }))[1];

  unsigned int width = std::ceil(max_x) + 1 + exterior_margin;
  unsigned int height = std::ceil(max_y) + 1 + exterior_margin;

  // normalize tcoords
  for (vector_2d& v: tcoords)
  {
    v(0) /= width;
    v(1) /= height;
  }

  mesh->set_tex_coords(tcoords);

  return std::make_pair(width, height);
}

}
}
}
