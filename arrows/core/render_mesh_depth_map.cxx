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
 * \brief Header for core render_mesh_depth_map function
 */

#include "render_mesh_depth_map.h"

#include <vital/types/camera_perspective.h>
#include <vital/types/image.h>
#include <vital/types/vector.h>


namespace kwiver {
namespace arrows {
namespace core {

vital::image_container_sptr render_mesh_depth_map(vital::mesh_sptr mesh, vital::camera_perspective_sptr camera)
{
  vital::mesh_vertex_array<3>& vertices = dynamic_cast< vital::mesh_vertex_array<3>& >(mesh->vertices());

  std::vector<vital::vector_2d> points_2d(vertices.size());
  for (unsigned int i = 0; i < vertices.size(); ++i)
  {
    points_2d[i] = camera->project(vertices[i]);
  }

  vital::image_of<double> zbuffer(camera->image_width(), camera->image_height(), 1);
  for (unsigned int j = 0; j < zbuffer.height(); ++j)
  {
    for (unsigned int i = 0; i < zbuffer.width(); ++i)
    {
      zbuffer(i, j) = std::numeric_limits<double>::infinity();
    }
  }

  if (mesh->faces().regularity() == 3)
  {
    auto const& triangles = static_cast< const vital::mesh_regular_face_array<3>& >(mesh->faces());
    double d1, d2, d3;
    for (unsigned int f = 0; f < triangles.size(); ++f)
    {
      vital::vector_2d& v1 = points_2d[triangles(f, 0)];
      vital::vector_2d& v2 = points_2d[triangles(f, 1)];
      vital::vector_2d& v3 = points_2d[triangles(f, 2)];

      d1 = - 1.0 / camera->depth(vertices[triangles(f, 0)]);
      d2 = - 1.0 / camera->depth(vertices[triangles(f, 1)]);
      d3 = - 1.0 / camera->depth(vertices[triangles(f, 2)]);

      render_triangle(v1, v2, v3, d1, d2, d3, zbuffer);
    }

    for (unsigned int j = 0; j < zbuffer.height(); ++j)
    {
      for (unsigned int i = 0; i < zbuffer.width(); ++i)
      {
        if (!std::isinf(zbuffer(i, j)))
          zbuffer(i, j) = -1.0 / zbuffer(i, j);
      }
    }
  }
  else
  {
    LOG_ERROR(vital::get_logger("arrows.core.render_mesh_depth_map" ), "The mesh has to be triangular.");
  }
  return std::make_shared<vital::simple_image_container>(zbuffer);
}


vital::image_container_sptr render_mesh_height_map(vital::mesh_sptr mesh, vital::camera_sptr camera)
{
  vital::mesh_vertex_array<3>& vertices = dynamic_cast< vital::mesh_vertex_array<3>& >(mesh->vertices());

  std::vector<vital::vector_2d> points_2d(vertices.size());
  for (unsigned int i = 0; i < vertices.size(); ++i)
  {
    points_2d[i] = camera->project(vertices[i]);
  }

  vital::image_of<double> zbuffer(camera->image_width(), camera->image_height(), 1);
  vital::image_of<double> height_map(camera->image_width(), camera->image_height(), 1);
  for (unsigned int j = 0; j < zbuffer.height(); ++j)
  {
    for (unsigned int i = 0; i < zbuffer.width(); ++i)
    {
      zbuffer(i, j) = std::numeric_limits<double>::infinity();
      height_map(i, j) = std::numeric_limits<double>::infinity();
    }
  }

  if (mesh->faces().regularity() == 3)
  {
    auto const& triangles = static_cast< const vital::mesh_regular_face_array<3>& >(mesh->faces());
    double d1, d2, d3;
    double h1, h2, h3;
    vital::camera_perspective* perspective_camera = dynamic_cast<vital::camera_perspective*>(camera.get());
    for (unsigned int f = 0; f < triangles.size(); ++f)
    {
      vital::vector_2d& v1 = points_2d[triangles(f, 0)];
      vital::vector_2d& v2 = points_2d[triangles(f, 1)];
      vital::vector_2d& v3 = points_2d[triangles(f, 2)];

      h1 = vertices[triangles(f, 0)](2);
      h2 = vertices[triangles(f, 1)](2);
      h3 = vertices[triangles(f, 2)](2);
      if (perspective_camera)
      {
        d1 = -1.0 / perspective_camera->depth(vertices[triangles(f, 0)]);
        d2 = -1.0 / perspective_camera->depth(vertices[triangles(f, 1)]);
        d3 = -1.0 / perspective_camera->depth(vertices[triangles(f, 2)]);

        h1 *= -d1;
        h2 *= -d2;
        h3 *= -d3;
      }
      else
      {
        d1 = -vertices[triangles(f, 0)](2);
        d2 = -vertices[triangles(f, 1)](2);
        d3 = -vertices[triangles(f, 2)](2);
      }

      render_triangle<double>(v1, v2, v3,
                              d1, d2, d3,
                              h1, h2, h3,
                              zbuffer, height_map);
    }

    if (perspective_camera)
    {
      for (unsigned int j = 0; j < zbuffer.height(); ++j)
      {
        for (unsigned int i = 0; i < zbuffer.width(); ++i)
        {
          if (!std::isinf(zbuffer(i, j)))
          {
            zbuffer(i, j) = -1.0 / zbuffer(i, j);
            height_map(i, j) *= zbuffer(i, j);
          }
        }
      }
    }
  }
  else
  {
    LOG_ERROR(vital::get_logger("arrows.core.render_mesh_depth_map" ), "The mesh has to be triangular.");
  }
  return std::make_shared<vital::simple_image_container>(height_map);
}


void render_triangle(const vital::vector_2d& v1, const vital::vector_2d& v2, const vital::vector_2d& v3,
                     double depth_v1, double depth_v2, double depth_v3,
                     vital::image_of<double>& depth_img)
{
  vital::triangle_scan_iterator tsi(v1, v2, v3);

  // Linear interpolation depth
  vital::vector_3d b1(v2.x()-v1.x(), v2.y()-v1.y(), depth_v2 - depth_v1);
  vital::vector_3d b2(v3.x()-v1.x(), v3.y()-v1.y(), depth_v3 - depth_v1);
  vital::vector_3d n = b1.cross(b2);
  double A = -n.x()/n.z();
  double B = -n.y()/n.z();
  double C = (v1.x() * n.x() + v1.y() * n.y() + depth_v1 * n.z()) / n.z();

  for (tsi.reset(); tsi.next(); )
  {
    int y = tsi.scan_y();
    if (y < 0 || y >= static_cast<int>(depth_img.height()))
      continue;
    int min_x = std::max(0, tsi.start_x());
    int max_x = std::min(static_cast<int>(depth_img.width()) - 1, tsi.end_x());

    double new_i = B * y + C;
    for (int x = min_x; x <= max_x; ++x)
    {
      double depth = new_i + A * x;
      if (depth < depth_img(x, y))
      {
        depth_img(x, y) = depth;
      }
    }
  }
}

}
}
}
