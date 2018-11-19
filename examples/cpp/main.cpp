/*ckwg +29
 * Copyright 2017 by Kitware, Inc.
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

// Predefine methods that show off various functionality in kwiver
void how_to_part_01_images();
void how_to_part_02_detections();

#include <vital/types/mesh.h>
#include <vital/algo/uv_unwrap_mesh.h>
#include <vital/io/mesh_io.h>
#include <vital/plugin_loader/plugin_manager.h>
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

int main()
{
  using namespace kwiver::vital;
  // use comments to execute a particular method
  kwiver::vital::plugin_manager::instance().load_all_plugins();
//  how_to_part_01_images();
//  how_to_part_02_detections();


  // cube mesh of size 1.0
  std::vector<vector_3d> verts = {
        {-0.500000, -0.500000, -0.500000},
        {-0.500000, -0.500000, 0.500000},
        {-0.500000, 0.500000, -0.500000},
        {-0.500000, 0.500000, 0.500000},
        {0.500000 ,-0.500000, -0.500000},
        {0.500000 ,-0.500000, 0.500000},
        {0.500000 ,0.500000 ,-0.500000},
        {0.500000 ,0.500000 ,0.500000}
  };
  std::vector< mesh_regular_face<3> > faces;
  faces.push_back(mesh_regular_face<3>({0, 1, 2}));
  faces.push_back(mesh_regular_face<3>({3, 2, 1}));
  faces.push_back(mesh_regular_face<3>({4, 6, 5}));
  faces.push_back(mesh_regular_face<3>({7, 5, 6}));
  faces.push_back(mesh_regular_face<3>({0, 4, 1}));
  faces.push_back(mesh_regular_face<3>({5, 1, 4}));
  faces.push_back(mesh_regular_face<3>({2, 3, 6}));
  faces.push_back(mesh_regular_face<3>({7, 6, 3}));
  faces.push_back(mesh_regular_face<3>({0, 2, 4}));
  faces.push_back(mesh_regular_face<3>({6, 4, 2}));
  faces.push_back(mesh_regular_face<3>({1, 5, 3}));
  faces.push_back(mesh_regular_face<3>({7, 3, 5}));

  std::unique_ptr<mesh_vertex_array_base> vertices_array_ptr(new mesh_vertex_array<3>(verts));
  std::unique_ptr<mesh_face_array_base> faces_array_ptr(new mesh_regular_face_array<3>(faces));
//  mesh_sptr m = std::make_shared<mesh>(std::move(vertices_array_ptr), std::move(faces_array_ptr));



//  kwiver::vital::mesh_sptr m = kwiver::vital::read_obj("/home/matthieu/data_plane/f16.obj");
  kwiver::vital::mesh_sptr m = kwiver::vital::read_obj("/home/matthieu/data_towers/towers.obj");
  std::unique_ptr< kwiver::vital::mesh_regular_face_array<3> > regular_faces(new kwiver::vital::mesh_regular_face_array<3>);
   for (int i = 0; i < m->faces().size(); ++i)
   {
     kwiver::vital::mesh_regular_face<3> f;
     f[0] = m->faces()(i, 0);
     f[1] = m->faces()(i, 1);
     f[2] = m->faces()(i, 2);
     regular_faces->push_back(f);
   }
   m->set_faces(std::move(regular_faces));



  kwiver::vital::algo::uv_unwrap_mesh_sptr uv_param =
      kwiver::vital::algo::uv_unwrap_mesh::create("core");


  uv_param->parameterize(m);


  /// get factor
  mesh_vertex_array<3> const& vertices = dynamic_cast< mesh_vertex_array<3> const& >(m->vertices());


  const std::vector<vector_2d>& tcoords = m->tex_coords();

  double area_2d = 0.0, area_3d = 0.0;
  int f = 0;
  while (area_2d < 1e-15)
  {
    auto tc1 = tcoords[f * 3 + 0];
    auto tc2 = tcoords[f * 3 + 1];
    auto tc3 = tcoords[f * 3 + 2];
    vector_2d a2 = tc2 - tc1;
    vector_2d b2 = tc3 - tc1;
    area_2d = a2(0) * b2(1) - a2(1) * b2(0);

    auto const& v1 = vertices[m->faces()(f, 0)];
    auto const& v2 = vertices[m->faces()(f, 1)];
    auto const& v3 = vertices[m->faces()(f, 2)];
    vector_3d a = v2 - v1;
    vector_3d b = v3 - v1;
    area_3d = a.cross(b).norm();

    f++;
  }

  std::cout << "area 3d = " << area_3d << std::endl;
  std::cout << "area 2d = " << area_2d << std::endl;
  double resolution = 0.03;
  int factor = static_cast<int>(std::ceil(sqrt(area_3d / area_2d) / resolution));
  std::cout << factor << std::endl;

  std::cout << m->has_tex_coords() << std::endl;

  std::vector<vector_2d> tcoords_un(tcoords.size(), vector_2d(0, 0));
  for (int i = 0; i < tcoords.size(); ++i)
  {
    tcoords_un[i][0] = tcoords[i][0] * factor; //atlas_dim.first;
    tcoords_un[i][1] = tcoords[i][1] * factor; //atlas_dim.second;
  }

  std::vector<cv::Point2d> points(tcoords.size());
  for (int i = 0; i < tcoords_un.size(); ++i)
  {
    points[i].x = tcoords_un[i][0];
    points[i].y = tcoords_un[i][1];
  }

  cv::Mat texture = cv::Mat::zeros(factor, factor, CV_8U);
  for (int i = 0; i < m->num_faces(); ++i)
  {
    for (int a = 0, b = 2; a < 3; b = a++)
    {
      cv::line(texture, points[i * 3 + a], points[i * 3 + b], 255);
    }
  }
  cv::imwrite("atlas.png", texture);

}
