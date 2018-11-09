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
#include <vital/io/mesh_io.h>
#include <vital/algo/compute_mesh_uv_parameterization.h>
#include <vital/plugin_loader/plugin_manager.h>
#include <iostream>

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
  mesh_sptr m = std::make_shared<mesh>(std::move(vertices_array_ptr), std::move(faces_array_ptr));


  double resolution = 0.03;   // mesh unit/pixel
  int interior_margin = 2;
  int exterior_margin = 2;

  kwiver::vital::algo::compute_mesh_uv_parameterization_sptr uv_param =
      kwiver::vital::algo::compute_mesh_uv_parameterization::create("core");

  kwiver::vital::config_block_sptr algo_config = uv_param->get_configuration();
  algo_config->set_value<double>("resolution", resolution);
  algo_config->set_value<double>("interior_margin", interior_margin);
  algo_config->set_value<double>("exterior_margin", exterior_margin);
  uv_param->set_configuration(algo_config);

  std::pair<unsigned int, unsigned int> atlas_dim = uv_param->parameterize(m);
  std::cout << atlas_dim.first << " " << atlas_dim.second << std::endl;

  std::cout << m->has_tex_coords() << std::endl;
  std::vector<kwiver::vital::vector_2d> tcoords = m->tex_coords();

  for (auto tc : tcoords)
  {
    std::cout << tc(0) << " " << tc(1) << std::endl;
  }

  kwiver::vital::write_obj("mesh_ou.obj", *m);
}
