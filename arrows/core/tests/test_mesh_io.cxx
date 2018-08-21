/*ckwg +29
 * Copyright 2015-2017 by Kitware, Inc.
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

#include <test_gtest.h>

#include <arrows/core/mesh_io.h>
#include <vital/plugin_loader/plugin_manager.h>

#include <gtest/gtest.h>

#include <algorithm>

using namespace kwiver::vital;

std::vector<vector_3d> verts;
std::vector< mesh_regular_face<3> > faces;
std::vector<vector_3d> normals;
std::vector<vector_2d> tcoords(faces.size() * 3);


// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{
    verts.push_back({-0.500000, -0.500000, -0.500000});
    verts.push_back({-0.500000, -0.500000, 0.500000});
    verts.push_back({-0.500000, 0.500000, -0.500000});
    verts.push_back({-0.500000, 0.500000, 0.500000});
    verts.push_back({0.500000 ,-0.500000, -0.500000});
    verts.push_back({0.500000 ,-0.500000, 0.500000});
    verts.push_back({0.500000 ,0.500000 ,-0.500000});
    verts.push_back({0.500000 ,0.500000 ,0.500000});

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

    normals.push_back({-1,  0,  0});
    normals.push_back({-1,  0,  0});
    normals.push_back({ 1,  0,  0});
    normals.push_back({ 1,  0,  0});
    normals.push_back({ 0, -1,  0});
    normals.push_back({ 0, -1,  0});
    normals.push_back({ 0,  1,  0});
    normals.push_back({ 0,  1,  0});
    normals.push_back({ 0,  0, -1});
    normals.push_back({ 0,  0, -1});
    normals.push_back({ 0,  0,  1});
    normals.push_back({ 0,  0,  1});


    for (int i=0; i < faces.size() * 3; ++i)
    {
        tcoords.push_back({1.0 / (1.0 + i % 10),
                           1.0 / (1.0 + i % 4)});
    }

  ::testing::InitGoogleTest( &argc, argv );
  TEST_LOAD_PLUGINS();

  return RUN_ALL_TESTS();
}

// ----------------------------------------------------------------------------
TEST(mesh_io, vertices_faces)
{
    std::unique_ptr<mesh_vertex_array_base> vertices_array_ptr(new mesh_vertex_array<3>(verts));
    std::unique_ptr<mesh_face_array_base> faces_array_ptr(new mesh_regular_face_array<3>(faces));
    mesh_sptr mesh(new kwiver::vital::mesh(std::move(vertices_array_ptr), std::move(faces_array_ptr)));

    algo::mesh_io_sptr mesh_io = algo::mesh_io::create("core");
    std::string temp_filename("temp_mesh.obj");
    mesh_io->save(temp_filename, mesh);
    mesh_sptr reloaded_mesh = mesh_io->load(temp_filename);

    EXPECT_EQ(reloaded_mesh->num_faces(), faces.size());
    EXPECT_EQ(reloaded_mesh->num_verts(), verts.size());

    const mesh_vertex_array_base& reloaded_verts = reloaded_mesh->vertices();
    for (unsigned int i=0; i < reloaded_mesh->num_verts(); ++i)
    {
        for (int d=0; d < 3; ++d)
        {
            EXPECT_NEAR(reloaded_verts(i, d), verts[i][d], 1e-15);
        }
    }

    const mesh_face_array_base& reloaded_faces = reloaded_mesh->faces();
    for (unsigned int f=0; f < reloaded_faces.size(); ++f)
    {
        for (int v=0; v < 3; ++v)
        {
            EXPECT_NEAR(reloaded_faces(f, v), faces[f][v], 1e-15);
        }
    }
}

// ----------------------------------------------------------------------------
TEST(mesh_io, normals)
{
    std::unique_ptr<mesh_vertex_array_base> vertices_array_ptr(new mesh_vertex_array<3>(verts));
    std::unique_ptr<mesh_face_array_base> faces_array_ptr(new mesh_regular_face_array<3>(faces));
    mesh_sptr mesh(new kwiver::vital::mesh(std::move(vertices_array_ptr), std::move(faces_array_ptr)));
    mesh->faces().set_normals(normals);

    algo::mesh_io_sptr mesh_io = algo::mesh_io::create("core");
    std::string temp_filename("temp_mesh.obj");
    mesh_io->save(temp_filename, mesh);
    mesh_sptr reloaded_mesh = mesh_io->load(temp_filename);

    EXPECT_EQ(reloaded_mesh->num_faces(), faces.size());
    EXPECT_EQ(reloaded_mesh->num_verts(), verts.size());

    const mesh_vertex_array_base& reloaded_verts = reloaded_mesh->vertices();
    for (unsigned int i=0; i < reloaded_mesh->num_verts(); ++i)
    {
        for (int d=0; d < 3; ++d)
        {
            EXPECT_NEAR(reloaded_verts(i, d), verts[i][d], 1e-15);
        }
    }

    const mesh_face_array_base& reloaded_faces = reloaded_mesh->faces();
    for (unsigned int f=0; f < reloaded_faces.size(); ++f)
    {
        for (int v=0; v < 3; ++v)
        {
            EXPECT_NEAR(reloaded_faces(f, v), faces[f][v], 1e-15);
        }
    }

    const std::vector<vector_3d>& reloaded_normals = mesh->faces().normals();
    for (unsigned int f=0; f < normals.size(); ++f)
    {
        for (unsigned int i=0; i < 3; ++i)
        {
            EXPECT_NEAR(reloaded_normals[f][i], normals[f][i], 1e-15);
        }
    }
}

// ----------------------------------------------------------------------------
TEST(mesh_io, tex_coords)
{
    std::unique_ptr<mesh_vertex_array_base> vertices_array_ptr(new mesh_vertex_array<3>(verts));
    std::unique_ptr<mesh_face_array_base> faces_array_ptr(new mesh_regular_face_array<3>(faces));
    mesh_sptr mesh(new kwiver::vital::mesh(std::move(vertices_array_ptr), std::move(faces_array_ptr)));
    mesh->set_tex_coords(tcoords);

    algo::mesh_io_sptr mesh_io = algo::mesh_io::create("core");
    std::string temp_filename("temp_mesh.obj");
    mesh_io->save(temp_filename, mesh);
    mesh_sptr reloaded_mesh = mesh_io->load(temp_filename);

    EXPECT_EQ(reloaded_mesh->num_faces(), faces.size());
    EXPECT_EQ(reloaded_mesh->num_verts(), verts.size());

    const mesh_vertex_array_base& reloaded_verts = reloaded_mesh->vertices();
    for (unsigned int i=0; i < reloaded_mesh->num_verts(); ++i)
    {
        for (int d=0; d < 3; ++d)
        {
            EXPECT_NEAR(reloaded_verts(i, d), verts[i][d], 1e-15);
        }
    }

    const mesh_face_array_base& reloaded_faces = reloaded_mesh->faces();
    for (unsigned int f=0; f < reloaded_faces.size(); ++f)
    {
        for (int v=0; v < 3; ++v)
        {
            EXPECT_NEAR(reloaded_faces(f, v), faces[f][v], 1e-15);
        }
    }

    const std::vector<vector_2d>& reloaded_tcoords = mesh->tex_coords();
    for (unsigned int f=0; f < faces.size(); ++f)
    {
        for (unsigned int i=0; i < 3; ++i)
        {
            EXPECT_NEAR(reloaded_tcoords[f * 3 + i][0], tcoords[f * 3 + i][0], 1e-15);
            EXPECT_NEAR(reloaded_tcoords[f * 3 + i][1], tcoords[f * 3 + i][1], 1e-15);
        }
    }
}

// ----------------------------------------------------------------------------
TEST(mesh_io, all)
{
      std::unique_ptr<mesh_vertex_array_base> vertices_array_ptr(new mesh_vertex_array<3>(verts));
      std::unique_ptr<mesh_face_array_base> faces_array_ptr(new mesh_regular_face_array<3>(faces));
      mesh_sptr mesh(new kwiver::vital::mesh(std::move(vertices_array_ptr), std::move(faces_array_ptr)));
      mesh->faces().set_normals(normals);
      mesh->set_tex_coords(tcoords);

      algo::mesh_io_sptr mesh_io = algo::mesh_io::create("core");
      std::string temp_filename("temp_mesh.obj");
      mesh_io->save(temp_filename, mesh);
      mesh_sptr reloaded_mesh = mesh_io->load(temp_filename);

      EXPECT_EQ(reloaded_mesh->num_faces(), faces.size());
      EXPECT_EQ(reloaded_mesh->num_verts(), verts.size());

      const mesh_vertex_array_base& reloaded_verts = reloaded_mesh->vertices();
      for (unsigned int i=0; i < reloaded_mesh->num_verts(); ++i)
      {
          for (int d=0; d < 3; ++d)
          {
              EXPECT_NEAR(reloaded_verts(i, d), verts[i][d], 1e-15);
          }
      }

      const mesh_face_array_base& reloaded_faces = reloaded_mesh->faces();
      for (unsigned int f=0; f < reloaded_faces.size(); ++f)
      {
          for (int v=0; v < 3; ++v)
          {
              EXPECT_NEAR(reloaded_faces(f, v), faces[f][v], 1e-15);
          }
      }

      const std::vector<vector_3d>& reloaded_normals = mesh->faces().normals();
      for (unsigned int f=0; f < normals.size(); ++f)
      {
          for (unsigned int i=0; i < 3; ++i)
          {
              EXPECT_NEAR(reloaded_normals[f][i], normals[f][i], 1e-15);
          }
      }

      const std::vector<vector_2d>& reloaded_tcoords = mesh->tex_coords();
      for (unsigned int f=0; f < faces.size(); ++f)
      {
          for (unsigned int i=0; i < 3; ++i)
          {
              EXPECT_NEAR(reloaded_tcoords[f * 3 + i][0], tcoords[f * 3 + i][0], 1e-15);
              EXPECT_NEAR(reloaded_tcoords[f * 3 + i][1], tcoords[f * 3 + i][1], 1e-15);
          }
      }
}

