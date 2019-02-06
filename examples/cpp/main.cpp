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

#include <vital/algo/image_io.h>
#include <vital/exceptions.h>
#include <vital/io/mesh_io.h>
#include <vital/plugin_loader/plugin_manager.h>
#include <vital/types/camera.h>
#include <vital/io/camera_io.h>
#include <vital/types/camera_perspective.h>
#include <vital/types/image_container.h>
#include <arrows/core/generate_texture.h>
#include <kwiversys/SystemTools.hxx>
#include <fstream>

#include <opencv2/opencv.hpp>

#define PROFILE(str, ...)      \
{                              \
    std::cout << str << " ... \t\t" << std::flush;    \
    clock_t profile_t0 = clock();   \
    __VA_ARGS__;                    \
    std::cout << double(clock() - profile_t0) / CLOCKS_PER_SEC   \
    << " s" << std::endl;           \
}


// Predefine methods that show off various functionality in kwiver
void how_to_part_01_images();
void how_to_part_02_detections();

namespace {
std::pair<kwiver::vital::vector_3d, kwiver::vital::rotation_d>
read_camera(const std::string& filename)
{
  if ( !kwiversys::SystemTools::FileExists( filename ) )
  {
    VITAL_THROW( kwiver::vital::path_not_exists, filename);
  }
  std::ifstream file(filename);
  kwiver::vital::vector_3d center;
  kwiver::vital::matrix_3x3d R;
  file >> center[0] >> center[1] >> center[2];
  double v;
  for (int i=0; i < 3; ++i)
  {
    for (int j=0; j < 3; ++j)
    {
      file >> v;
      R(i, j) = v;
    }
  }
  kwiver::vital::rotation_d orientation(R);
  file.close();
  return std::pair<kwiver::vital::vector_3d, kwiver::vital::rotation_d>(center, orientation);
}

void draw_uv_parameterization(const std::vector<kwiver::vital::vector_2d>& tcoords, int w, int h, const std::string &filename)
{
    cv::Mat image(h, w, CV_8UC3, 0.0);
    std::vector<cv::Point2i> points(3);
    kwiver::vital::vector_2d scale(w, h);
    for (int f=0; f < tcoords.size(); f+=3)
    {
        kwiver::vital::vector_2d tcoord_0 = tcoords[f + 0];
        kwiver::vital::vector_2d tcoord_1 = tcoords[f + 1];
        kwiver::vital::vector_2d tcoord_2 = tcoords[f + 2];

        tcoord_0.y() = 1.0 - tcoord_0.y();
        tcoord_1.y() = 1.0 - tcoord_1.y();
        tcoord_2.y() = 1.0 - tcoord_2.y();

        tcoord_0 = tcoord_0.cwiseProduct(scale);
        tcoord_1 = tcoord_1.cwiseProduct(scale);
        tcoord_2 = tcoord_2.cwiseProduct(scale);

        points[0].x = std::round(tcoord_0[0]);
        points[0].y = std::round(tcoord_0[1]);

        points[1].x = std::round(tcoord_1[0]);
        points[1].y = std::round(tcoord_1[1]);

        points[2].x = std::round(tcoord_2[0]);
        points[2].y = std::round(tcoord_2[1]);

        cv::Scalar random_color(rand() % 255, rand() % 255, rand() % 255);
        cv::polylines(image, points, true, random_color);
    }
    cv::imwrite(filename, image);
}



}



using namespace kwiver::vital;
int main2()
{
  kwiver::vital::plugin_manager::instance().load_all_plugins();

  // use comments to execute a particular method

  std::string current_dir = "/media/matthieu/DATA/MAPTk++/test7/";
  std::string krtd_dir = "output/krtd/";
  std::string frames_dir = "frames/";

  std::ifstream frames_file(current_dir + "frames_name_list.txt");


  kwiver::vital::algo::image_io_sptr ocv_io = kwiver::vital::algo::image_io::create("ocv");

  std::vector<image_container_sptr> images;

  std::string line;
  std::vector<camera_perspective_sptr> cameras;
  int first_image_to_use = 0;
  int nb_images_to_use = 40;
  int depth_image_index = first_image_to_use + nb_images_to_use / 2;
  int i=1;
  int sampling_rate = 515;
  while(std::getline(frames_file, line))
  {
    if (/*i==440 || */i == 184 || i == 259 || i == 368 || i == 489) // i % sampling_rate == 0) // i >= first_image_to_use && i < first_image_to_use + nb_images_to_use)

//    if (i==440 || i == 184 || i == 259 || i == 348)

      //    if (i==447 || i == 184 || i == 258 || i == 367) // i % sampling_rate == 0) // i >= first_image_to_use && i < first_image_to_use + nb_images_to_use)
    {
      //60  151 258 367 ###447 istead of 60 because krtd not available for 60 184 instead of 151 for the same reason
      std::string name = kwiversys::SystemTools::GetFilenameWithoutExtension(line);
      std::cout << name << std::endl;
      cameras.push_back(read_krtd_file(current_dir + krtd_dir + name + ".krtd"));
      images.push_back(ocv_io->load(current_dir + frames_dir + name + ".png"));
    }
    i++;
  }

  for (auto& c : cameras)
  {
    std::dynamic_pointer_cast<simple_camera_intrinsics>(c->intrinsics())->set_image_height(1080);
    std::dynamic_pointer_cast<simple_camera_intrinsics>(c->intrinsics())->set_image_width(1920);
  }

  // get vital::image
  std::vector< kwiver::vital::image_of<unsigned char> > frames;
  for (int i=0; i < images.size(); ++i)
  {
    frames.push_back(kwiver::vital::image_of<unsigned char>(images[i]->get_image()));
  }

  auto mesh = read_obj(current_dir + "mesh_simplified_saved_enhanced6.obj");
  std::unique_ptr< kwiver::vital::mesh_regular_face_array<3> > regular_faces(new kwiver::vital::mesh_regular_face_array<3>);
  for (int i = 0; i < mesh->faces().size(); ++i)
  {
    kwiver::vital::mesh_regular_face<3> f;
    f[0] = mesh->faces()(i, 0);
    f[1] = mesh->faces()(i, 1);
    f[2] = mesh->faces()(i, 2);
    regular_faces->push_back(f);
  }
  mesh->set_faces(std::move(regular_faces));




  auto texture = kwiver::arrows::core::generate_texture<unsigned char>(mesh, cameras, frames, 0.003);

  /// Write textured mesh
  ocv_io->save("texture.png", texture);

  /// Write OBJ
  kwiver::vital::write_obj("texture_mesh.obj", *mesh);

  /// Write material file
  std::ofstream mtl_file("./material.mtl");
  mtl_file << "newmtl mat\n";
  mtl_file << "Ka 1.0 1.0 1.0\n";
  mtl_file << "Kd 1.0 1.0 1.0\n";
  mtl_file << "Ks 1 1 1\n";
  mtl_file << "d 1\n";
  mtl_file << "Ns 75\n";
  mtl_file << "illum 1\n";
  mtl_file << "map_Kd texture.png";
  mtl_file.close();
  return 0;
}







int main()
{
//  main2();
//  return 0;

  kwiver::vital::plugin_manager::instance().load_all_plugins();
  // use comments to execute a particular method

  /// Cameras
  kwiver::vital::camera_intrinsics_sptr camera_intrinsic;
//  camera_intrinsic.reset(new kwiver::vital::simple_camera_intrinsics(1024, {480, 270}, 1.0, 0.0, {}, 960, 540));
  camera_intrinsic.reset(new kwiver::vital::simple_camera_intrinsics(4480, {2048, 1536}, 1.0, 0.0, {}, 4096, 3072));
//  camera_intrinsic.reset(new kwiver::vital::simple_camera_intrinsics(2100, {1920/2, 1080/2}, 1.0, 0.0, {}, 1920, 1080));

  std::vector< std::pair<kwiver::vital::vector_3d, kwiver::vital::rotation_d> > param_cams;
//  param_cams.push_back(read_camera("/home/matthieu/data_cube_texture/cameras/cam1.txt"));
//  param_cams.push_back(read_camera("/home/matthieu/data_cube_texture/cameras/cam2.txt"));
//  param_cams.push_back(read_camera("/home/matthieu/data_cube_texture/cameras/cam3.txt"));
//  param_cams.push_back(read_camera("/home/matthieu/data_cube_texture/cameras/cam4.txt"));
//  param_cams.push_back(read_camera("/home/matthieu/data_cube_texture/cameras/cam5.txt"));
//  param_cams.push_back(read_camera("/home/matthieu/data_cube_texture/cameras/cam6.txt"));

  param_cams.push_back(read_camera("/home/matthieu/data_plane/cam1.txt"));
  param_cams.push_back(read_camera("/home/matthieu/data_plane/cam2.txt"));
  param_cams.push_back(read_camera("/home/matthieu/data_plane/cam3.txt"));

//  param_cams.push_back(read_camera("/home/matthieu/data_towers/cam_0.txt"));
//  param_cams.push_back(read_camera("/home/matthieu/data_towers/cam_1.txt"));
//  param_cams.push_back(read_camera("/home/matthieu/data_towers/cam_2.txt"));


  std::vector<kwiver::vital::camera_perspective_sptr> cameras;
  for (unsigned int c = 0; c < param_cams.size(); ++c)
  {
    cameras.push_back(std::make_shared<kwiver::vital::simple_camera_perspective>(param_cams[c].first, param_cams[c].second.inverse(), camera_intrinsic));
  }

  /// Images
  std::vector<kwiver::vital::image_container_sptr> images_temp;
  std::vector< kwiver::vital::image_of<unsigned char> > images;
  kwiver::vital::algo::image_io_sptr image_io = kwiver::vital::algo::image_io::create("ocv");
//  images_temp.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam1.png"));
//  images_temp.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam2.png"));
//  images_temp.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam3.png"));
//  images_temp.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam4.png"));
//  images_temp.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam5.png"));
//  images_temp.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam6.png"));

  images_temp.push_back(image_io->load("/home/matthieu/data_plane/cam1.png"));
  images_temp.push_back(image_io->load("/home/matthieu/data_plane/cam2.png"));
  images_temp.push_back(image_io->load("/home/matthieu/data_plane/cam3.png"));

//  images_temp.push_back(image_io->load("/home/matthieu/data_towers/img_0.png"));
//  images_temp.push_back(image_io->load("/home/matthieu/data_towers/img_1.png"));
//  images_temp.push_back(image_io->load("/home/matthieu/data_towers/img_2.png"));

  for (unsigned int i=0; i < images_temp.size(); ++i)
  {
    images.push_back(kwiver::vital::image_of<unsigned char>(images_temp[i]->get_image()));

  }

  /// Mesh
  kwiver::vital::mesh_sptr mesh;
//  mesh = kwiver::vital::read_obj("/home/matthieu/data_cube_texture/cube.obj");
//  mesh = kwiver::vital::read_obj("/home/matthieu/data_plane/f16_small_3.obj");
  mesh = kwiver::vital::read_obj("/home/matthieu/data_plane/f16.obj");
//  mesh = kwiver::vital::read_obj("/home/matthieu/data_towers/towers.obj");

  /// Turn mesh into reglar faces (should be done also for the occlusion mesh)
  std::unique_ptr< kwiver::vital::mesh_regular_face_array<3> > regular_faces(new kwiver::vital::mesh_regular_face_array<3>);
  for (unsigned int i = 0; i < mesh->faces().size(); ++i)
  {
    kwiver::vital::mesh_regular_face<3> f;
    f[0] = mesh->faces()(i, 0);
    f[1] = mesh->faces()(i, 1);
    f[2] = mesh->faces()(i, 2);
    regular_faces->push_back(f);
  }
  mesh->set_faces(std::move(regular_faces));

  kwiver::vital::image_container_sptr texture;
  PROFILE("texture mapping ",
      //texture = kwiver::arrows::core::generate_texture<unsigned char>(mesh, cameras, images, 0.00088484);
//          texture = kwiver::arrows::core::generate_texture<unsigned char>(mesh, cameras, images, 0.003);
          texture = kwiver::arrows::core::generate_texture<unsigned char>(mesh, cameras, images,  0.0008);
//      texture = kwiver::arrows::core::generate_texture<unsigned char>(mesh, cameras, images,  0.00088484);
      )

  kwiver::vital::algo::image_io_sptr ocv_io = kwiver::vital::algo::image_io::create("ocv");
  ocv_io->save("texture.png", texture);

  kwiver::vital::write_obj("mesh.obj", *mesh);

  draw_uv_parameterization(mesh->tex_coords(), texture->get_image().width(), texture->get_image().height(), "param.png");

  /// Write material file
  std::ofstream mtl_file("./material.mtl");
  mtl_file << "newmtl mat\n";
  mtl_file << "Ka 1.0 1.0 1.0\n";
  mtl_file << "Kd 1.0 1.0 1.0\n";
  mtl_file << "Ks 1 1 1\n";
  mtl_file << "d 1\n";
  mtl_file << "Ns 75\n";
  mtl_file << "illum 1\n";
  mtl_file << "map_Kd texture.png";
  mtl_file.close();

  return 0;
}

