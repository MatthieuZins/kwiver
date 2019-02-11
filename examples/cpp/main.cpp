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


#include "vital/types/image.h"
#include "vital/types/image_container.h"

#include "vital/algo/image_io.h"
#include "vital/algo/image_filter.h"

#include "vital/plugin_loader/plugin_manager.h"

#include <kwiversys/SystemTools.hxx>
#include <vital/types/camera_map.h>
#include "vital/types/camera_perspective.h"
#include <vital/io/camera_io.h>
#include <vital/io/landmark_map_io.h>
#include <vital/types/image_container.h>
#include <vital/algo/image_io.h>
#include <kwiversys/SystemTools.hxx>
#include <fstream>
#include <vector>
#include <string>

#include <vital/algo/compute_depth.h>
#include <vital/io/mesh_io.h>

#include <arrows/core/generate_texture.h>
// Predefine methods that show off various functionality in kwiver
void how_to_part_01_images();
void how_to_part_02_detections();

using namespace kwiver::vital;
int main()
{
  kwiver::vital::plugin_manager::instance().load_all_plugins();

  // use comments to execute a particular method

//  std::string current_dir = "/media/matthieu/DATA/Head_Reconstruction/";
//  std::string current_dir = "/media/matthieu/DATA/MAPTk++/test7/";
  std::string current_dir = "/home/matthieu/Dev/lidar-to-mesh/build/";


//  std::string frames_dir = "frames/";
//  std::string krtd_dir = "/home/matthieu/Dev/lidar-to-mesh/src/krtd/";
  std::string krtd_dir = "/home/matthieu/Dev/lidar-to-mesh/build/output/krtd/";

  std::string frames_dir = "/home/matthieu/Dev/lidar-to-mesh/build/output/frames/";

  std::ifstream frames_file(current_dir + "frames_name_list.txt");


  kwiver::vital::algo::image_io_sptr ocv_io = kwiver::vital::algo::image_io::create("ocv");

  std::vector<image_container_sptr> images;

  std::string line;
  std::vector<camera_perspective_sptr> cameras;
  int first_image_to_use = 0;
  int nb_images_to_use = 40;
  int depth_image_index = first_image_to_use + nb_images_to_use / 2;
  int i=0;
  int sampling_rate = 515;
  while(std::getline(frames_file, line))
  {
//    if (i==0 || i == 39|| i==440 || i == 184 || i == 259 || i == 348) // i % sampling_rate == 0) // i >= first_image_to_use && i < first_image_to_use + nb_images_to_use)
    if (i < 40)
    {
      std::string name = kwiversys::SystemTools::GetFilenameWithoutExtension(line);
      std::cout << name << std::endl;
      cameras.push_back(read_krtd_file(krtd_dir + name + ".krtd"));
      images.push_back(ocv_io->load(frames_dir + name + ".png"));
      std::cout << "camera " << i << " -----------------------------\n";
    }
    i++;
  }

  for (auto& c : cameras)
  {
//    std::dynamic_pointer_cast<simple_camera_intrinsics>(c->intrinsics())->set_image_height(1080);
//    std::dynamic_pointer_cast<simple_camera_intrinsics>(c->intrinsics())->set_image_width(1920);
//    std::dynamic_pointer_cast<simple_camera_intrinsics>(c->intrinsics())->set_image_height(951);
//    std::dynamic_pointer_cast<simple_camera_intrinsics>(c->intrinsics())->set_image_width(891);

    // KITTI
    std::dynamic_pointer_cast<simple_camera_intrinsics>(c->intrinsics())->set_image_height(375);
    std::dynamic_pointer_cast<simple_camera_intrinsics>(c->intrinsics())->set_image_width(1242);

//      std::dynamic_pointer_cast<simple_camera_intrinsics>(c->intrinsics())->set_image_height(950);
//      std::dynamic_pointer_cast<simple_camera_intrinsics>(c->intrinsics())->set_image_width(2460);

  }


  // get vital::image
  std::vector< kwiver::vital::image_of<unsigned char> > frames;
  for (int i=0; i < images.size(); ++i)
  {
    frames.push_back(kwiver::vital::image_of<unsigned char>(images[i]->get_image()));
  }

//  auto mesh = read_obj(current_dir + "mesh_smooth2.obj");
  auto mesh = read_obj(current_dir + "decimated_mesh_ready_for_texture_5.obj");
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




  auto texture = kwiver::arrows::core::generate_texture<unsigned char>(mesh, cameras, frames, 0.006);

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
}
