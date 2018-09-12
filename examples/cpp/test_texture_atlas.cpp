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

#include <sstream>
#include <iomanip>

#include <vital/types/image_container.h>

#include <vital/plugin_loader/plugin_manager.h>

#include <opencv2/opencv_modules.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <arrows/core/mesh_uv_parameterization.h>
#include <vital/algo/mesh_io.h>
#include <arrows/ocv/image_container.h>

#include <vital/types/camera_perspective.h>
#include <vital/algo/image_io.h>

#include <vital/types/geodesy.h>
#include <vital/types/camera_rpc.h>

#include <vital/types/image_container.h>
#include <vital/algo/image_io.h>

#include <arrows/core/atlas_processing.h>
#include <arrows/core/compute_mesh_cameras_ratings.h>
#include <fstream>
#include <vital/util/cpu_timer.h>
#include <vital/algo/compute_mesh_depthmap.h>

#include <kwiversys/SystemTools.hxx>
#include <vital/exceptions/io.h>


kwiver::vital::camera_rpc_sptr
loadcamera_from_tif_image(const std::string& filename, unsigned int utm_zone)
{
  kwiver::vital::algo::image_io_sptr image_io = kwiver::vital::algo::image_io::create("gdal");

  kwiver::vital::image_container_sptr img = image_io->load(filename);
  kwiver::vital::metadata_sptr md = img->get_metadata();

  kwiver::vital::camera_rpc_sptr cam(new kwiver::vital::simple_camera_rpc(utm_zone));
  kwiver::vital::simple_camera_rpc* cam_pt = dynamic_cast<kwiver::vital::simple_camera_rpc*>(cam.get());
  kwiver::vital::vector_3d world_scale;
  kwiver::vital::vector_3d world_offset;
  kwiver::vital::vector_2d image_scale;
  kwiver::vital::vector_2d image_offset;

  world_scale[0] = md->find(kwiver::vital::VITAL_META_RPC_LONG_SCALE).as_double();
  world_scale[1] = md->find(kwiver::vital::VITAL_META_RPC_LAT_SCALE).as_double();
  world_scale[2] = md->find(kwiver::vital::VITAL_META_RPC_HEIGHT_SCALE).as_double();

  world_offset[0] = md->find(kwiver::vital::VITAL_META_RPC_LONG_OFFSET).as_double();
  world_offset[1] = md->find(kwiver::vital::VITAL_META_RPC_LAT_OFFSET).as_double();
  world_offset[2] = md->find(kwiver::vital::VITAL_META_RPC_HEIGHT_OFFSET).as_double();

  image_scale[0] = md->find(kwiver::vital::VITAL_META_RPC_COL_SCALE).as_double();
  image_scale[1] = md->find(kwiver::vital::VITAL_META_RPC_ROW_SCALE).as_double();

  image_offset[0] = md->find(kwiver::vital::VITAL_META_RPC_COL_OFFSET).as_double();
  image_offset[1] = md->find(kwiver::vital::VITAL_META_RPC_ROW_OFFSET).as_double();

  kwiver::vital::rpc_matrix rpc_coeffs;
  std::stringstream ss;
  ss << md->find(kwiver::vital::VITAL_META_RPC_COL_NUM_COEFF).as_string();
  for (int i=0; i < 20; ++i)
  {
    ss >> rpc_coeffs(0, i);
  }
  ss.clear();
  ss << md->find(kwiver::vital::VITAL_META_RPC_COL_DEN_COEFF).as_string();
  for (int i=0; i < 20; ++i)
  {
    ss >> rpc_coeffs(1, i);
  }
  ss.clear();
  ss << md->find(kwiver::vital::VITAL_META_RPC_ROW_NUM_COEFF).as_string();
  for (int i=0; i < 20; ++i)
  {
    ss >> rpc_coeffs(2, i);
  }
  ss.clear();
  ss << md->find(kwiver::vital::VITAL_META_RPC_ROW_DEN_COEFF).as_string();
  for (int i=0; i < 20; ++i)
  {
    ss >> rpc_coeffs(3, i);
  }

  cam_pt->set_world_scale(world_scale);
  cam_pt->set_world_offset(world_offset);
  cam_pt->set_image_scale(image_scale);
  cam_pt->set_image_offset(image_offset);
  cam_pt->set_rpc_coeffs(rpc_coeffs);

  return cam;
}

std::pair<kwiver::vital::vector_3d, kwiver::vital::rotation_d>
read_camera(const std::string& filename)
{
  if ( !kwiversys::SystemTools::FileExists( filename ) )
  {
    VITAL_THROW( path_not_exists, filename);
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

void test_compute_mesh_depthmap()
{
  kwiver::vital::plugin_manager::instance().load_all_plugins();

  kwiver::vital::algo::compute_mesh_depthmap_sptr depthmap_generator =
      kwiver::vital::algo::compute_mesh_depthmap::create("core");

  kwiver::vital::algo::mesh_io_sptr mesh_io =
      kwiver::vital::algo::mesh_io::create("core");
  //    kwiver::vital::mesh_sptr mesh =  mesh_io->load("/home/matthieu/data_cube_texture/cube.obj");
  //    kwiver::vital::mesh_sptr mesh =  mesh_io->load("/home/matthieu/data_plane/f16.obj");
  kwiver::vital::mesh_sptr mesh =  mesh_io->load("/home/matthieu/data_towers/towers.obj");


  //    kwiver::vital::camera_intrinsics_sptr camera_intrinsic(
  //                new kwiver::vital::simple_camera_intrinsics(1024, {480, 270}));
  //    kwiver::vital::camera_intrinsics_sptr camera_intrinsic(
  //                new kwiver::vital::simple_camera_intrinsics(4480, {2048, 1536}));
  kwiver::vital::camera_intrinsics_sptr camera_intrinsic(
        new kwiver::vital::simple_camera_intrinsics(2100, {1920/2, 1080/2}));
  //    auto param_cam = read_camera("/home/matthieu/data_cube_texture/cameras/cam1.txt");
  //    auto param_cam = read_camera("/home/matthieu/data_plane/cam1.txt");
  auto param_cam = read_camera("/home/matthieu/data_towers/cam_2.txt");

  kwiver::vital::camera_sptr camera(new kwiver::vital::simple_camera_perspective(param_cam.first,
                                                                                 param_cam.second.inverse(),
                                                                                 camera_intrinsic));
  std::cout << dynamic_cast<kwiver::vital::simple_camera_perspective*>(camera.get())->center() << std::endl;
  std::cout << dynamic_cast<kwiver::vital::simple_camera_perspective*>(camera.get())->get_rotation() << std::endl;

  //    std::pair<kwiver::vital::image_container_sptr,
  //              kwiver::vital::image_container_sptr> depthmap_pair = depthmap_generator->compute(mesh, camera, 960, 540, 0);
  //    std::pair<kwiver::vital::image_container_sptr,
  //              kwiver::vital::image_container_sptr> depthmap_pair = depthmap_generator->compute(mesh, camera, 4096, 3072, 0);
  std::pair<kwiver::vital::image_container_sptr,
      kwiver::vital::image_container_sptr> depthmap_pair = depthmap_generator->compute(mesh, camera, 1920, 1080, 0);


  // write depthmap
  cv::Mat image = kwiver::arrows::ocv::image_container_to_ocv_matrix(*depthmap_pair.first,  kwiver::arrows::ocv::image_container::OTHER_COLOR);
  cv::Mat mask;
  double min, max;
  // create an object/background mask
  cv::threshold(image, mask, std::numeric_limits<double>::max() / 2, 1, cv::THRESH_BINARY_INV);
  mask.convertTo(mask, CV_8U);
  cv::minMaxLoc(image, &min, &max, 0, 0, mask);
  min -=1;    // avoid too short range
  max += 1;
  // rescale range to 0-255
  image -= min;
  image /= (max-min);
  image *= 255;
  // threshold max to 255
  cv::threshold(image, image, 255, 0, cv::THRESH_TRUNC);
  // threshold min to 0
  cv::threshold(image, image, 0, 0, cv::THRESH_TOZERO);
  image.convertTo(image, CV_8U);
  cv::imwrite("depthmap.png", image);

  // write depth id map
  cv::Mat id_image = kwiver::arrows::ocv::image_container_to_ocv_matrix(*depthmap_pair.second, kwiver::arrows::ocv::image_container::OTHER_COLOR);
  id_image.convertTo(id_image, CV_32F);
  cv::threshold(id_image, id_image, 600, 0, cv::THRESH_TRUNC);
  cv::normalize(id_image, id_image, 0, 255, cv::NORM_MINMAX);
  id_image.convertTo(id_image, CV_8U);
  cv::imwrite("id_depthmap.png", id_image);
}


void test_fuse_multi_pinhole_cameras()
{
  kwiver::vital::plugin_manager::instance().load_all_plugins();

  std::string mesh_filename = ("/home/matthieu/data_cube_texture/cube.obj");
  //    std::string mesh_filename = ("/home/matthieu/data_plane/f16.obj");
  //    std::string mesh_filename("/home/matthieu/data_towers/towers.obj");

  // mesh
  kwiver::vital::algo::mesh_io_sptr mesh_io = kwiver::vital::algo::mesh_io::create("core");
  kwiver::vital::mesh_sptr mesh = mesh_io->load(mesh_filename);

  // uv parameterization
  std::pair<unsigned int, unsigned int> atlas_dim = kwiver::arrows::core::parameterize(mesh, 0.03, 8000, 10, 5);
  //    std::pair<unsigned int, unsigned int> atlas_dim = kwiver::arrows::core::parameterize(mesh, 0.003, 8000, 10, 5);
  //    std::pair<unsigned int, unsigned int> atlas_dim = kwiver::arrows::core::parameterize(mesh, 0.03, 8000, 10, 5);
  // id map
  kwiver::vital::image_container_sptr id_map = kwiver::arrows::core::generate_triangles_map(mesh, atlas_dim.first, atlas_dim.second);

  // perspective cameras
  kwiver::vital::camera_intrinsics_sptr camera_intrinsic(new kwiver::vital::simple_camera_intrinsics(1024, {480, 270}));
  //    kwiver::vital::camera_intrinsics_sptr camera_intrinsic(new kwiver::vital::simple_camera_intrinsics(4480, {2048, 1536}));
  //    kwiver::vital::camera_intrinsics_sptr camera_intrinsic(new kwiver::vital::simple_camera_intrinsics(2100, {1920/2, 1080/2}));

  kwiver::vital::camera_sptr_list cameras;
  auto param_cam = read_camera("/home/matthieu/data_cube_texture/cameras/cam1.txt");
  cameras.push_back(kwiver::vital::camera_sptr(new kwiver::vital::simple_camera_perspective(param_cam.first, param_cam.second.inverse(), camera_intrinsic)));
  param_cam = read_camera("/home/matthieu/data_cube_texture/cameras/cam2.txt");
  cameras.push_back(kwiver::vital::camera_sptr(new kwiver::vital::simple_camera_perspective(param_cam.first, param_cam.second.inverse(), camera_intrinsic)));
  param_cam = read_camera("/home/matthieu/data_cube_texture/cameras/cam3.txt");
  cameras.push_back(kwiver::vital::camera_sptr(new kwiver::vital::simple_camera_perspective(param_cam.first, param_cam.second.inverse(), camera_intrinsic)));
  param_cam = read_camera("/home/matthieu/data_cube_texture/cameras/cam4.txt");
  cameras.push_back(kwiver::vital::camera_sptr(new kwiver::vital::simple_camera_perspective(param_cam.first, param_cam.second.inverse(), camera_intrinsic)));
  param_cam = read_camera("/home/matthieu/data_cube_texture/cameras/cam5.txt");
  cameras.push_back(kwiver::vital::camera_sptr(new kwiver::vital::simple_camera_perspective(param_cam.first, param_cam.second.inverse(), camera_intrinsic)));
  param_cam = read_camera("/home/matthieu/data_cube_texture/cameras/cam6.txt");
  cameras.push_back(kwiver::vital::camera_sptr(new kwiver::vital::simple_camera_perspective(param_cam.first, param_cam.second.inverse(), camera_intrinsic)));

  //    auto param_cam = read_camera("/home/matthieu/data_plane/cam1.txt");
  //    cameras.push_back(kwiver::vital::camera_sptr(new kwiver::vital::simple_camera_perspective(param_cam.first, param_cam.second.inverse(), camera_intrinsic)));
  //    param_cam = read_camera("/home/matthieu/data_plane/cam2.txt");
  //    cameras.push_back(kwiver::vital::camera_sptr(new kwiver::vital::simple_camera_perspective(param_cam.first, param_cam.second.inverse(), camera_intrinsic)));
  //    param_cam = read_camera("/home/matthieu/data_plane/cam3.txt");
  //    cameras.push_back(kwiver::vital::camera_sptr(new kwiver::vital::simple_camera_perspective(param_cam.first, param_cam.second.inverse(), camera_intrinsic)));

  //    auto param_cam = read_camera("/home/matthieu/data_towers/cam_0.txt");
  //    cameras.push_back(kwiver::vital::camera_sptr(new kwiver::vital::simple_camera_perspective(param_cam.first, param_cam.second.inverse(), camera_intrinsic)));
  //    param_cam = read_camera("/home/matthieu/data_towers/cam_1.txt");
  //    cameras.push_back(kwiver::vital::camera_sptr(new kwiver::vital::simple_camera_perspective(param_cam.first, param_cam.second.inverse(), camera_intrinsic)));
  //    param_cam = read_camera("/home/matthieu/data_towers/cam_2.txt");
  //    cameras.push_back(kwiver::vital::camera_sptr(new kwiver::vital::simple_camera_perspective(param_cam.first, param_cam.second.inverse(), camera_intrinsic)));


  // images
  kwiver::vital::image_container_sptr_list images;
  kwiver::vital::algo::image_io_sptr image_io = kwiver::vital::algo::image_io::create("ocv");
  images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam1.png"));
  images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam2.png"));
  images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam3.png"));
  images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam4.png"));
  images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam5.png"));
  images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam6.png"));

  //    images.push_back(image_io->load("/home/matthieu/data_plane/cam1.png"));
  //    images.push_back(image_io->load("/home/matthieu/data_plane/cam2.png"));
  //    images.push_back(image_io->load("/home/matthieu/data_plane/cam3.png"));

  //    images.push_back(image_io->load("/home/matthieu/data_towers/img_0.png"));
  //    images.push_back(image_io->load("/home/matthieu/data_towers/img_1.png"));
  //    images.push_back(image_io->load("/home/matthieu/data_towers/img_2.png"));


  // depthmaps
  kwiver::vital::algo::compute_mesh_depthmap_sptr depthmap_generator =
      kwiver::vital::algo::compute_mesh_depthmap::create("core");
  image_container_sptr_list depthmaps;
  int i=0;
  for (auto cam: cameras)
  {
    auto res = depthmap_generator->compute(mesh, cam, images[i]->width(), images[i]->height(), 0);
    depthmaps.push_back(res.first);
    i++;
  }

  // ratings
  std::vector< std::vector<float> > ratings;
  kwiver::arrows::core::compute_mesh_cameras_ratings(mesh, cameras, ratings);


  // rasterizations
  kwiver::vital::image_container_sptr_list textures, visibilities, scores;
  for (int i=0; i < cameras.size(); ++i)
  {
    auto raster_result = kwiver::arrows::core::rasterize_texture_atlas<unsigned char>(mesh, id_map, images[i], cameras[i], depthmaps[i], ratings[i]);
    auto texture = std::get<0>(raster_result);
    textures.push_back(std::get<0>(raster_result));
    visibilities.push_back(std::get<1>(raster_result));
    scores.push_back(std::get<2>(raster_result));

    cv::Mat cv_tex = kwiver::arrows::ocv::image_container_to_ocv_matrix(*(texture.get()),
                                                                        kwiver::arrows::ocv::image_container::RGB_COLOR);
    cv_tex.convertTo(cv_tex, CV_8UC3);
    cv::cvtColor(cv_tex, cv_tex, CV_BGR2RGB);
    std::string output_name = "mesh_texture_" + std::to_string(i) + ".obj";
    cv::imwrite(output_name+".png", cv_tex);
    mesh_io->save(output_name, mesh, texture->width(), texture->height(), true);
  }

  // fusion
  auto fused = kwiver::arrows::core::fuse_texture_atlases<unsigned char>(textures, visibilities, scores, 0);
  cv::Mat cv_fused = kwiver::arrows::ocv::image_container_to_ocv_matrix(*(fused.get()), kwiver::arrows::ocv::image_container::RGB_COLOR);
  cv_fused.convertTo(cv_fused, CV_8UC3);
  cv::cvtColor(cv_fused, cv_fused, CV_BGR2RGB);
  std::string output_name = "mesh_texture_fused.obj";
  cv::imwrite(output_name + ".png", cv_fused);
  mesh_io->save(output_name, mesh, fused->width(), fused->height(), true);
}

void test_fuse_multi_rpc_cameras()
{
  kwiver::vital::plugin_manager::instance().load_all_plugins();

  // mesh
  //    std::string mesh_filename = ("/media/matthieu/DATA/core3D-data/test_aoi4.obj");
  //    std::string mesh_filename = ("/media/matthieu/DATA/core3D-data/AOI_D4_(Nick)/result_new/buildings.obj");
  std::string mesh_filename = ("/home/matthieu/Downloads/JIDO-Jacksonville-Model/Jacksonville_OBJ_Buildings_Only_offset_cropped2.obj");
  kwiver::vital::algo::mesh_io_sptr mesh_io = kwiver::vital::algo::mesh_io::create("core");
  kwiver::vital::mesh_sptr mesh = mesh_io->load(mesh_filename);
  kwiver::vital::mesh_vertex_array<3>& vertices = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(mesh->vertices());
  kwiver::vital::vector_3d mesh_offset = {435530.547508, 3354095.61004, -36.844062};
  for (int i=0; i < vertices.size(); ++i)
  {
    vertices[i] += mesh_offset;
  }

  // uv parameterization
  std::pair<unsigned int, unsigned int> atlas_dim = kwiver::arrows::core::parameterize(mesh, 0.3, 8000, 10, 5);
  // id map
  kwiver::vital::image_container_sptr id_map = kwiver::arrows::core::generate_triangles_map(mesh, atlas_dim.first, atlas_dim.second);

  std::vector<std::string> images_filenames;
  //    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/27JAN15WV031100015JAN27160845-P1BS-500648062010_01_P001_________AAE_0AAAAABPABS0_pansharpen_8.tif");
  //    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/26APR15WV031200015APR26162435-P1BS-501504472050_01_P001_________AAE_0AAAAABPABR0_pansharpen_8.tif");
  //    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/15FEB15WV031200015FEB15161208-P1BS-500648061070_01_P001_________AAE_0AAAAABPABP0_pansharpen_8.tif");
  //    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/11OCT14WV031100014OCT11155720-P1BS-500648061020_01_P001_________AAE_0AAAAABPABR0_pansharpen_8.tif");
  //    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/08FEB15WV031100015FEB08160130-P1BS-500648061090_01_P001_________AAE_0AAAAABPABP0_pansharpen_8.tif");
  //    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/02MAY15WV031100015MAY02161943-P1BS-500648061030_01_P001_________AAE_0AAAAABPABR0_pansharpen_8.tif");
  //    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/01MAY15WV031200015MAY01160357-P1BS-500648062030_01_P001_________AAE_0AAAAABPABQ0_pansharpen_8.tif");

  images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/15FEB15WV031200015FEB15161208-P1BS-500648061070_01_P001_________AAE_0AAAAABPABP0_pansharpen_8.tif");
  images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/05JUL15WV031100015JUL05162954-P1BS-500648062020_01_P001_________AAE_0AAAAABPABQ0_pansharpen_8.tif");
  images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/01NOV15WV031100015NOV01162034-P1BS-500648061060_01_P001_________AAE_0AAAAABPABE0_pansharpen_8.tif");
  images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/01NOV15WV031100015NOV01161954-P1BS-500648062080_01_P001_________AAE_0AAAAABPABS0_pansharpen_8.tif");
  images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/01MAY15WV031200015MAY01160357-P1BS-500648062030_01_P001_________AAE_0AAAAABPABQ0_pansharpen_8.tif");

  // camera
  kwiver::vital::camera_sptr_list cameras;
  for (auto s: images_filenames)
  {
    cameras.push_back(loadcamera_from_tif_image(s, 17));
  }

  // image
  kwiver::vital::image_container_sptr_list images;
  kwiver::vital::algo::image_io_sptr image_io = kwiver::vital::algo::image_io::create("gdal");
  for (auto s: images_filenames)
  {
    images.push_back(image_io->load(s));
  }


  // depthmaps
  kwiver::vital::algo::compute_mesh_depthmap_sptr depthmap_generator =
      kwiver::vital::algo::compute_mesh_depthmap::create("core");
  image_container_sptr_list depthmaps;
  int i=0;
  for (auto cam: cameras)
  {
    auto res = depthmap_generator->compute(mesh, cam, images[i]->width(), images[i]->height(), 17);
    depthmaps.push_back(res.first);
    i++;
  }

  std::vector< std::vector<float> > ratings;
  kwiver::arrows::core::compute_mesh_cameras_ratings(mesh, cameras, ratings);
  //    for (int f=0; f < mesh->num_faces(); ++f)
  //    {
  //        std::cout << "face " << f << " : ";
  //        for (int c=0; c < cameras.size(); ++c)
  //        {
  //            std::cout << ratings[c][f] << "  ";
  //        }
  //        std::cout << std::endl;
  //    }

  // rasterizations
  kwiver::vital::image_container_sptr_list textures, visibilities, scores;
  for (int i=0; i < cameras.size(); ++i)
  {
    kwiver::vital::cpu_timer timer;
    timer.start();
    auto raster_result = kwiver::arrows::core::rasterize_texture_atlas<unsigned short>(mesh, id_map, images[i], cameras[i], depthmaps[i], ratings[i]);
    timer.stop();
    std::cout << "rasterization timer => " << timer.elapsed() << " s" << std::endl;
    auto texture = std::get<0>(raster_result);
    textures.push_back(std::get<0>(raster_result));
    visibilities.push_back(std::get<1>(raster_result));
    scores.push_back(std::get<2>(raster_result));

    cv::Mat cv_tex = kwiver::arrows::ocv::image_container_to_ocv_matrix(*(texture.get()),
                                                                        kwiver::arrows::ocv::image_container::RGB_COLOR);
    cv_tex.convertTo(cv_tex, CV_8UC3);
    std::vector<cv::Mat> splitted;
    cv::split(cv_tex, splitted);
    std::vector<cv::Mat> rgb_splitted = {splitted[1], splitted[2], splitted[4]};
    //        cv::cvtColor(cv_fused, cv_fused, CV_BGR2RGB);
    cv::merge(rgb_splitted, cv_tex);
    std::string output_name = "texture_" + std::to_string(i) + ".obj";
    cv::imwrite(output_name+".png", cv_tex);
    mesh_io->save(output_name, mesh, texture->width(), texture->height(), true);
  }

  // remove offset
  for (int i=0; i < vertices.size(); ++i)
  {
    vertices[i] -= mesh_offset;
  }

  // fusion

  kwiver::vital::cpu_timer timer;
  timer.start();
  auto fused = kwiver::arrows::core::fuse_texture_atlases<unsigned short>(textures, visibilities, scores, 0);
  timer.stop();
  std::cout << "timer : " << timer.elapsed() << " s" << std::endl;
  cv::Mat cv_fused = kwiver::arrows::ocv::image_container_to_ocv_matrix(*(fused.get()), kwiver::arrows::ocv::image_container::RGB_COLOR);
  cv_fused.convertTo(cv_fused, CV_8UC3);
  std::vector<cv::Mat> splitted;
  cv::split(cv_fused, splitted);
  std::vector<cv::Mat> rgb_splitted = {splitted[1], splitted[2], splitted[4]};
  //        cv::cvtColor(cv_fused, cv_fused, CV_BGR2RGB);
  cv::merge(rgb_splitted, cv_fused);    std::string output_name = "texture_fused.obj";
  cv::imwrite(output_name + ".png", cv_fused);
  mesh_io->save(output_name, mesh, fused->width(), fused->height(), true);

}
