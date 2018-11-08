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

#include <vital/types/camera_rpc.h>
#include <vital/types/camera.h>
#include <vital/types/camera_rpc.h>
#include <vital/types/image_container.h>
#include <vital/io/camera_from_metadata.h>
#include <vital/io/camera_io.h>
#include <vital/algo/image_io.h>
#include <vital/types/geodesy.h>
#include <vital/types/mesh.h>
#include <vital/io/mesh_io.h>
#include <vital/types/vector.h>
#include <vital/plugin_loader/plugin_manager.h>
#include <vital/types/vector.h>
//#include <arrows/core/render_mesh_depth_map.h>
#include <arrows/ocv/image_container.h>
#include <opencv2/opencv.hpp>
#include <kwiversys/SystemTools.hxx>
#include <vital/exceptions.h>
#include <memory>
#include <arrows/core/render.h>

//kwiver::vital::camera_rpc_sptr
//loadcamera_from_tif_image(const std::string& filename)
//{
//  kwiver::vital::algo::image_io_sptr image_io = kwiver::vital::algo::image_io::create("gdal");

//  kwiver::vital::image_container_sptr img = image_io->load(filename);
//  kwiver::vital::metadata_sptr md = img->get_metadata();

//  kwiver::vital::camera_sptr camera = kwiver::vital::camera_from_metadata(md);

//  kwiver::vital::simple_camera_rpc* cam = dynamic_cast<kwiver::vital::simple_camera_rpc*>(camera.get());
//  cam->set_image_height(img->height());
//  cam->set_image_width(img->width());

//  return std::dynamic_pointer_cast<kwiver::vital::camera_rpc>(camera);
//}

//std::pair<kwiver::vital::vector_3d, kwiver::vital::rotation_d>
//read_camera(const std::string& filename)
//{
//  if ( !kwiversys::SystemTools::FileExists( filename ) )
//  {
//    VITAL_THROW( kwiver::vital::path_not_exists, filename);
//  }
//  std::ifstream file(filename);
//  kwiver::vital::vector_3d center;
//  kwiver::vital::matrix_3x3d R;
//  file >> center[0] >> center[1] >> center[2];
//  double v;
//  for (int i=0; i < 3; ++i)
//  {
//    for (int j=0; j < 3; ++j)
//    {
//      file >> v;
//      R(i, j) = v;
//    }
//  }
//  kwiver::vital::rotation_d orientation(R);
//  file.close();
//  return std::pair<kwiver::vital::vector_3d, kwiver::vital::rotation_d>(center, orientation);
//}


int main()
{
//  // use comments to execute a particular method
//  kwiver::vital::plugin_manager::instance().load_all_plugins();

////  how_to_part_01_images();
////  how_to_part_02_detections();

//  kwiver::vital::camera_intrinsics_sptr camera_intrinsic;
////  camera_intrinsic.reset(new kwiver::vital::simple_camera_intrinsics(1024, {480, 270}, 1.0, 0.0, {}, 960, 540));
////  camera_intrinsic.reset(new kwiver::vital::simple_camera_intrinsics(4480, {2048, 1536}, 1.0, 0.0, {}, 4096, 3072));
////  camera_intrinsic.reset(new kwiver::vital::simple_camera_intrinsics(2100, {1920/2, 1080/2}, 1.0, 0.0, {}, 1920, 1080));
//  std::pair<kwiver::vital::vector_3d, kwiver::vital::rotation_d> param_cam;
////  param_cam = read_camera("/home/matthieu/data_cube_texture/cameras/cam1.txt");
////  param_cam = read_camera("/home/matthieu/data_plane/cam1.txt");
////  param_cam = read_camera("/home/matthieu/data_towers/cam_2.txt");



//  kwiver::vital::camera_sptr camera;
////  camera.reset(new kwiver::vital::simple_camera_perspective(param_cam.first, param_cam.second.inverse(), camera_intrinsic));
////  camera.reset(loadcamera_from_tif_image("/media/matthieu/DATA/core3D-data/AOI2/images/pansharpen/rescaled/07OCT16WV031100016OCT07164906-P1BS-500941044050_01_P001_________AAE_0AAAAABPABJ0_pansharpen_8.tif"));
//  camera = loadcamera_from_tif_image("/media/matthieu/DATA/core3d_results/20180928/AOI4/images/18FEB16WV031200016FEB18164007-P1BS-501504472090_01_P001_________AAE_0AAAAABPABP0_crop_pansharpened_processed.tif");

//  kwiver::vital::mesh_sptr mesh;
////  mesh= kwiver::vital::read_obj("/home/matthieu/data_cube_texture/test.obj");
////  mesh = kwiver::vital::read_obj("/home/matthieu/data_plane/f16.obj");
////  mesh = kwiver::vital::read_obj("/home/matthieu/data_towers/towers.obj");

//  //  kwiver::vital::vector_3d mesh_offset = {749376.2, 4407080.0, 239.85687};
//  //  mesh = kwiver::vital::read_obj("/media/matthieu/DATA/core3d_results/20180928/AOI2/meshes/test/mesh.obj");

//    kwiver::vital::vector_3d mesh_offset = {435516.726081, 3354093.8, -47.911346};
//    mesh = kwiver::vital::read_obj("/media/matthieu/DATA/core3d_results/20180928/AOI4/meshes/concatenated/concatenated.obj");




//  // Turn the mesh into a triangular mesh
//  std::vector< kwiver::vital::mesh_regular_face<3> > regular_faces(mesh->faces().size());
//  for (unsigned int f = 0; f < mesh->faces().size(); ++f)
//  {
//    std::vector<unsigned int> face(3);
//    for (int i = 0; i < 3; ++i)
//    {
//      face[i] = mesh->faces()(f, i);
//    }
//    regular_faces[f] = kwiver::vital::mesh_regular_face<3>(face);
//  }
//  std::unique_ptr< kwiver::vital::mesh_regular_face_array<3> > new_faces(new kwiver::vital::mesh_regular_face_array<3>(regular_faces));
//  mesh->set_faces(std::move(new_faces));




//  kwiver::vital::camera_rpc_sptr cam_rpc = std::dynamic_pointer_cast<kwiver::vital::camera_rpc>(camera);
//  kwiver::vital::mesh_vertex_array<3>& vertices = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(mesh->vertices());
//  int dest_coord_sys = kwiver::vital::SRID::lat_lon_WGS84;
//  kwiver::vital::utm_ups_zone_t utm_zone = kwiver::vital::utm_ups_zone(cam_rpc->back_project({0.0, 0.0}, 0.0).head(2));
//  int src_coord_sys = (utm_zone.north ? kwiver::vital::SRID::UTM_WGS84_north : kwiver::vital::SRID::UTM_WGS84_south) + utm_zone.number;
//  kwiver::vital::vector_3d center(0.0, 0.0, 0.0);
//  for (int i = 0; i < vertices.size(); ++i)
//  {
//    vertices[i] += mesh_offset;
//    auto res = kwiver::vital::geo_conv(vertices[i].head<2>(), src_coord_sys, dest_coord_sys);
//    vertices[i].head<2>() = res;
//    center += vertices[i];
//  }
//  center /= vertices.size();
////  camera = cam_rpc->approximate_affine_camera(center);


////  kwiver::vital::image_container_sptr depth_map = kwiver::arrows::render_mesh_depth_map(mesh, std::dynamic_pointer_cast<kwiver::vital::camera_perspective>(camera));

//  kwiver::vital::image_container_sptr depth_map = kwiver::arrows::render_mesh_height_map(mesh, std::dynamic_pointer_cast<kwiver::vital::camera_rpc>(camera));






  kwiver::vital::image_of<double> depth(400, 400);
  kwiver::vital::image_of<unsigned char> img(400, 400);
  kwiver::vital::vector_2d v1(0, 0);
  kwiver::vital::vector_2d v2(390, 0);
  kwiver::vital::vector_2d v3(150, 390);
  double d1 = 1;
  double d2 = 1;
  double d3 = 1;
  unsigned char i1 = 1;
  unsigned char i2 = 2;
  unsigned char i3 = 5;

  kwiver::arrows::render_triangle<unsigned char>(v1, v2, v3, d1, d2, d3, i1, i2, i3,  depth, img, false);

  kwiver::vital::simple_image_container image_temp(img);

  // write depthmap
  cv::Mat image = kwiver::arrows::ocv::image_container_to_ocv_matrix(image_temp,  kwiver::arrows::ocv::image_container::OTHER_COLOR);
  std::cout << image.rows << " " << image.cols << std::endl;
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
}
