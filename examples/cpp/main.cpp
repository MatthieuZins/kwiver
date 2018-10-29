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
#include <vital/types/image_container.h>
#include <vital/io/camera_from_metadata.h>
#include <vital/io/camera_io.h>
#include <vital/algo/image_io.h>
#include <vital/types/mesh.h>
#include <vital/io/mesh_io.h>
#include <vital/types/vector.h>
#include <vital/plugin_loader/plugin_manager.h>
#include <arrows/core/compute_mesh_depth_map.h>
#include <arrows/ocv/image_container.h>
#include <opencv2/opencv.hpp>

kwiver::vital::camera_rpc_sptr
loadcamera_from_tif_image(const std::string& filename)
{
  kwiver::vital::algo::image_io_sptr image_io = kwiver::vital::algo::image_io::create("gdal");

  kwiver::vital::image_container_sptr img = image_io->load(filename);
  kwiver::vital::metadata_sptr md = img->get_metadata();

  kwiver::vital::camera_sptr camera = kwiver::vital::camera_from_metadata(md);

  kwiver::vital::simple_camera_rpc* cam = dynamic_cast<kwiver::vital::simple_camera_rpc*>(camera.get());
  cam->set_image_height(img->height());
  cam->set_image_width(img->width());

  return std::dynamic_pointer_cast<kwiver::vital::camera_rpc>(camera);
//  return camera;
}

int main()
{
  // use comments to execute a particular method
  kwiver::vital::plugin_manager::instance().load_all_plugins();

//  how_to_part_01_images();
//  how_to_part_02_detections();

//  kwiver::vital::camera_rpc_sptr cam = loadcamera_from_tif_image("/media/matthieu/DATA/core3D-data/AOI2/images/pansharpen/rescaled/07OCT16WV031100016OCT07164906-P1BS-500941044050_01_P001_________AAE_0AAAAABPABJ0_pansharpen_8.tif");
  kwiver::vital::camera_rpc_sptr cam = loadcamera_from_tif_image("/media/matthieu/DATA/core3d_results/20180928/AOI4/images/15FEB15WV031200015FEB15161208-P1BS-500648061070_01_P001_________AAE_0AAAAABPABP0_crop_pansharpened_processed.tif");
//  kwiver::vital::camera_affine_sptr affine_approx = cam->approximate_affine_camera();


//  std::cout << "affine approx \n" << affine_approx->get_matrix() << std::endl;
//  std::cout << "camera center " << affine_approx->get_center() << std::endl;
  //  cam->get_center_and_axis();

  kwiver::vital::vector_3d pt = {-84.083787, 39.780415, 200};
  kwiver::vital::vector_3d pt2 = {-84.086041, 39.782417, 200};
//  std::cout << "pt = " << affine_approx->project(pt) << std::endl;
//  std::cout << "pt rpc = " << cam->project(pt) << std::endl;

//  std::cout << "pt2 = " << affine_approx->project(pt2) << std::endl;
//  std::cout << "pt2 rpc = " << cam->project(pt2) << std::endl;


  kwiver::vital::mesh_sptr mesh;
//  kwiver::vital::vector_3d mesh_offset = {749376.2, 4407080.0, 239.85687};
//  mesh = kwiver::vital::read_obj("/media/matthieu/DATA/core3d_results/20180928/AOI2/meshes/test/mesh.obj");
  kwiver::vital::vector_3d mesh_offset = {435516.726081, 3354093.8, -47.911346};
  mesh = kwiver::vital::read_obj("/media/matthieu/DATA/core3d_results/20180928/AOI4/meshes/concatenated/concatenated.obj");

  kwiver::vital::mesh_vertex_array<3>& vertices = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(mesh->vertices());
  kwiver::vital::vector_3d center(0.0, 0.0, 0.0);
  for (int i=0; i < vertices.size(); ++i)
  {
    vertices[i] += mesh_offset;
    center += vertices[i];
  }
  center /= vertices.size();

  std::pair<kwiver::vital::image_container_sptr,
        kwiver::vital::image_container_sptr> depthmap_pair = kwiver::arrows::compute_mesh_depth_map(mesh, cam);


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

//  kwiver::vital::mesh_vertex_array<3>& vertices = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(mesh->vertices());
//  for (int i=0; i < vertices.size(); ++i)
//  {
//    vertices[i] += mesh_offset;
//    std::cout << vertices[i][0] << " "<< vertices[i][1] << " "<< vertices[i][2] << std::endl;
//    kwiver::vital::vector_2d uv = affine_approx->project(vertices[i]);
//    std::cout << uv(0) << " " << uv(1) << std::endl;
//  }


//  std::cout << "-------------------------\n" << affine_approx->get_matrix() << std::endl;
//  kwiver::vital::simple_camera_affine aff({0, 0, -1}, {0, 1, 0}, mesh_offset, {0, 0}, {4, 4}, 6000, 4000);
//  std::cout << "-------------------------\n" << aff.get_matrix() << std::endl;

}
