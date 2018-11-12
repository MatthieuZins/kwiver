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
#include <vital/types/image_container.h>
#include <fstream>
#include <arrows/core/render_mesh_depth_map.h>

#define PROFILE(str, ...)      \
{                              \
    std::cout << str << " ... \t\t" << std::flush;    \
    clock_t profile_t0 = clock();   \
    __VA_ARGS__;                    \
    std::cout << double(clock() - profile_t0) / CLOCKS_PER_SEC   \
    << " s" << std::endl;           \
}

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
}

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
    std::cout << "start uv drawing" << std::endl;
    for (int f=0; f < tcoords.size(); f+=3)
    {
        std::cout << "face " << f << std::endl;
        kwiver::vital::vector_2d tcoord_0 = tcoords[f + 0];
        kwiver::vital::vector_2d tcoord_1 = tcoords[f + 1];
        kwiver::vital::vector_2d tcoord_2 = tcoords[f + 2];

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

    std::cout << "::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::" << std::endl;
    std::cout << image.size() << std::endl;
    cv::imwrite(filename, image);
    std::cout << "::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::" << std::endl;

}


#include <vital/types/mesh.h>
#include <vital/io/mesh_io.h>
#include <vital/algo/compute_mesh_uv_parameterization.h>
#include <vital/plugin_loader/plugin_manager.h>
#include <iostream>

int main()
{
  //  // use comments to execute a particular method
  kwiver::vital::plugin_manager::instance().load_all_plugins();

  ////  how_to_part_01_images();
  ////  how_to_part_02_detections();

  kwiver::vital::camera_intrinsics_sptr camera_intrinsic;
//    camera_intrinsic.reset(new kwiver::vital::simple_camera_intrinsics(1024, {480, 270}, 1.0, 0.0, {}, 960, 540));
//  camera_intrinsic.reset(new kwiver::vital::simple_camera_intrinsics(4480, {2048, 1536}, 1.0, 0.0, {}, 4096, 3072));
//    camera_intrinsic.reset(new kwiver::vital::simple_camera_intrinsics(2100, {1920/2, 1080/2}, 1.0, 0.0, {}, 1920, 1080));

  std::vector< std::pair<kwiver::vital::vector_3d, kwiver::vital::rotation_d> > param_cams;
//  param_cams.push_back(read_camera("/home/matthieu/data_cube_texture/cameras/cam1.txt"));
//  param_cams.push_back(read_camera("/home/matthieu/data_cube_texture/cameras/cam2.txt"));
//  param_cams.push_back(read_camera("/home/matthieu/data_cube_texture/cameras/cam3.txt"));
//  param_cams.push_back(read_camera("/home/matthieu/data_cube_texture/cameras/cam4.txt"));
//  param_cams.push_back(read_camera("/home/matthieu/data_cube_texture/cameras/cam5.txt"));
//  param_cams.push_back(read_camera("/home/matthieu/data_cube_texture/cameras/cam6.txt"));

//  param_cams.push_back(read_camera("/home/matthieu/data_plane/cam1.txt"));
//  param_cams.push_back(read_camera("/home/matthieu/data_plane/cam2.txt"));
//  param_cams.push_back(read_camera("/home/matthieu/data_plane/cam3.txt"));

//  param_cams.push_back(read_camera("/home/matthieu/data_towers/cam_0.txt"));
//  param_cams.push_back(read_camera("/home/matthieu/data_towers/cam_1.txt"));
//  param_cams.push_back(read_camera("/home/matthieu/data_towers/cam_2.txt"));



//  std::vector<kwiver::vital::camera_sptr> cameras;
  for (int c = 0; c < param_cams.size(); ++c)
  {
//    cameras.push_back(std::make_shared<kwiver::vital::simple_camera_perspective>(param_cams[c].first, param_cams[c].second.inverse(), camera_intrinsic));
//      cameras.push_back(loadcamera_from_tif_image("/media/matthieu/DATA/core3D-data/AOI2/images/pansharpen/rescaled/07OCT16WV031100016OCT07164906-P1BS-500941044050_01_P001_________AAE_0AAAAABPABJ0_pansharpen_8.tif"));
//      cameras.push_back(loadcamera_from_tif_image("/media/matthieu/DATA/core3d_results/20180928/AOI4/images/18FEB16WV031200016FEB18164007-P1BS-501504472090_01_P001_________AAE_0AAAAABPABP0_crop_pansharpened_processed.tif"));
  }

  kwiver::vital::mesh_sptr mesh;
//    mesh= kwiver::vital::read_obj("/home/matthieu/data_cube_texture/cube.obj");
//    mesh = kwiver::vital::read_obj("/home/matthieu/data_plane/f16.obj");
//    mesh = kwiver::vital::read_obj("/home/matthieu/data_towers/towers.obj");

  //  kwiver::vital::vector_3d mesh_offset = {749376.2, 4407080.0, 239.85687};
  //  mesh = kwiver::vital::read_obj("/media/matthieu/DATA/core3d_results/20180928/AOI2/meshes/test/mesh.obj");

    kwiver::vital::vector_3d mesh_offset = {435516.726081, 3354093.8, -47.911346};
    mesh = kwiver::vital::read_obj("/media/matthieu/DATA/core3d_results/20180928/AOI4/meshes/triangulated/ground.obj");
    kwiver::vital::mesh_sptr occlusion_mesh = kwiver::vital::read_obj("/media/matthieu/DATA/core3d_results/20180928/AOI4/meshes/concatenated/concatenated.obj");


//  kwiver::vital::image_container_sptr_list images;
//  kwiver::vital::algo::image_io_sptr image_io = kwiver::vital::algo::image_io::create("ocv");
//  images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam1.png"));
//  images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam2.png"));
//  images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam3.png"));
//  images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam4.png"));
//  images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam5.png"));
//  images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam6.png"));

//  images.push_back(image_io->load("/home/matthieu/data_plane/cam1.png"));
//  images.push_back(image_io->load("/home/matthieu/data_plane/cam2.png"));
//  images.push_back(image_io->load("/home/matthieu/data_plane/cam3.png"));

//    images.push_back(image_io->load("/home/matthieu/data_towers/img_0.png"));
//    images.push_back(image_io->load("/home/matthieu/data_towers/img_1.png"));
//    images.push_back(image_io->load("/home/matthieu/data_towers/img_2.png"));




  std::vector<std::string> images_filenames;
  //    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/27JAN15WV031100015JAN27160845-P1BS-500648062010_01_P001_________AAE_0AAAAABPABS0_pansharpen_8.tif");
  //    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/26APR15WV031200015APR26162435-P1BS-501504472050_01_P001_________AAE_0AAAAABPABR0_pansharpen_8.tif");
  //    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/15FEB15WV031200015FEB15161208-P1BS-500648061070_01_P001_________AAE_0AAAAABPABP0_pansharpen_8.tif");
  //    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/11OCT14WV031100014OCT11155720-P1BS-500648061020_01_P001_________AAE_0AAAAABPABR0_pansharpen_8.tif");
  //    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/08FEB15WV031100015FEB08160130-P1BS-500648061090_01_P001_________AAE_0AAAAABPABP0_pansharpen_8.tif");
  //    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/02MAY15WV031100015MAY02161943-P1BS-500648061030_01_P001_________AAE_0AAAAABPABR0_pansharpen_8.tif");
  //    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/01MAY15WV031200015MAY01160357-P1BS-500648062030_01_P001_________AAE_0AAAAABPABQ0_pansharpen_8.tif");

  images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/15FEB15WV031200015FEB15161208-P1BS-500648061070_01_P001_________AAE_0AAAAABPABP0_pansharpen_8.tif");
//  images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/05JUL15WV031100015JUL05162954-P1BS-500648062020_01_P001_________AAE_0AAAAABPABQ0_pansharpen_8.tif");
//  images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/01NOV15WV031100015NOV01162034-P1BS-500648061060_01_P001_________AAE_0AAAAABPABE0_pansharpen_8.tif");
//  images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/01NOV15WV031100015NOV01161954-P1BS-500648062080_01_P001_________AAE_0AAAAABPABS0_pansharpen_8.tif");
//  images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/01MAY15WV031200015MAY01160357-P1BS-500648062030_01_P001_________AAE_0AAAAABPABQ0_pansharpen_8.tif");

  // camera
  kwiver::vital::camera_sptr_list cameras;
  for (auto s: images_filenames)
  {
    cameras.push_back(loadcamera_from_tif_image(s));
  }

  // image
  kwiver::vital::image_container_sptr_list images;
  kwiver::vital::algo::image_io_sptr image_tif_io = kwiver::vital::algo::image_io::create("gdal");
  for (auto s: images_filenames)
  {
    images.push_back(image_tif_io->load(s));
  }



  /** ----- */
  std::unique_ptr< kwiver::vital::mesh_vertex_array<3> > v_mesh_utm(new kwiver::vital::mesh_vertex_array<3>(mesh->num_verts()));
  std::unique_ptr< kwiver::vital::mesh_vertex_array<3> > v_occlusion_utm(new kwiver::vital::mesh_vertex_array<3>(occlusion_mesh->num_verts()));
  std::unique_ptr< kwiver::vital::mesh_vertex_array<3> > v_mesh_latlong(new kwiver::vital::mesh_vertex_array<3>(mesh->num_verts()));
  std::unique_ptr< kwiver::vital::mesh_vertex_array<3> > v_occlusion_latlong(new kwiver::vital::mesh_vertex_array<3>(occlusion_mesh->num_verts()));

  *v_mesh_utm = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(mesh->vertices());
  *v_occlusion_utm = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(occlusion_mesh->vertices());

  int dest_coord_sys = kwiver::vital::SRID::lat_lon_WGS84;

  kwiver::vital::utm_ups_zone_t utm_zone = kwiver::vital::utm_ups_zone(std::dynamic_pointer_cast<kwiver::vital::camera_rpc>(cameras[0])->back_project({0.0, 0.0}, 0.0).head(2));
  int src_coord_sys = (utm_zone.north ? kwiver::vital::SRID::UTM_WGS84_north : kwiver::vital::SRID::UTM_WGS84_south) + utm_zone.number;

  for (int i = 0; i < v_mesh_utm->size(); ++i)
  {
//    std::cout << "utm " << v_mesh_utm[i] << std::endl;
    (*v_mesh_latlong)[i] = (*v_mesh_utm)[i] + mesh_offset;
    auto res = kwiver::vital::geo_conv((*v_mesh_latlong)[i].head<2>(), src_coord_sys, dest_coord_sys);
    (*v_mesh_latlong)[i].head<2>() = res;
//    std::cout << "latlong " << v_mesh_latlong[i] << std::endl;
  }

  for (int i = 0; i < v_occlusion_utm->size(); ++i)
  {
    (*v_occlusion_latlong)[i] = (*v_occlusion_utm)[i] + mesh_offset;
    auto res = kwiver::vital::geo_conv((*v_occlusion_latlong)[i].head<2>(), src_coord_sys, dest_coord_sys);
    (*v_occlusion_latlong)[i].head<2>() = res;
  }



  /** ------ */






  double resolution = 0.3;   // mesh unit/pixel
  int interior_margin = 2;
  int exterior_margin = 2;

  kwiver::vital::algo::compute_mesh_uv_parameterization_sptr uv_param =
      kwiver::vital::algo::compute_mesh_uv_parameterization::create("core");

  kwiver::vital::config_block_sptr algo_config = uv_param->get_configuration();
  algo_config->set_value<double>("resolution", resolution);
  algo_config->set_value<double>("interior_margin", interior_margin);
  algo_config->set_value<double>("exterior_margin", exterior_margin);
  uv_param->set_configuration(algo_config);

  std::cout << "mesh regularity = " << mesh->faces().regularity() << std::endl;

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


  std::pair<unsigned int, unsigned int> atlas_dim = uv_param->parameterize(mesh);
  std::cout << "uv param is computed " << std::endl;

  auto const& param = mesh->tex_coords();
  draw_uv_parameterization(param, atlas_dim.first, atlas_dim.second, "uv_param.png");
  std::cout << "uv param is drawed" << std::endl;
  std::cout << "atlas dim " << atlas_dim.first << " " << atlas_dim.second << std::endl;


  //  use LatLong vertices
  mesh->set_vertices(std::move(v_mesh_latlong));
  occlusion_mesh->set_vertices(std::move(v_occlusion_latlong));


  kwiver::vital::plugin_manager::instance().load_all_plugins();
  kwiver::vital::algo::image_io_sptr ocv_io = kwiver::vital::algo::image_io::create("ocv");


  kwiver::vital::image_of<unsigned short> texture(atlas_dim.first, atlas_dim.second, images[0]->depth());
  for (int i = 0; i < texture.height(); ++i)
  {
    for (int j= 0 ;j < texture.width(); ++j)
    {
      texture(j, i) = 0;
    }
  }
  std::cout << "ATLAS DIM " << texture.width() << " " << texture.height() << std::endl;

  kwiver::vital::mesh_vertex_array<3>& vertices = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(mesh->vertices());
  auto const& triangles = static_cast< const kwiver::vital::mesh_regular_face_array<3>& >(mesh->faces());
  auto const& tcoords =  mesh->tex_coords();
  double d1 = 1, d2 = 1, d3 = 1;

  std::vector<kwiver::vital::image> temps(images.size());
  for (int i = 0; i < images.size(); ++i)
  {
    temps[i] = images[i]->get_image();
  }
  std::cout << "before depth map generation " << std::endl;

  std::vector<kwiver::vital::image> depth_maps(images.size());
  for (int i = 0; i < images.size(); ++i)
  {
    depth_maps[i] = kwiver::arrows::render_mesh_height_map(mesh, cameras[i])->get_image();
  }
  std::cout << "Depth maps rendered " << std::endl;

//  cv::Mat points_projected(temp.height(), temp.width(), CV_8U);
  kwiver::vital::vector_2d scale(texture.width(), texture.height());
  int nb=0;
  for (int f = 0; f < triangles.size(); ++f)
  {
    kwiver::vital::vector_2d a = tcoords[f * 3 + 0];
    kwiver::vital::vector_2d b = tcoords[f * 3 + 1];
    kwiver::vital::vector_2d c = tcoords[f * 3 + 2];
    a.y() = 1.0 - a.y();
    b.y() = 1.0 - b.y();
    c.y() = 1.0 - c.y();
    a = a.cwiseProduct(scale);
    b = b.cwiseProduct(scale);
    c = c.cwiseProduct(scale);
    kwiver::vital::vector_3d pt_a = vertices[triangles(f, 0)];
    kwiver::vital::vector_3d pt_b = vertices[triangles(f, 1)];
    kwiver::vital::vector_3d pt_c = vertices[triangles(f, 2)];
//    std::cout << a(0) << " " << a(1) << std::endl;
//    std::cout << b(0) << " " << b(1) << std::endl;
//    std::cout << c(0) << " " << c(1) << std::endl;
//    std::cout << "-----------------------------\n";


    std::vector<kwiver::vital::vector_3d> depths(images.size());
    for (int i = 0; i < depths.size(); ++i)
    {
  //    d1 = dynamic_cast<kwiver::vital::camera_perspective*>(camera.get())->depth(pt_a);
  //    d2 = dynamic_cast<kwiver::vital::camera_perspective*>(camera.get())->depth(pt_b);
  //    d3 = dynamic_cast<kwiver::vital::camera_perspective*>(camera.get())->depth(pt_c);
      depths[i](0) = pt_a(2);
      depths[i](1) = pt_b(2);
      depths[i](2) = pt_c(2);
    }


    kwiver::vital::vector_2d t1 = b-a;
    kwiver::vital::vector_2d t2 = c-a;
    if ( std::abs(t1(0)*t2(1)-t1(1)*t2(0)) < 0.01)
    {
      continue;
    }
    nb++;

//   auto tmp1 = camera->project(pt_a);
//   auto tmp2 = camera->project(pt_b);
//   auto tmp3 = camera->project(pt_c);
//   points_projected.at<uchar>(std::round(tmp1(1)), std::round(tmp1(0))) = 255;
//   points_projected.at<uchar>(std::round(tmp2(1)), std::round(tmp2(0))) = 255;
//   points_projected.at<uchar>(std::round(tmp3(1)), std::round(tmp3(0))) = 255;


    kwiver::arrows::render_triangle_from_image<unsigned short>(a, b, c, pt_a, pt_b, pt_c, cameras, temps, depths, depth_maps, texture);
  }

//  cv::imwrite("projected_points.png", points_projected);

  std::cout << "NB FACES = " << nb << " over " << triangles.size() << std::endl;

  kwiver::vital::image_container_sptr out_texture(new kwiver::vital::simple_image_container(texture));

  std::cout << "depth  = " << out_texture->depth() << std::endl;


  // write 3-channel texture
//  ocv_io->save("texture.png", out_texture);

  //---------------- write tif texture
  cv::Mat tex = kwiver::arrows::ocv::image_container_to_ocv_matrix(*out_texture, kwiver::arrows::ocv::image_container::OTHER_COLOR);
  std::cout << "tex size " << tex.size() << std::endl;
  std::vector<cv::Mat> channels;
  cv::split(tex, channels);
  std::cout << "channels " << channels.size() << std::endl;
  std::vector<cv::Mat> rgb;
  rgb.push_back(channels[4]);
  rgb.push_back(channels[2]);
  rgb.push_back(channels[1]);
  cv::Mat rgb_tex;
  cv::merge(rgb, rgb_tex);
  rgb_tex.convertTo(rgb_tex, CV_8UC3);
  cv::imwrite("texture.png", rgb_tex);



  //------------------




    std::vector<kwiver::vital::vector_2d> tcoords2 = mesh->tex_coords();

    for (auto& tc : tcoords2)
    {
      tc(0) *= texture.width();
      tc(1) *= texture.height();

      tc(0) += 0.5;
      tc(1) -= 0.5;

      tc(0) /= texture.width();
      tc(1) /= texture.height();
    }
    mesh->set_tex_coords(tcoords2);

  mesh->set_vertices(std::move(v_mesh_utm));
  occlusion_mesh->set_vertices(std::move(v_occlusion_utm));

  kwiver::vital::write_obj("texture_mesh.obj", *mesh);

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


  std::cout << "image " << sizeof(kwiver::vital::image) << std::endl;
  std::cout << "image_memory_sptr " << sizeof(kwiver::vital::image_memory_sptr) << std::endl;
  std::cout << "image_pixel_traits " << sizeof(kwiver::vital::image_pixel_traits) << std::endl;
  std::cout << "size_t " << sizeof(size_t) << std::endl;
  std::cout << "ptrdiff_t " << sizeof(ptrdiff_t) << std::endl;
  std::cout << "image container " << sizeof(kwiver::vital::simple_image_container) << std::endl;

  //  kwiver::vital::image_of<double> depth(400, 400);
  //  kwiver::vital::image_of<double> img(400, 400);
  //  for (int i = 0; i < img.height(); ++i)
  //  {
  //    for (int j= 0 ;j < img.width(); ++j)
  //    {
  //      depth(j, i) = std::numeric_limits<double>::infinity();
  //      img(j, i) = 0;
  //    }
  //  }
  //  kwiver::vital::vector_2d v1(0, 0);
  //  kwiver::vital::vector_2d v2(390, 0);
  //  kwiver::vital::vector_2d v3(150, 390);
  //  double d1 = 10;
  //  double d2 = 10;
  //  double d3 = 1;
  //  double i1 = 10.0;
  //  double i2 = 10.0;
  //  double i3 = 1.0;

  //  kwiver::vital::vector_2d g = (v1 + v2 + v3) / 3;

  //  kwiver::arrows::render_triangle<double>(v1, v2, v3, d1, d2, d3, i1, i2, i3,  depth, img, true);

  //  std::cout << "Barycenter " << (int)img((int)g(0), (int)g(1)) << std::endl;


  return 0; //#####################################

// ----- depth map 1
  kwiver::vital::image_container_sptr res;
  PROFILE("render_mesh_depth map",
    res = kwiver::arrows::render_mesh_depth_map2(mesh, std::dynamic_pointer_cast<kwiver::vital::camera_perspective>(cameras[0]));
)
    //  // write depthmap
    cv::Mat image = kwiver::arrows::ocv::image_container_to_ocv_matrix(*res,  kwiver::arrows::ocv::image_container::OTHER_COLOR);

    double min, max;
    cv::minMaxLoc(image, &min, &max, 0, 0);
    for (int i = 0; i < image.rows; ++i)
    {
      for (int j= 0 ; j < image.cols; ++j)
      {
        if (std::isinf(image.at<double>(i, j) ))
        {
          image.at<double>(i, j) = 0;
        }
      }
    }
    cv::minMaxLoc(image, &min, &max, 0, 0);
    std::cout << "min max " << min << " " << max << std::endl;
    image -= min;
    image /= (max-min);
    image *= 255;
    image.convertTo(image, CV_8U);
    cv::imwrite("depthmap.png", image);

// ---- height map 2

  kwiver::vital::image_container_sptr res2;
  PROFILE("render_mesh_height map 2",
    res2 = kwiver::arrows::render_mesh_height_map(mesh, cameras[0]);
)
    //  // write depthmap
    cv::Mat image2 = kwiver::arrows::ocv::image_container_to_ocv_matrix(*res2,  kwiver::arrows::ocv::image_container::OTHER_COLOR);

    double min2, max2;
    cv::minMaxLoc(image2, &min2, &max2, 0, 0);
    for (int i = 0; i < image2.rows; ++i)
    {
      for (int j= 0 ; j < image2.cols; ++j)
      {
        if (std::isinf(image2.at<double>(i, j) ))
        {
          image2.at<double>(i, j) = min2;
        }
      }
    }
    cv::minMaxLoc(image2, &min2, &max2, 0, 0);
    std::cout << "min max " << min2 << " " << max2 << std::endl;
    image2 -= min2;
    image2 /= (max2-min2);
    image2 *= 255;
    image2.convertTo(image2, CV_8U);
    cv::imwrite("heightmap.png", image2);



  /** ---------------------------------- MESH UV PARAM ---------------------- **/
  //  using namespace kwiver::vital;
  //  // use comments to execute a particular method
  //  kwiver::vital::plugin_manager::instance().load_all_plugins();
  ////  how_to_part_01_images();
  ////  how_to_part_02_detections();


  //  // cube mesh of size 1.0
  //  std::vector<vector_3d> verts = {
  //        {-0.500000, -0.500000, -0.500000},
  //        {-0.500000, -0.500000, 0.500000},
  //        {-0.500000, 0.500000, -0.500000},
  //        {-0.500000, 0.500000, 0.500000},
  //        {0.500000 ,-0.500000, -0.500000},
  //        {0.500000 ,-0.500000, 0.500000},
  //        {0.500000 ,0.500000 ,-0.500000},
  //        {0.500000 ,0.500000 ,0.500000}
  //  };
  //  std::vector< mesh_regular_face<3> > faces;
  //  faces.push_back(mesh_regular_face<3>({0, 1, 2}));
  //  faces.push_back(mesh_regular_face<3>({3, 2, 1}));
  //  faces.push_back(mesh_regular_face<3>({4, 6, 5}));
  //  faces.push_back(mesh_regular_face<3>({7, 5, 6}));
  //  faces.push_back(mesh_regular_face<3>({0, 4, 1}));
  //  faces.push_back(mesh_regular_face<3>({5, 1, 4}));
  //  faces.push_back(mesh_regular_face<3>({2, 3, 6}));
  //  faces.push_back(mesh_regular_face<3>({7, 6, 3}));
  //  faces.push_back(mesh_regular_face<3>({0, 2, 4}));
  //  faces.push_back(mesh_regular_face<3>({6, 4, 2}));
  //  faces.push_back(mesh_regular_face<3>({1, 5, 3}));
  //  faces.push_back(mesh_regular_face<3>({7, 3, 5}));

  //  std::unique_ptr<mesh_vertex_array_base> vertices_array_ptr(new mesh_vertex_array<3>(verts));
  //  std::unique_ptr<mesh_face_array_base> faces_array_ptr(new mesh_regular_face_array<3>(faces));
  //  mesh_sptr m = std::make_shared<mesh>(std::move(vertices_array_ptr), std::move(faces_array_ptr));


  //  double resolution = 0.03;   // mesh unit/pixel
  //  int interior_margin = 2;
  //  int exterior_margin = 2;

  //  kwiver::vital::algo::compute_mesh_uv_parameterization_sptr uv_param =
  //      kwiver::vital::algo::compute_mesh_uv_parameterization::create("core");

  //  kwiver::vital::config_block_sptr algo_config = uv_param->get_configuration();
  //  algo_config->set_value<double>("resolution", resolution);
  //  algo_config->set_value<double>("interior_margin", interior_margin);
  //  algo_config->set_value<double>("exterior_margin", exterior_margin);
  //  uv_param->set_configuration(algo_config);

  //  std::pair<unsigned int, unsigned int> atlas_dim = uv_param->parameterize(m);
  //  std::cout << atlas_dim.first << " " << atlas_dim.second << std::endl;

  //  std::cout << m->has_tex_coords() << std::endl;
  //  std::vector<kwiver::vital::vector_2d> tcoords = m->tex_coords();

  //  for (auto tc : tcoords)
  //  {
  //    std::cout << tc(0) << " " << tc(1) << std::endl;
  //  }

  //  kwiver::vital::write_obj("mesh_ou.obj", *m);
}
