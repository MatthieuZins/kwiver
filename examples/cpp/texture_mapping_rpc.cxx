#include <iostream>
#include <fstream>

#include <arrows/core/render.h>
#include <arrows/core/render_mesh_depth_map.h>

#include <vital/plugin_loader/plugin_manager.h>

#include <vital/algo/compute_mesh_uv_parameterization.h>
#include <vital/algo/image_io.h>

#include <vital/io/mesh_io.h>
#include <vital/io/camera_from_metadata.h>

#include <vital/types/camera_rpc.h>
#include <vital/types/image.h>
#include <vital/types/geodesy.h>
#include <vital/types/mesh.h>

#include <arrows/ocv/image_container.h>
#include <opencv2/opencv.hpp>


using namespace kwiver;

namespace {
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
}


vital::image texture_mapping_rpc(vital::mesh_sptr mesh, const vital::camera_sptr_list& cameras,
                                 const std::vector<vital::image>& images);



void run_texture_mapping_rpc()
{
  kwiver::vital::plugin_manager::instance().load_all_plugins();
  /// Prepare data and call the real function
  kwiver::vital::mesh_sptr mesh;
//  kwiver::vital::vector_3d mesh_offset = {435516.726081, 3354093.8, -47.911346};
  kwiver::vital::vector_3d mesh_offset = {435531.480325, 3354095.24186, -36.561596}; // for jido
  mesh = kwiver::vital::read_obj("/home/matthieu/Downloads/JIDO-Jacksonville-Model/Jacksonville_OBJ_Buildings_Only_offset_cropped2.obj");
  /// Turn mesh into reglar faces (should be done also for the occlusion mesh)
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
  /// Add offset
  vital::mesh_vertex_array<3>& verts = dynamic_cast<vital::mesh_vertex_array<3>&>(mesh->vertices());
  for (auto & v : verts)
  {
    v = v + mesh_offset;
  }


  /// Load cameras and images
  std::vector<std::string> images_filenames;
  images_filenames.push_back("/media/matthieu/DATA/core3d_results/20180928/AOI4/images/15FEB15WV031200015FEB15161208-P1BS-500648061070_01_P001_________AAE_0AAAAABPABP0_crop_pansharpened_processed.tif");
  images_filenames.push_back("/media/matthieu/DATA/core3d_results/20180928/AOI4/images/01NOV15WV031100015NOV01161954-P1BS-500648062080_01_P001_________AAE_0AAAAABPABS0_crop_pansharpened_processed.tif");
  images_filenames.push_back("/media/matthieu/DATA/core3d_results/20180928/AOI4/images/18OCT14WV031100014OCT18160722-P1BS-500648062090_01_P001_________AAE_0AAAAABPABS0_crop_pansharpened_processed.tif");
  images_filenames.push_back("/media/matthieu/DATA/core3d_results/20180928/AOI4/images/21JAN15WV031100015JAN21161253-P1BS-500648062050_01_P001_________AAE_0AAAAABPABO0_crop_pansharpened_processed.tif");

  // images
  std::vector<vital::image> images(images_filenames.size());
  kwiver::vital::algo::image_io_sptr image_tif_io = kwiver::vital::algo::image_io::create("gdal");
  for (int i = 0; i < images.size(); ++i)
  {
    images[i] = image_tif_io->load(images_filenames[i])->get_image();
  }

  // cameras
  kwiver::vital::camera_sptr_list cameras(images_filenames.size());
  for (int i = 0; i < cameras.size(); ++i)
  {
    cameras[i] = loadcamera_from_tif_image(images_filenames[i]);
  }

  auto const& texture = texture_mapping_rpc(mesh, cameras, images);



  /// Write textured mesh

  /// Extract RGB from tif images
  kwiver::vital::image_container_sptr out_texture(new kwiver::vital::simple_image_container(texture));
  cv::Mat tex = kwiver::arrows::ocv::image_container_to_ocv_matrix(*out_texture, kwiver::arrows::ocv::image_container::OTHER_COLOR);
  std::vector<cv::Mat> channels;
  cv::split(tex, channels);
  std::vector<cv::Mat> rgb;
  rgb.push_back(channels[1]);
  rgb.push_back(channels[2]);
  rgb.push_back(channels[4]);
  cv::Mat rgb_tex;
  cv::merge(rgb, rgb_tex);
  rgb_tex.convertTo(rgb_tex, CV_8UC3);
  cv::imwrite("texture.png", rgb_tex);


  /// Write OBJ
  /// Remove offset
  vital::mesh_vertex_array<3>& verts2 = dynamic_cast<vital::mesh_vertex_array<3>&>(mesh->vertices());
  for (auto & v : verts2)
  {
    v = v - mesh_offset;
  }
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


/// Texture mapping for RPC cameras
/// inputs: mesh (utm), images, cameas
/// outputs: mesh gets tcoords, texture
vital::image texture_mapping_rpc(vital::mesh_sptr mesh, const vital::camera_sptr_list& cameras,
                                 const std::vector<vital::image>& images) // return image or image_container
{
  /// Mesh uv parameterization
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

  std::pair<unsigned int, unsigned int> atlas_dim = uv_param->parameterize(mesh);
//  auto const& param = mesh->tex_coords();
//  draw_uv_parameterization(param, atlas_dim.first, atlas_dim.second, "uv_param.png");


  /// Create arrays of points in lat long
  std::unique_ptr< kwiver::vital::mesh_vertex_array<3> > v_mesh_utm(new kwiver::vital::mesh_vertex_array<3>(mesh->num_verts()));
  std::unique_ptr< kwiver::vital::mesh_vertex_array<3> > v_mesh_latlong(new kwiver::vital::mesh_vertex_array<3>(mesh->num_verts()));

  *v_mesh_utm = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(mesh->vertices());

  int dest_coord_sys = kwiver::vital::SRID::lat_lon_WGS84;
  kwiver::vital::utm_ups_zone_t utm_zone = kwiver::vital::utm_ups_zone(std::dynamic_pointer_cast<kwiver::vital::camera_rpc>(cameras[0])->back_project({0.0, 0.0}, 0.0).head(2));
  int src_coord_sys = (utm_zone.north ? kwiver::vital::SRID::UTM_WGS84_north : kwiver::vital::SRID::UTM_WGS84_south) + utm_zone.number;

  for (int i = 0; i < v_mesh_utm->size(); ++i)
  {
    (*v_mesh_latlong)[i] = (*v_mesh_utm)[i];
    auto res = kwiver::vital::geo_conv((*v_mesh_latlong)[i].head<2>(), src_coord_sys, dest_coord_sys);
    (*v_mesh_latlong)[i].head<2>() = res;
  }
  mesh->set_vertices(std::move(v_mesh_latlong));



  /// Render mesh depth/height maps
  std::vector<kwiver::vital::image> depth_maps(images.size());
  for (int i = 0; i < images.size(); ++i)
  {
    depth_maps[i] = kwiver::arrows::render_mesh_height_map(mesh, cameras[i])->get_image();
    //  // write depthmap
//    kwiver::vital::image_container_sptr container(new kwiver::vital::simple_image_container(depth_maps[i]));
//    cv::Mat image2 = kwiver::arrows::ocv::image_container_to_ocv_matrix(*container,  kwiver::arrows::ocv::image_container::OTHER_COLOR).clone();
//    double min2, max2;
//    cv::minMaxLoc(image2, &min2, &max2, 0, 0);
//    for (int i = 0; i < image2.rows; ++i)
//    {
//      for (int j= 0 ; j < image2.cols; ++j)
//      {
//        if (std::isinf(image2.at<double>(i, j) ))
//        {
//          image2.at<double>(i, j) = min2;
//        }
//      }
//    }
//    cv::minMaxLoc(image2, &min2, &max2, 0, 0);
//    std::cout << "min max " << min2 << " " << max2 << std::endl;
//    image2 -= min2;
//    image2 /= (max2-min2);
//    image2 *= 255;
//    image2.convertTo(image2, CV_8U);
//    cv::imwrite("heightmap_" + std::to_string(i) + ".png", image2);
  }

  /// Create empty texture
  kwiver::vital::image_of<unsigned short> texture(atlas_dim.first, atlas_dim.second, images[0].depth());
  for (int i = 0; i < texture.height(); ++i)
  {
    for (int j= 0 ;j < texture.width(); ++j)
    {
      for (int k = 0; k < texture.depth(); ++k)
      {
        texture(j, i, k) = 0;
      }
    }
  }
  /// Fill texture
  std::vector<vital::vector_2d> tcoords = mesh->tex_coords();
  kwiver::vital::mesh_vertex_array<3>& vertices = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(mesh->vertices());
  auto const& triangles = static_cast< const kwiver::vital::mesh_regular_face_array<3>& >(mesh->faces());
  kwiver::vital::vector_2d scale(texture.width(), texture.height());
  for (int f = 0; f < mesh->num_faces() ;++f)
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

    std::vector<kwiver::vital::vector_3d> depths(images.size());
    for (int i = 0; i < depths.size(); ++i)
    {
      depths[i](0) = pt_a(2);
      depths[i](1) = pt_b(2);
      depths[i](2) = pt_c(2);
    }

    kwiver::vital::vector_2d t1 = b-a;
    kwiver::vital::vector_2d t2 = c-a;
    if (std::abs(t1(0) * t2(1) - t1(1) * t2(0)) < 0.001)
      continue;
    kwiver::arrows::render_triangle_from_image<unsigned short>(a, b, c, pt_a, pt_b, pt_c, cameras, images, depths, depth_maps, texture, 0.1);
  }


  /// Half-pixel shift (need to test different shifts)
  for (auto& tc : tcoords)
  {
    tc(0) *= texture.width();
    tc(1) *= texture.height();

    tc(0) += 0.5;
    tc(1) -= 0.5;

    tc(0) /= texture.width();
    tc(1) /= texture.height();
  }
  mesh->set_tex_coords(tcoords);

  /// Re-set UTM vertices
  mesh->set_vertices(std::move(v_mesh_utm));

  return texture;
}
