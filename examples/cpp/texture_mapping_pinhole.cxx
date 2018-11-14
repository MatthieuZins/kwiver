#include <iostream>
#include <fstream>

#include <arrows/core/render.h>
#include <arrows/core/render_mesh_depth_map.h>
#include <arrows/ocv/image_container.h>

#include <vital/plugin_loader/plugin_manager.h>

#include <vital/algo/compute_mesh_uv_parameterization.h>
#include <vital/algo/image_io.h>

#include <vital/exceptions.h>

#include <vital/io/mesh_io.h>
#include <vital/io/camera_from_metadata.h>

#include <vital/types/camera_perspective.h>
#include <vital/types/image.h>
#include <vital/types/mesh.h>

#include <kwiversys/SystemTools.hxx>
#include <opencv2/opencv.hpp>

using namespace kwiver;

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

vital::image texture_mapping_pinhole(vital::mesh_sptr mesh, const vital::camera_sptr_list& cameras,
                             const std::vector<vital::image>& images);


void run_texture_mapping_pinhole()
{
  kwiver::vital::plugin_manager::instance().load_all_plugins();


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


  std::vector<kwiver::vital::camera_sptr> cameras;
  for (int c = 0; c < param_cams.size(); ++c)
  {
    cameras.push_back(std::make_shared<kwiver::vital::simple_camera_perspective>(param_cams[c].first, param_cams[c].second.inverse(), camera_intrinsic));
  }

  /// Images
  std::vector<kwiver::vital::image_container_sptr> images_temp;
  std::vector<kwiver::vital::image> images;
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

  for (int i=0; i < images_temp.size(); ++i)
  {
    images.push_back(images_temp[i]->get_image());
  }

  /// Mesh
  kwiver::vital::mesh_sptr mesh;
//  mesh = kwiver::vital::read_obj("/home/matthieu/data_cube_texture/cube.obj");
  mesh = kwiver::vital::read_obj("/home/matthieu/data_plane/f16.obj");
//  mesh = kwiver::vital::read_obj("/home/matthieu/data_towers/towers.obj");

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



  auto const& texture = texture_mapping_pinhole(mesh, cameras, images);

  /// Write textured mesh
  kwiver::vital::algo::image_io_sptr ocv_io = kwiver::vital::algo::image_io::create("ocv");
  kwiver::vital::image_container_sptr out_texture(new kwiver::vital::simple_image_container(texture));
  ocv_io->save("texture.png", out_texture);

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


vital::image texture_mapping_pinhole(vital::mesh_sptr mesh, const vital::camera_sptr_list& cameras,
                             const std::vector<vital::image>& images)
{

  /// Mesh uv parameterization
  double resolution = 0.001;   // mesh unit/pixel
  int interior_margin = 8;
  int exterior_margin = 2;

  kwiver::vital::algo::compute_mesh_uv_parameterization_sptr uv_param =
      kwiver::vital::algo::compute_mesh_uv_parameterization::create("core");

  kwiver::vital::config_block_sptr algo_config = uv_param->get_configuration();
  algo_config->set_value<double>("resolution", resolution);
  algo_config->set_value<double>("interior_margin", interior_margin);
  algo_config->set_value<double>("exterior_margin", exterior_margin);
  uv_param->set_configuration(algo_config);
  std::pair<unsigned int, unsigned int> atlas_dim = uv_param->parameterize(mesh);
  auto const& param = mesh->tex_coords();
  draw_uv_parameterization(param, atlas_dim.first, atlas_dim.second, "uv_param.png");


  /// Render mesh depth/height maps
  std::vector<kwiver::vital::image> depth_maps(images.size());
  for (int i = 0; i < images.size(); ++i)
  {
    depth_maps[i] = kwiver::arrows::render_mesh_depth_map(mesh, std::dynamic_pointer_cast<vital::camera_perspective>(cameras[i]))->get_image();
    //  // write depthmap
    kwiver::vital::image_container_sptr container(new kwiver::vital::simple_image_container(depth_maps[i]));
    cv::Mat image2 = kwiver::arrows::ocv::image_container_to_ocv_matrix(*container,  kwiver::arrows::ocv::image_container::OTHER_COLOR).clone();
    double min2, max2;
    cv::minMaxLoc(image2, &min2, &max2, nullptr, nullptr);
    for (int i = 0; i < image2.rows; ++i)
    {
      for (int j= 0 ; j < image2.cols; ++j)
      {
        if (std::isinf(image2.at<double>(i, j) ))
        {
          image2.at<double>(i, j) = 0;
        }
      }
    }
    cv::minMaxLoc(image2, &min2, &max2, 0, 0);
    std::cout << "min max " << min2 << " " << max2 << std::endl;
    image2 -= min2;
    image2 /= (max2-min2);
    image2 *= 255;
    image2.convertTo(image2, CV_8U);
    cv::imwrite("heightmap_" + std::to_string(i) + ".png", image2);
  }



  /// Create empty texture
  kwiver::vital::image_of<unsigned char> texture(atlas_dim.first, atlas_dim.second, 3/* images[0].depth()*/);
  kwiver::vital::image_of<bool> texture_label(atlas_dim.first, atlas_dim.second, 1);
  for (int i = 0; i < texture.height(); ++i)
  {
    for (int j= 0 ;j < texture.width(); ++j)
    {
      for (int k = 0; k < texture.depth(); ++k)
      {
        texture(j, i, k) = 0;
        texture_label(j, i) = false;
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
//      depths[i](0) = pt_a(2);
//      depths[i](1) = pt_b(2);
//      depths[i](2) = pt_c(2);
      depths[i](0) = std::dynamic_pointer_cast<vital::camera_perspective>(cameras[i])->depth(pt_a);
      depths[i](1) = std::dynamic_pointer_cast<vital::camera_perspective>(cameras[i])->depth(pt_b);
      depths[i](2) = std::dynamic_pointer_cast<vital::camera_perspective>(cameras[i])->depth(pt_c);
    }

    kwiver::vital::vector_2d t1 = b-a;
    kwiver::vital::vector_2d t2 = c-a;
    if (std::abs(t1(0) * t2(1) - t1(1) * t2(0)) < 0.001)
      continue;
    kwiver::arrows::render_triangle_from_image<unsigned char>(a, b, c, pt_a, pt_b, pt_c, cameras, images, depths, depth_maps, texture, 0.01);

    kwiver::arrows::render_triangle_label<bool>(a, b, c, true, texture_label);
  }



  kwiver::arrows::dilate_atlas<unsigned char>(texture, texture_label, 4);

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

  return texture;
}


