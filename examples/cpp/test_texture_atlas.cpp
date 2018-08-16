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
#include <vital/types/image_container.h>

#include <vital/plugin_loader/plugin_manager.h>

#include <opencv2/opencv_modules.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <arrows/core/compute_mesh_depthmap.h>
#include <arrows/core/mesh_uv_parameterization.h>
#include <arrows/core/mesh_io.h>
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
void test_uv_parameterization()
{
    kwiver::arrows::core::uv_parameterization_t param;
    std::vector<kwiver::arrows::core::tcoord_t> tcoords_test;
    for (int i=0; i < 25; ++i)
    {
        tcoords_test.push_back({i * 5 + 3, i * i + 1});
    }
    param.tcoords = tcoords_test;

    double bounds[4];
    param.get_bounds(bounds);
    std::cout << "xmin, xmax: " << bounds[0] << " " << bounds[1] << std::endl;
    std::cout << "ymin, ymax: " << bounds[2] << " " << bounds[3] << std::endl;


    kwiver::arrows::core::mesh_io mesh_io;
//    kwiver::vital::mesh_sptr mesh = mesh_io.load("/media/matthieu/DATA/core3D-data/AOI4/meshes/AOI4_Purdue.obj");
    kwiver::vital::mesh_sptr mesh = mesh_io.load("/home/matthieu/cube.obj");


    std::cout << "mesh vertices: " << mesh->num_verts() << std::endl;
    std::cout << "mesh faces: " << mesh->num_faces() << std::endl;

    param = kwiver::arrows::core::parameterize(mesh, 0.025, 8000, 5, 3);
//    for (auto f: param.face_mapping)
//    {
//        std::cout << "face: " << std::endl;
//        std::cout << param.tcoords[f[0]] << std::endl;
//        std::cout << param.tcoords[f[1]] << std::endl;
//        std::cout << param.tcoords[f[2]] << std::endl;
//    }
    param.get_bounds(bounds);
    std::cout << "parameterization bounds: " << std::endl;
    std::cout << "xmin, xmax: " << bounds[0] << " " << bounds[1] << std::endl;
    std::cout << "ymin, ymax: " << bounds[2] << " " << bounds[3] << std::endl;


    cv::Mat image(bounds[3] + 1 + 3, bounds[1] + 1 + 3, CV_8UC3, 0.0); // +3 correspond to the
    // margins at the right and bottom borders

    std::vector<cv::Point2i> points(3);
    for (unsigned int f=0; f < param.face_mapping.size(); ++f)
    {
        const kwiver::arrows::core::tcoord_t& tcoord_0 = param.tcoords[param.face_mapping[f][0]];
        const kwiver::arrows::core::tcoord_t& tcoord_1 = param.tcoords[param.face_mapping[f][1]];
        const kwiver::arrows::core::tcoord_t& tcoord_2 = param.tcoords[param.face_mapping[f][2]];

        points[0].x = std::round(tcoord_0[0]);
        points[0].y = std::round(tcoord_0[1]);

        points[1].x = std::round(tcoord_1[0]);
        points[1].y = std::round(tcoord_1[1]);

        points[2].x = std::round(tcoord_2[0]);
        points[2].y = std::round(tcoord_2[1]);

        cv::Scalar random_color(rand() % 255, rand() % 255, rand() % 255);
        cv::polylines(image, points, true, random_color);
    }
    cv::imwrite("parameterization.png", image);
}

kwiver::vital::camera_rpc_sptr
load_camera_from_tif_image(const std::string& filename, unsigned int utm_zone)
{
//    std::string filename("/media/matthieu/DATA/core3D-data/AOI1/images/pansharpen/rescaled/01JAN17WV031200017JAN01163905-P1BS-501073324070_01_P001_________AAE_0AAAAABPABM0_pansharpen_4.tif");
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

void test_depthmap()
{
    kwiver::vital::plugin_manager::instance().load_all_plugins();
    kwiver::arrows::core::compute_mesh_depthmap depthmap_generator;
    kwiver::arrows::core::mesh_io mesh_io;
//    kwiver::vital::mesh_sptr mesh =  mesh_io.load("/home/matthieu/cube.obj");
//    kwiver::vital::mesh_sptr mesh =  mesh_io.load("/home/matthieu/AOI1_centered.obj");
    kwiver::vital::mesh_sptr mesh =  mesh_io.load("/media/matthieu/DATA/core3D-data/AOI4/meshes/AOI4_cropped.obj");


    // Perspective camera
    Eigen::Vector3d center(0, 0, 300);
//    Eigen::Vector3d mesh_offset(747572.5, 4407386.0, 226.0009);   //WPAFB
    Eigen::Vector3d mesh_offset(436213.25, 3354848.5, 53.4106);     //JACKSONVILLE
    center += mesh_offset;
    kwiver::vital::rotation_d orientation(3.14, Eigen::Vector3d(0, 1, 0));
    kwiver::vital::camera_intrinsics_sptr camera_intrinsic(new kwiver::vital::simple_camera_intrinsics(500, {500*4, 4*500}));
    kwiver::vital::camera_perspective_sptr camera(new kwiver::vital::simple_camera_perspective(center, orientation.inverse(), camera_intrinsic));

    // RPC camera
    kwiver::vital::camera_rpc_sptr cam = load_camera_from_tif_image("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/26APR15WV031200015APR26162435-P1BS-501504472050_01_P001_________AAE_0AAAAABPABR0_pansharpen_8.tif", 17);


    std::pair<kwiver::vital::image_container_sptr,
              kwiver::vital::image_container_sptr> depthmap_pair = depthmap_generator.compute(mesh, cam, 4077, 4606, 17);
    std::cout << "done." << std::endl;

    cv::Mat image = kwiver::arrows::ocv::image_container_to_ocv_matrix(*depthmap_pair.first,  kwiver::arrows::ocv::image_container::OTHER_COLOR);
    cv::Mat mask;
    cv::threshold(image, mask, std::numeric_limits<double>::max() / 2, 1, cv::THRESH_BINARY_INV);
    double min, max;
    cv::minMaxLoc(mask, &min, &max, 0, 0);
    mask.convertTo(mask, CV_8U);
    cv::minMaxLoc(image, &min, &max, 0, 0, mask);
    image -= min;
    image /= (max-min);
    image *= 255;
    cv::threshold(image, image, 255, 0, cv::THRESH_TRUNC);
    cv::normalize(image, image, 0, 255, cv::NORM_MINMAX);
    image.convertTo(image, CV_8U);
    cv::imwrite("depthmap.png", image);


    cv::Mat id_image = kwiver::arrows::ocv::image_container_to_ocv_matrix(*depthmap_pair.second, kwiver::arrows::ocv::image_container::OTHER_COLOR);
    id_image.convertTo(id_image, CV_32F);
    cv::threshold(id_image, id_image, 600, 0, cv::THRESH_TRUNC);
    cv::normalize(id_image, id_image, 0, 255, cv::NORM_MINMAX);
    id_image.convertTo(id_image, CV_8U);
    cv::imwrite("id_depthmap.png", id_image);
}


void test_proj4()
{
    kwiver::vital::plugin_manager::instance().load_all_plugins();

    std::cout << "id: " << kwiver::vital::get_geo_conv()->id() << std::endl;


    kwiver::vital::vector_2d utm_pt = kwiver::vital::geo_conv({-84.0822,   39.7774}, kwiver::vital::SRID::lat_lon_WGS84, kwiver::vital::SRID::UTM_WGS84_north + 16);       // WGS to UTM

    std::cout << utm_pt << "\n-------------------------------------------\n" << std::endl;
}

void test_rpc_camera()
{
    kwiver::vital::simple_camera_rpc cam;
//    std::cout << "camera depth " << cam.depth({10, 10, 10}) << std::endl;

    kwiver::vital::vector_3d world_scale = {0.1015, 0.0698, 501};
    kwiver::vital::vector_3d world_offset = {-84.1053,  39.7924, 232. };
    kwiver::vital::vector_2d image_scale = {21250.0, 19634.0};
    kwiver::vital::vector_2d image_offset = {-3261.0, -3226.0};

    kwiver::vital::rpc_matrix rpc_coeffs;
    rpc_coeffs << -0.007731984547, 1.036371350555, 0.000571564184, -0.031336997087, 9.308693e-05, 1.5846718e-05, 0.000133194972, 0.00013056833, 0.000155796844, -1.99407e-07, 7.2351e-08, -1.73472e-07, -4.67971e-07, -1.43193e-07, 4.63809e-07, -0.000537782633, 3.1469e-08, -1.0968e-08, 2.0036943e-05, 4.728e-09, 1, -0.005567722482, -0.004714976925, -0.000641270901, -4.7638374e-05, 9.98384e-07, -9.379604e-06, 8.103659e-06, 0.000101390522, -2.862e-08, -2.70038e-07, 5.414e-09, 3.286629e-06, 3.031e-09, -6.88826e-07, 1.3444285e-05, -1.5802e-08, 1.7427e-08, 2.61646e-07, -9.001e-09, 0.019378390535, 0.099345828828, -1.10886666603, 0.010913032364, -0.000174576673, 1.909662e-06, -1.0480013e-05, -0.000826373775, -0.000226533588, -6.42252e-07, -2.0936e-08, -1.5105e-07, 2.209486e-06, 9.829e-09, -3.09441e-07, -1.2454048e-05, -4.31497e-07, -1.4832e-08, 1.53232e-07, 1.24e-09, 1, -0.012471867109, -0.00374336399, -0.001312895399, 0.000274800697, -2.43118e-06, 2.1717097e-05, -1.017716e-06, -0.001379345582, 3.81162e-07, 1.15705e-07, -1.055e-09, 1.8679136e-05, -1.352e-08, 2.5352e-08, -0.000216295644, 4.1381e-08, -2.9282e-08, 3.478123e-06, -4.9e-11;

//    std::cout << "rpc_coeffs: " << rpc_coeffs << std::endl;
    cam.set_image_offset(image_offset);
    cam.set_image_scale(image_scale);
    cam.set_world_offset(world_offset);
    cam.set_world_scale(world_scale);
    cam.set_rpc_coeffs(rpc_coeffs);

    kwiver::vital::vector_3d mesh_center = {749375.413584, 4407077.61724, 240.546514};
    std::cout << "distance to mesh center: " << cam.depth(mesh_center) << std::endl;
}


void test_open_tif()
{
    std::string filename("/media/matthieu/DATA/core3D-data/AOI1/images/pansharpen/rescaled/01JAN17WV031200017JAN01163905-P1BS-501073324070_01_P001_________AAE_0AAAAABPABM0_pansharpen_4.tif");
    kwiver::vital::algo::image_io_sptr image_io = kwiver::vital::algo::image_io::create("gdal");

    kwiver::vital::image_container_sptr img = image_io->load(filename);
    kwiver::vital::metadata_sptr md = img->get_metadata();
    for(auto item: *(md.get()))
    {
        std::cout << item.second->name() << std::endl;
    }
}

void test_generate_triangles_map()
{
    kwiver::arrows::core::mesh_io mesh_io;
    kwiver::vital::mesh_sptr mesh = mesh_io.load("/home/matthieu/cube.obj");
//    kwiver::vital::mesh_sptr mesh = mesh_io.load("/media/matthieu/DATA/core3D-data/AOI4/meshes/AOI4_Purdue.obj");

    kwiver::arrows::core::uv_parameterization_t param;
    param = kwiver::arrows::core::parameterize(mesh, 0.025, 8000, 5, 3);
    kwiver::vital::image_container_sptr map = kwiver::arrows::core::generate_triangles_map(param, 3);
    cv::Mat cv_map = kwiver::arrows::ocv::image_container_to_ocv_matrix(*(map.get()), kwiver::arrows::ocv::image_container::OTHER_COLOR);
    double min, max;
    cv::minMaxLoc(cv_map, &min, &max, 0, 0);
    cv_map.convertTo(cv_map, CV_32F);
    std::cout << "cv_map type " << cv_map.type() << std::endl;
    cv_map -= min;
    cv_map /= (max-min);
    cv_map *= 255;
    cv_map.convertTo(cv_map, CV_8U);
    cv::imwrite("triangle_map.png", cv_map);
}

void test_mesh_cameras_ratings()
{
    kwiver::vital::plugin_manager::instance().load_all_plugins();

    kwiver::arrows::core::mesh_io mesh_io;
//    kwiver::vital::mesh_sptr mesh = mesh_io.load("/media/matthieu/DATA/core3D-data/AOI4/meshes/AOI4_Purdue.obj");
    kwiver::vital::mesh_sptr mesh = mesh_io.load("/home/matthieu/cube.obj");


    // Perspective camera
    Eigen::Vector3d center1(0, 0, 2);
    kwiver::vital::rotation_d orientation1(3.14159265359, Eigen::Vector3d(0, 1, 0));
    kwiver::vital::camera_intrinsics_sptr camera_intrinsic(new kwiver::vital::simple_camera_intrinsics(500, {500, 500}));
    kwiver::vital::camera_perspective_sptr cam1(new kwiver::vital::simple_camera_perspective(center1, orientation1.inverse(), camera_intrinsic));

    Eigen::Vector3d center2(2, 0, 2);
    kwiver::vital::rotation_d orientation2(3.14159265359 + 0.785, Eigen::Vector3d(0, 1, 0));
    kwiver::vital::camera_perspective_sptr cam2(new kwiver::vital::simple_camera_perspective(center2, orientation2.inverse(), camera_intrinsic));

    Eigen::Vector3d center3(0, -2, 2);
    kwiver::vital::rotation_d orientation3(3.14, Eigen::Vector3d(0, 1, 0));
    kwiver::vital::rotation_d orientation32(-0.785, Eigen::Vector3d(1, 0, 0));
    orientation3 = orientation3 * orientation32;
    kwiver::vital::camera_perspective_sptr cam3(new kwiver::vital::simple_camera_perspective(center3, orientation3.inverse(), camera_intrinsic));

    kwiver::vital::camera_sptr_list cameras = {cam1, cam2, cam3};

    std::vector< std::vector<float> > ratings;
    kwiver::arrows::core::compute_mesh_cameras_ratings(mesh, cameras, ratings);

    for (int i=0; i < cameras.size(); ++i)
    {
        std::cout << "cam" << i << " : ";
        for (int j=0; j < mesh->num_faces(); ++j)
        {
            std::cout << ratings[i][j] << " ";
        }
        std::cout << std::endl;
    }


    kwiver::vital::mesh_sptr mesh2 = mesh_io.load("/media/matthieu/DATA/core3D-data/AOI4/meshes/AOI4_cropped.obj");

    kwiver::vital::camera_rpc_sptr cam21 = load_camera_from_tif_image("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/01MAY15WV031200015MAY01160357-P1BS-500648062030_01_P001_________AAE_0AAAAABPABQ0_pansharpen_8.tif", 17);
    kwiver::vital::camera_rpc_sptr cam22 = load_camera_from_tif_image("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/05JUL15WV031100015JUL05162954-P1BS-500648062020_01_P001_________AAE_0AAAAABPABQ0_pansharpen_8.tif", 17);
    kwiver::vital::camera_rpc_sptr cam23 = load_camera_from_tif_image("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/26APR15WV031200015APR26162435-P1BS-501504472050_01_P001_________AAE_0AAAAABPABR0_pansharpen_8.tif", 17);

    std::vector< std::vector<float> > ratings2;
    kwiver::arrows::core::compute_mesh_cameras_ratings(mesh2, {cam21, cam22, cam23}, ratings2);

    for (int j=0; j < mesh2->num_faces(); ++j)
    {
        std::cout << ratings2[0][j] << " ";
    }
    std::cout << std::endl;
}

void test_rasterize()
{
    kwiver::vital::plugin_manager::instance().load_all_plugins();

    std::string mesh_filename = ("/media/matthieu/DATA/core3D-data/AOI_D4_(Nick)/result_new/buildings.obj");
//    std::string mesh_filename = ("/media/matthieu/DATA/core3D-data/AOI_D4_(Nick)/triangulated/41.obj");
//    std::string mesh_filename = ("/media/matthieu/DATA/core3D-data/AOI_D4_(Nick)/test.obj");
    std::string image_filename = ("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/01MAY15WV031200015MAY01160357-P1BS-500648062030_01_P001_________AAE_0AAAAABPABQ0_pansharpen_8.tif");

    // mesh
    kwiver::arrows::core::mesh_io mesh_io;
    kwiver::vital::mesh_sptr mesh = mesh_io.load(mesh_filename);

    kwiver::vital::mesh_vertex_array<3>& vertices = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(mesh->vertices());
    kwiver::vital::vector_3d mesh_offset = {435530.547508, 3354095.61004, -36.844062};
    for (int i=0; i < vertices.size(); ++i)
    {
        vertices[i] += mesh_offset;
    }

    // uv parameterization
    kwiver::arrows::core::uv_parameterization_t param = kwiver::arrows::core::parameterize(mesh, 0.3, 8000, 10, 5);

    // camera
    kwiver::vital::camera_rpc_sptr camera = load_camera_from_tif_image(image_filename, 17);

    // image
    kwiver::vital::algo::image_io_sptr image_io = kwiver::vital::algo::image_io::create("gdal");
    kwiver::vital::image_container_sptr image = image_io->load(image_filename);

    kwiver::vital::image_container_sptr id_map = kwiver::arrows::core::generate_triangles_map(param, 5);
    cv::Mat cv_map = kwiver::arrows::ocv::image_container_to_ocv_matrix(*(id_map.get()), kwiver::arrows::ocv::image_container::OTHER_COLOR);
    double min, max;
    cv::minMaxLoc(cv_map, &min, &max, 0, 0);
    cv_map.convertTo(cv_map, CV_32F);
    std::cout << "cv_map type " << cv_map.type() << std::endl;
    cv_map -= min;
    cv_map /= (max-min);
    cv_map *= 255;
    cv_map.convertTo(cv_map, CV_8U);
    cv::imwrite("triangle_map.png", cv_map);


    kwiver::arrows::core::compute_mesh_depthmap depthmap_generator;
    auto depthmaps = depthmap_generator.compute(mesh, camera, image->width(), image->height(), 17);

    auto raster_result = kwiver::arrows::core::rasterize<unsigned short>(mesh, param, id_map, image, camera, depthmaps.first, {});
    auto texture = std::get<0>(raster_result);
    cv::Mat cv_tex = kwiver::arrows::ocv::image_container_to_ocv_matrix(*(texture.get()),
                                                                        kwiver::arrows::ocv::image_container::OTHER_COLOR);
    std::cout << "cv_tex channels " << cv_tex.channels() << std::endl;
    std::vector<cv::Mat> splitted;
    cv::split(cv_tex, splitted);
    std::vector<cv::Mat> rgb_splitted = {splitted[1], splitted[2], splitted[4]};
    cv::Mat rgb_tex;
    cv::merge(rgb_splitted, rgb_tex);
    cv::minMaxLoc(rgb_tex, &min, &max, 0, 0);
    std::cout << "min max " << min << " " << max << std::endl;
    rgb_tex.convertTo(rgb_tex, CV_8UC3);
    cv::imwrite("texture.png", rgb_tex);

    for (int i=0; i < vertices.size(); ++i)
    {
        vertices[i] -= mesh_offset;
    }
    mesh_io.save("test.obj", mesh, &param, texture.get());


    //------------------------------
    std::cout << "uv_param: " << std::endl;
    for (int i=0; i < param.face_mapping.size(); ++i)
    {
        const kwiver::vital::vector_2d& a_uv = param.tcoords[param.face_mapping[i][0]];
        const kwiver::vital::vector_2d& b_uv = param.tcoords[param.face_mapping[i][1]];
        const kwiver::vital::vector_2d& c_uv = param.tcoords[param.face_mapping[i][2]];
    }

    double bounds[4];
    param.get_bounds(bounds);
    cv::Mat image_param(texture->height(), texture->width(), CV_8UC3, 0.0);
    // margins at the right and bottom borders

    std::vector<cv::Point2i> points(3);
    for (unsigned int f=0; f < param.face_mapping.size(); ++f)
    {
        const kwiver::arrows::core::tcoord_t& tcoord_0 = param.tcoords[param.face_mapping[f][0]];
        const kwiver::arrows::core::tcoord_t& tcoord_1 = param.tcoords[param.face_mapping[f][1]];
        const kwiver::arrows::core::tcoord_t& tcoord_2 = param.tcoords[param.face_mapping[f][2]];

        points[0].x = std::round(tcoord_0[0]);
        points[0].y = std::round(tcoord_0[1]);

        points[1].x = std::round(tcoord_1[0]);
        points[1].y = std::round(tcoord_1[1]);

        points[2].x = std::round(tcoord_2[0]);
        points[2].y = std::round(tcoord_2[1]);

        cv::Scalar random_color(rand() % 255, rand() % 255, rand() % 255);
        cv::polylines(image_param, points, true, random_color);
    }
    cv::imwrite("parameterization.png", image_param);

}


void test_rasterize_pinhole()
{
    kwiver::vital::plugin_manager::instance().load_all_plugins();

    std::string mesh_filename = ("/home/matthieu/data_cube_texture/cube.obj");
    std::string image_filename = ("/home/matthieu/data_cube_texture/images2/cam1.png");

    // mesh
    kwiver::arrows::core::mesh_io mesh_io;
    kwiver::vital::mesh_sptr mesh = mesh_io.load(mesh_filename);

    // uv parameterization
    kwiver::arrows::core::uv_parameterization_t param = kwiver::arrows::core::parameterize(mesh, 0.03, 800, 10, 5);

    // perspective camera
    Eigen::Vector3d center(-10, 0, -10);
    Eigen::Matrix< double, 3, 3 > R;
//    R << 1,   0,   0,
//         0,   1,   0,
//         0,   0,   1;
    R << 0.70711, 0.00000, 0.70711,
         0.00000, 1.00000, 0.00000,
         -0.70711, 0.00000, 0.70711;

    kwiver::vital::rotation_d orientation(R);
    kwiver::vital::camera_intrinsics_sptr camera_intrinsic(new kwiver::vital::simple_camera_intrinsics(1024, {480, 270}));
    kwiver::vital::camera_perspective_sptr camera(new kwiver::vital::simple_camera_perspective(center, orientation.inverse(), camera_intrinsic));


    // image
    kwiver::vital::algo::image_io_sptr image_io = kwiver::vital::algo::image_io::create("ocv");
    kwiver::vital::image_container_sptr image = image_io->load(image_filename);
    std::cout << "image depth " << image->depth() << std::endl;

    // id map
    kwiver::vital::image_container_sptr id_map = kwiver::arrows::core::generate_triangles_map(param, 5);

    // depthmap
    kwiver::arrows::core::compute_mesh_depthmap depthmap_generator;
    auto depthmaps = depthmap_generator.compute(mesh, camera, image->width(), image->height(), 0);
    // write depthmap
    cv::Mat depth_cv;
    kwiver::arrows::ocv::image_container_to_ocv_matrix(*depthmaps.first,  kwiver::arrows::ocv::image_container::OTHER_COLOR).copyTo(depth_cv);
    cv::Mat mask;
    cv::threshold(depth_cv, mask, std::numeric_limits<double>::max() / 2, 1, cv::THRESH_BINARY_INV);
    double min, max;
    mask.convertTo(mask, CV_8U);
    cv::minMaxLoc(depth_cv, &min, &max, 0, 0, mask);
    std::cout << "min max " << min << " " << max << std::endl;
    max *= 1.5;
    depth_cv -= min;
    depth_cv /= (max-min);
    depth_cv *= 255;
    cv::threshold(depth_cv, depth_cv, 255, 0, cv::THRESH_TRUNC);
    cv::normalize(depth_cv, depth_cv, 0, 255, cv::NORM_MINMAX);
    depth_cv.convertTo(depth_cv, CV_8U);
    cv::imwrite("depthmap.png", depth_cv);



    auto raster_result = kwiver::arrows::core::rasterize<unsigned char>(mesh, param, id_map, image, camera, depthmaps.first, {});
    auto texture = std::get<0>(raster_result);

    cv::Mat cv_tex = kwiver::arrows::ocv::image_container_to_ocv_matrix(*(texture.get()),
                                                                        kwiver::arrows::ocv::image_container::RGB_COLOR);
    cv_tex.convertTo(cv_tex, CV_8UC3);
    cv::cvtColor(cv_tex, cv_tex, CV_BGR2RGB);
    cv::imwrite("texture.png", cv_tex);
    mesh_io.save("test.obj", mesh, &param, texture.get());




    // project all points on image
//    kwiver::vital::mesh_vertex_array<3>& vertices = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(mesh->vertices());

//    std::vector<Eigen::Vector2d> points_uvs(mesh->num_verts());
//    int i=0;
//    for (auto pt3d : vertices)
//    {
//        points_uvs[i++] = camera->project(pt3d);
//    }
//    cv::Mat cv_image = kwiver::arrows::ocv::image_container_to_ocv_matrix(*(image.get()),
//                                                                          kwiver::arrows::ocv::image_container::RGB_COLOR);
//    cv::cvtColor(cv_image, cv_image, CV_BGR2RGB);
//    for (auto p: points_uvs)
//    {
//        cv_image.at<cv::Vec3b>(p[1], p[0]) = cv::Vec3b::all(255);
//    }
//    cv::imwrite("projected_points.png", cv_image);

}

std::pair<kwiver::vital::vector_3d, kwiver::vital::rotation_d>
read_camera(const std::string& filename)
{
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

void test_fuse_multi_pinhole_cameras()
{
    kwiver::vital::plugin_manager::instance().load_all_plugins();

    std::string mesh_filename = ("/home/matthieu/data_cube_texture/cube.obj");
    // mesh
    kwiver::arrows::core::mesh_io mesh_io;
    kwiver::vital::mesh_sptr mesh = mesh_io.load(mesh_filename);

    // uv parameterization
    kwiver::arrows::core::uv_parameterization_t param = kwiver::arrows::core::parameterize(mesh, 0.03, 800, 10, 5);
    // id map
    kwiver::vital::image_container_sptr id_map = kwiver::arrows::core::generate_triangles_map(param, 5);

    // perspective cameras
    kwiver::vital::camera_intrinsics_sptr camera_intrinsic(new kwiver::vital::simple_camera_intrinsics(1024, {480, 270}));
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

    // images
    kwiver::vital::image_container_sptr_list images;
    kwiver::vital::algo::image_io_sptr image_io = kwiver::vital::algo::image_io::create("ocv");
    images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam1.png"));
    images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam2.png"));
    images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam3.png"));
    images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam4.png"));
    images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam5.png"));
    images.push_back(image_io->load("/home/matthieu/data_cube_texture/images/cam6.png"));



    // depthmaps
    kwiver::arrows::core::compute_mesh_depthmap depthmap_generator;
    image_container_sptr_list depthmaps;
    int i=0;
    for (auto cam: cameras)
    {
        auto res = depthmap_generator.compute(mesh, cam, images[i]->width(), images[i]->height(), 0);
        depthmaps.push_back(res.first);
        i++;
    }

    std::vector< std::vector<float> > ratings;
    kwiver::arrows::core::compute_mesh_cameras_ratings(mesh, cameras, ratings);

    // rasterizations
    kwiver::vital::image_container_sptr_list textures, visibilities, scores;
    for (int i=0; i < cameras.size(); ++i)
    {
        auto raster_result = kwiver::arrows::core::rasterize<unsigned char>(mesh, param, id_map, images[i], cameras[i], depthmaps[i], ratings[i]);
        auto texture = std::get<0>(raster_result);
        textures.push_back(std::get<0>(raster_result));
        visibilities.push_back(std::get<1>(raster_result));
        scores.push_back(std::get<2>(raster_result));

        cv::Mat cv_tex = kwiver::arrows::ocv::image_container_to_ocv_matrix(*(texture.get()),
                                                                            kwiver::arrows::ocv::image_container::RGB_COLOR);
        cv_tex.convertTo(cv_tex, CV_8UC3);
        cv::cvtColor(cv_tex, cv_tex, CV_BGR2RGB);
        std::string output_name = "texture_" + std::to_string(i) + ".obj";
        cv::imwrite(output_name+".png", cv_tex);
        mesh_io.save(output_name, mesh, &param, texture.get());
    }

    // fusion
    auto fused = kwiver::arrows::core::fuse_texture_atlases<unsigned char>(textures, visibilities, scores, 0);
    cv::Mat cv_fused = kwiver::arrows::ocv::image_container_to_ocv_matrix(*(fused.get()), kwiver::arrows::ocv::image_container::RGB_COLOR);
    cv_fused.convertTo(cv_fused, CV_8UC3);
    cv::cvtColor(cv_fused, cv_fused, CV_BGR2RGB);
    std::string output_name = "texture_fused.obj";
    cv::imwrite(output_name + ".png", cv_fused);
    mesh_io.save(output_name, mesh, &param, fused.get());
}

void test_fuse_multi_rpc_cameras()
{
    kwiver::vital::plugin_manager::instance().load_all_plugins();

    // mesh
//    std::string mesh_filename = ("/media/matthieu/DATA/core3D-data/test_aoi4.obj");
//    std::string mesh_filename = ("/media/matthieu/DATA/core3D-data/AOI_D4_(Nick)/result_new/buildings.obj");
    std::string mesh_filename = ("/home/matthieu/Downloads/JIDO-Jacksonville-Model/Jacksonville_OBJ_Buildings_Only_offset_cropped.obj");
    kwiver::arrows::core::mesh_io mesh_io;
    kwiver::vital::mesh_sptr mesh = mesh_io.load(mesh_filename);
    kwiver::vital::mesh_vertex_array<3>& vertices = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(mesh->vertices());
    kwiver::vital::vector_3d mesh_offset = {435530.547508, 3354095.61004, -36.844062};
    for (int i=0; i < vertices.size(); ++i)
    {
        vertices[i] += mesh_offset;
    }


    // uv parameterization
    kwiver::arrows::core::uv_parameterization_t param = kwiver::arrows::core::parameterize(mesh, 0.3, 8000, 10, 5);
    // id map
    kwiver::vital::image_container_sptr id_map = kwiver::arrows::core::generate_triangles_map(param, 5);


    std::vector<std::string> images_filenames;
    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/27JAN15WV031100015JAN27160845-P1BS-500648062010_01_P001_________AAE_0AAAAABPABS0_pansharpen_8.tif");
    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/26APR15WV031200015APR26162435-P1BS-501504472050_01_P001_________AAE_0AAAAABPABR0_pansharpen_8.tif");
    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/15FEB15WV031200015FEB15161208-P1BS-500648061070_01_P001_________AAE_0AAAAABPABP0_pansharpen_8.tif");
    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/11OCT14WV031100014OCT11155720-P1BS-500648061020_01_P001_________AAE_0AAAAABPABR0_pansharpen_8.tif");
    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/08FEB15WV031100015FEB08160130-P1BS-500648061090_01_P001_________AAE_0AAAAABPABP0_pansharpen_8.tif");
    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/02MAY15WV031100015MAY02161943-P1BS-500648061030_01_P001_________AAE_0AAAAABPABR0_pansharpen_8.tif");
    images_filenames.push_back("/media/matthieu/DATA/core3D-data/AOI4/images/pansharpen/rescaled/01MAY15WV031200015MAY01160357-P1BS-500648062030_01_P001_________AAE_0AAAAABPABQ0_pansharpen_8.tif");

    // camera
    kwiver::vital::camera_sptr_list cameras;
    for (auto s: images_filenames)
    {
        cameras.push_back(load_camera_from_tif_image(s, 17));
    }

    // image
    kwiver::vital::image_container_sptr_list images;
    kwiver::vital::algo::image_io_sptr image_io = kwiver::vital::algo::image_io::create("gdal");
    for (auto s: images_filenames)
    {
        images.push_back(image_io->load(s));
    }



    // depthmaps
    kwiver::arrows::core::compute_mesh_depthmap depthmap_generator;
    image_container_sptr_list depthmaps;
    int i=0;
    for (auto cam: cameras)
    {
        auto res = depthmap_generator.compute(mesh, cam, images[i]->width(), images[i]->height(), 17);
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
        auto raster_result = kwiver::arrows::core::rasterize<unsigned short>(mesh, param, id_map, images[i], cameras[i], depthmaps[i], ratings[i]);
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
        mesh_io.save(output_name, mesh, &param, texture.get());
    }

    // remove offset
    for (int i=0; i < vertices.size(); ++i)
    {
        vertices[i] -= mesh_offset;
    }

    // fusion
    auto fused = kwiver::arrows::core::fuse_texture_atlases<unsigned short>(textures, visibilities, scores, 0);
    cv::Mat cv_fused = kwiver::arrows::ocv::image_container_to_ocv_matrix(*(fused.get()), kwiver::arrows::ocv::image_container::RGB_COLOR);
    cv_fused.convertTo(cv_fused, CV_8UC3);
    std::vector<cv::Mat> splitted;
    cv::split(cv_fused, splitted);
    std::vector<cv::Mat> rgb_splitted = {splitted[1], splitted[2], splitted[4]};
//        cv::cvtColor(cv_fused, cv_fused, CV_BGR2RGB);
    cv::merge(rgb_splitted, cv_fused);    std::string output_name = "texture_fused.obj";
    cv::imwrite(output_name + ".png", cv_fused);
    mesh_io.save(output_name, mesh, &param, fused.get());

}


void test_fuse_multi_pinhole_cameras2()
{
    kwiver::vital::plugin_manager::instance().load_all_plugins();

    std::string mesh_filename = ("/home/matthieu/data_plane/f16.obj");
    // mesh
    kwiver::arrows::core::mesh_io mesh_io;
    kwiver::vital::mesh_sptr mesh = mesh_io.load(mesh_filename);

    // uv parameterization
    kwiver::arrows::core::uv_parameterization_t param = kwiver::arrows::core::parameterize(mesh, 0.003, 8000, 10, 5);
    // id map
    kwiver::vital::image_container_sptr id_map = kwiver::arrows::core::generate_triangles_map(param, 5);

    // perspective cameras
    kwiver::vital::camera_intrinsics_sptr camera_intrinsic(new kwiver::vital::simple_camera_intrinsics(4480, {2048, 1536}));
    kwiver::vital::camera_sptr_list cameras;
    auto param_cam = read_camera("/home/matthieu/data_plane/cam1.txt");
    cameras.push_back(kwiver::vital::camera_sptr(new kwiver::vital::simple_camera_perspective(param_cam.first, param_cam.second.inverse(), camera_intrinsic)));
    param_cam = read_camera("/home/matthieu/data_plane/cam2.txt");
    cameras.push_back(kwiver::vital::camera_sptr(new kwiver::vital::simple_camera_perspective(param_cam.first, param_cam.second.inverse(), camera_intrinsic)));
    param_cam = read_camera("/home/matthieu/data_plane/cam3.txt");
    cameras.push_back(kwiver::vital::camera_sptr(new kwiver::vital::simple_camera_perspective(param_cam.first, param_cam.second.inverse(), camera_intrinsic)));

    // images
    kwiver::vital::image_container_sptr_list images;
    kwiver::vital::algo::image_io_sptr image_io = kwiver::vital::algo::image_io::create("ocv");
    images.push_back(image_io->load("/home/matthieu/data_plane/cam1.png"));
    images.push_back(image_io->load("/home/matthieu/data_plane/cam2.png"));
    images.push_back(image_io->load("/home/matthieu/data_plane/cam3.png"));



    // depthmaps
    kwiver::arrows::core::compute_mesh_depthmap depthmap_generator;
    image_container_sptr_list depthmaps;
    int i=0;
    for (auto cam: cameras)
    {
        auto res = depthmap_generator.compute(mesh, cam, images[i]->width(), images[i]->height(), 0);
        depthmaps.push_back(res.first);
        i++;
    }

    std::vector< std::vector<float> > ratings;
    kwiver::arrows::core::compute_mesh_cameras_ratings(mesh, cameras, ratings);

    // rasterizations
    kwiver::vital::image_container_sptr_list textures, visibilities, scores;
    for (int i=0; i < cameras.size(); ++i)
    {
        auto raster_result = kwiver::arrows::core::rasterize<unsigned char>(mesh, param, id_map, images[i], cameras[i], depthmaps[i], ratings[i]);
        auto texture = std::get<0>(raster_result);
        textures.push_back(std::get<0>(raster_result));
        visibilities.push_back(std::get<1>(raster_result));
        scores.push_back(std::get<2>(raster_result));

        cv::Mat cv_tex = kwiver::arrows::ocv::image_container_to_ocv_matrix(*(texture.get()),
                                                                            kwiver::arrows::ocv::image_container::RGB_COLOR);
        cv_tex.convertTo(cv_tex, CV_8UC3);
        cv::cvtColor(cv_tex, cv_tex, CV_BGR2RGB);
        std::string output_name = "texture_" + std::to_string(i) + ".obj";
        cv::imwrite(output_name+".png", cv_tex);
        mesh_io.save(output_name, mesh, &param, texture.get());
    }

    // fusion
    auto fused = kwiver::arrows::core::fuse_texture_atlases<unsigned char>(textures, visibilities, scores, 0);
    cv::Mat cv_fused = kwiver::arrows::ocv::image_container_to_ocv_matrix(*(fused.get()), kwiver::arrows::ocv::image_container::RGB_COLOR);
    cv_fused.convertTo(cv_fused, CV_8UC3);
    cv::cvtColor(cv_fused, cv_fused, CV_BGR2RGB);
    std::string output_name = "texture_fused.obj";
    cv::imwrite(output_name + ".png", cv_fused);
    mesh_io.save(output_name, mesh, &param, fused.get());

}
