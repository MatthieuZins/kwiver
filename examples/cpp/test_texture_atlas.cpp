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

#include <vital/types/image_container.h>

#include <opencv2/opencv_modules.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <arrows/ocv/image_container.h>
#include <arrows/core/mesh_uv_parameterization.h>
#include <arrows/core/mesh_io.h>

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

    param = kwiver::arrows::core::parameterize(mesh, 0.25, 8000, 5, 10);
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
