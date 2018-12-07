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
#include "vital/algo/split_image.h"

#include "vital/plugin_loader/plugin_manager.h"

// We will be calling some OpenCV code, so we need to include
// some OpenCV related files
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "arrows/ocv/image_container.h"

#include <kwiversys/SystemTools.hxx>
#include <vital/types/camera_map.h>
#include <vital/types/camera_perspective.h>
#include <vital/io/camera_io.h>
#include <vital/io/landmark_map_io.h>
#include <vital/types/image_container.h>
#include <vital/algo/image_io.h>
#include <kwiversys/SystemTools.hxx>
#include <fstream>
#include <vector>
#include <string>

#include <vital/algo/compute_depth.h>
// Predefine methods that show off various functionality in kwiver
void how_to_part_01_images();
void how_to_part_02_detections();

using namespace kwiver::vital;
int main()
{
  kwiver::vital::plugin_manager::instance().load_all_plugins();

  // use comments to execute a particular method

  std::string current_dir = "/media/matthieu/DATA/MAPTk++/test_bundle/";
  std::string krtd_dir = "output/krtd/";
  std::string frames_dir = "frames/";

  std::ifstream frames_file(current_dir + "frames_list.txt");


  kwiver::vital::algo::image_io_sptr ocv_io = kwiver::vital::algo::image_io::create("ocv");

  std::vector<image_container_sptr> images;

  std::string line;
  std::vector<camera_perspective_sptr> cameras;
  int first_image_to_use = 0;
  int nb_images_to_use = 40;
  int depth_image_index = first_image_to_use + nb_images_to_use / 2;
  int i=0;
  while(std::getline(frames_file, line))
  {
    if (i >= first_image_to_use && i < first_image_to_use + nb_images_to_use)
    {
      std::string name = kwiversys::SystemTools::GetFilenameWithoutExtension(line);
      std::cout << name << std::endl;
      cameras.push_back(read_krtd_file(current_dir + krtd_dir + name + ".krtd"));
      images.push_back(ocv_io->load(current_dir + frames_dir + name + ".png"));
    }
    i++;
  }

   std::cout << "nb images: " << cameras.size() << std::endl;

  landmark_map_sptr landmarks_m = read_ply_file(current_dir + "output/landmarks.ply");
  std::vector<landmark_sptr> landmarks;
  for (auto iter : landmarks_m->landmarks())
  {
    landmarks.push_back(iter.second);
  }

  frames_file.close();

  std::cout << "All data loaded" << std::endl;

  algo::compute_depth_sptr compute_depth = algo::compute_depth::create("super3d");
  auto depth_map = compute_depth->compute(images, cameras, landmarks, depth_image_index);

  ocv_io->save("depth_map.png", depth_map);


  cv::Mat mat = kwiver::arrows::ocv::image_container::vital_to_ocv(depth_map->get_image(), kwiver::arrows::ocv::image_container::OTHER_COLOR);

  std::ofstream out_file("depth_map.txt");
  for (int y = 0; y < mat.rows; ++y)
  {
    for (int x = 0;  x < mat.cols; ++x)
    {
      out_file << mat.at<double>(y, x) << " ";
    }
    out_file << "\n";
  }
  out_file.close();

  double min, max;
  cv::minMaxLoc(mat, &min, &max, 0, 0);
  std::cout << "min " << min << std::endl;
  std::cout << "max " << max << std::endl;
  mat -= min;
  mat /= (max-min);
  mat *= 255;
  cv::Mat mat_uint8;
  mat.convertTo(mat_uint8, CV_8U);
  cv::imwrite("depth_map_uin8.png", mat);
}
