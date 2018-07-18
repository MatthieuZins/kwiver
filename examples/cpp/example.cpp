#include <iostream>

#include "vital/plugin_loader/plugin_manager.h"
#include "vital/algo/image_io.h"
#include "vital/algo/blur_image.h"
#include "arrows/ocv/image_container.h"
#include "arrows/vxl/image_container.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <vil/algo/vil_gauss_filter.h>

void test()
{
    kwiver::vital::plugin_manager::instance().load_all_plugins();

    std::cout << "losc" << std::endl;
    kwiver::vital::algo::image_io_sptr ocv_io = kwiver::vital::algo::image_io::create("ocv");
    kwiver::vital::algo::image_io_sptr vxl_io = kwiver::vital::algo::image_io::create("vxl");

    kwiver::vital::image_container_sptr ocv_img = ocv_io->load("/home/matthieu/Lib/kwiver/examples/images/cat.jpg");
    kwiver::vital::image_container_sptr vxl_img = vxl_io->load("/home/matthieu/Lib/kwiver/examples/images/cat.jpg");


    kwiver::vital::algo::blur_image_sptr ocv_blur = kwiver::vital::algo::blur_image::create("ocv");
    kwiver::vital::algo::blur_image_sptr vxl_blur = kwiver::vital::algo::blur_image::create("vxl");

    cv::Mat mat = kwiver::arrows::ocv::image_container_to_ocv_matrix(*ocv_img.get(), kwiver::arrows::ocv::image_container::RGB_COLOR);
    cv::namedWindow("window");
    cv::imshow("window", mat);
    cv::waitKey(0);


    kwiver::vital::image_container_sptr blurred = ocv_blur->blur(vxl_img);
    cv::Mat filtered;
    filtered = kwiver::arrows::ocv::image_container_to_ocv_matrix(*blurred.get(), kwiver::arrows::ocv::image_container::RGB_COLOR);
    cv::imshow("window", filtered);
    cv::waitKey(0);


    kwiver::vital::image_container_sptr blurred_vxl = vxl_blur->blur(vxl_img);
    filtered = kwiver::arrows::ocv::image_container::vital_to_ocv(blurred_vxl->get_image(), kwiver::arrows::ocv::image_container::RGB_COLOR);
    cv::imshow("window", filtered);
    cv::waitKey(0);

    cv::destroyAllWindows();


//    vil_image_view<vxl_byte> image = kwiver::arrows::vxl::image_container::vital_to_vxl(ocv_img->get_image());
//    vil_image_view<vxl_byte> blurred2;
//    vil_gauss_filter_2d(image, blurred2, 1.2, 3, 1.2, 3);



}
