#include "blur_image.h"

#include <arrows/ocv/image_container.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


namespace kwiver {
namespace arrows {
namespace ocv {

/// Constructor
blur_image::blur_image()
{
}

/// Destructor
blur_image::~blur_image()
{
}

/// Blur image
kwiver::vital::image_container_sptr blur_image::blur(vital::image_container_sptr img) const
{
    cv::Mat mat = ocv::image_container_to_ocv_matrix(*img.get(), ocv::image_container::RGB_COLOR);
    cv::Mat blurred;
    cv::GaussianBlur(mat, blurred, cv::Size(5, 5), 1.2);
    return kwiver::vital::image_container_sptr(new ocv::image_container(blurred, ocv::image_container::RGB_COLOR));
}

}
}
}
