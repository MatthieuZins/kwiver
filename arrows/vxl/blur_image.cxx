#include "blur_image.h"

#include <arrows/vxl/image_container.h>

#include <vil/algo/vil_gauss_filter.h>
#include <vil/vil_copy.h>

namespace kwiver {
namespace arrows {
namespace vxl {

/// Constructor
blur_image::blur_image()
{
}

/// Destructor
blur_image::~blur_image()
{
}

vital::image_container_sptr blur_image::blur(vital::image_container_sptr img) const
{
    vil_image_view<vxl_byte> image = vxl::image_container::vital_to_vxl(img->get_image());
    vil_image_view<vxl_byte> blurred_copy, blurred;
    vil_gauss_filter_2d(image, blurred, 1.2, 3, 1.2, 3);
    vil_copy_deep(blurred, blurred_copy);

    return vital::image_container_sptr(new vxl::image_container(blurred_copy));
}


}
}
}
