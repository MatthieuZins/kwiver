#ifndef KWIVER_ARROWS_OCV_BLUR_IMAGE_H
#define KWIVER_ARROWS_OCV_BLUR_IMAGE_H

#include <arrows/ocv/kwiver_algo_ocv_export.h>
#include <vital/algo/blur_image.h>
#include <vital/algo/algorithm.h>
#include <vital/types/image_container.h>

namespace kwiver {
namespace arrows {
namespace ocv {

    class KWIVER_ALGO_OCV_EXPORT blur_image : public vital::algorithm_impl<blur_image, vital::algo::blur_image>
    {
    public:

        /// Constructor
        blur_image();

        /// Destructor
        virtual ~blur_image();

        virtual void set_configuration(kwiver::vital::config_block_sptr) {}
        virtual bool check_configuration(kwiver::vital::config_block_sptr) const { return true; }

        /// Blur image
        virtual kwiver::vital::image_container_sptr blur(kwiver::vital::image_container_sptr img) const;
    };

}
}
}
#endif // KWIVER_ARROWS_OCV_BLUR_IMAGE_H
