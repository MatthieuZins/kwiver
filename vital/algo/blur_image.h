#ifndef VITAL_ALGO_BLUR_IMAGE_H_
#define VITAL_ALGO_BLUR_IMAGE_H_

#include <vital/vital_config.h>
#include <vital/algo/algorithm.h>
#include <vital/types/image_container.h>

namespace kwiver {
namespace vital {
namespace algo {

class VITAL_ALGO_EXPORT blur_image: public kwiver::vital::algorithm_def<blur_image>
{
public:
    /// Return the name of this algorithm
    static std::string static_type_name() { return "blur_image"; }

    /// Blur image
    virtual kwiver::vital::image_container_sptr blur(kwiver::vital::image_container_sptr img) const = 0;

protected:
    blur_image();
};

typedef std::shared_ptr<blur_image> blur_image_sptr;

}
}
}
#endif // VITAL_ALGO_BLUR_IMAGE_H_
