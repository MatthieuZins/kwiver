#include <vital/algo/algorithm.txx>

#include "blur_image.h"

INSTANTIATE_ALGORITHM_DEF(kwiver::vital::algo::blur_image);

namespace kwiver {
namespace vital {
namespace algo {

blur_image::blur_image()
{
    attach_logger("blur_image");
}

}
}
}
