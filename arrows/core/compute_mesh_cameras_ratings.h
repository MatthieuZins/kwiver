#ifndef KWIVER_ARROWS_CORE_COMPUTE_MESH_CAMERAS_RATINGS_H
#define KWIVER_ARROWS_CORE_COMPUTE_MESH_CAMERAS_RATINGS_H

#include <arrows/core/kwiver_algo_core_export.h>
#include <vital/types/camera.h>
#include <vital/types/mesh.h>

namespace kwiver {
namespace arrows {
namespace core {

KWIVER_ALGO_CORE_EXPORT
void compute_mesh_cameras_ratings(kwiver::vital::mesh_sptr mesh, const kwiver::vital::camera_sptr_list& cameras,
                                  std::vector<std::vector<float> > &ratings);

}
}
}
#endif // KWIVER_ARROWS_CORE_COMPUTE_MESH_CAMERAS_RATINGS_H
