#ifndef KWIVER_ARROWS_CORE_ATLAS_PROCESSING_H
#define KWIVER_ARROWS_CORE_ATLAS_PROCESSING_H

#include <arrows/core/kwiver_algo_core_export.h>
#include <arrows/core/mesh_uv_parameterization.h>
#include <vital/types/image_container.h>
#include <vital/types/mesh.h>
#include <vital/types/camera.h>

namespace kwiver {
namespace arrows {
namespace core {

KWIVER_ALGO_CORE_EXPORT
bool is_point_inside_triangle(const Eigen::Vector2d& p,
                              const Eigen::Vector2d& a,
                              const Eigen::Vector2d& b,
                              const Eigen::Vector2d& c);
KWIVER_ALGO_CORE_EXPORT
Eigen::Vector3d barycentric_coordinates(const Eigen::Vector2d& p,
                                        const Eigen::Vector2d& a,
                                        const Eigen::Vector2d& b,
                                        const Eigen::Vector2d& c);
KWIVER_ALGO_CORE_EXPORT
kwiver::vital::image_container_sptr generate_triangles_map(const kwiver::arrows::core::uv_parameterization_t& param, int exterior_margin);

KWIVER_ALGO_CORE_EXPORT
kwiver::vital::image_container_sptr rasterize(kwiver::vital::mesh_sptr mesh,
                                              const kwiver::arrows::core::uv_parameterization_t& param,
                                              vital::image_container_sptr triangles_id_map,
                                              kwiver::vital::image_container_sptr image,
                                              vital::camera_sptr camera,
                                              kwiver::vital::image_container_sptr depthmap);

}
}
}


#endif // KWIVER_ARROWS_CORE_ATLAS_PROCESSING_H
