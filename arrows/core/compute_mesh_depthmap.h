#ifndef KWIVER_ARROWS_CORE_COMPUTE_MESH_DEPTHMAP_H
#define KWIVER_ARROWS_CORE_COMPUTE_MESH_DEPTHMAP_H

#include <arrows/core/kwiver_algo_core_export.h>
#include <vital/types/camera.h>
#include <vital/algo/compute_mesh_depthmap.h>
#include <vital/types/image_container.h>
#include <vital/types/mesh.h>

namespace kwiver {
namespace arrows {
namespace core {

/// A class to compute depthmap from a mesh and a camera
class KWIVER_ALGO_CORE_EXPORT compute_mesh_depthmap
    : public vital::algorithm_impl<compute_mesh_depthmap, vital::algo::compute_mesh_depthmap>
{
public:
    /// Name of the algorithm
    static constexpr char const* name = "core";

    /// Description of the algorithm
    static constexpr char const* description = "Compute the depthmap of a mesh from a camera";

    // No configuration for this class yet
    /// \cond DoxygenSuppress
    virtual void set_configuration(vital::config_block_sptr /*config*/) { }
    virtual bool check_configuration(vital::config_block_sptr /*config*/) const { return true; }
    /// \endcond

    /// Constructor
    compute_mesh_depthmap();

    /// Destructor
    virtual ~compute_mesh_depthmap() {}

    virtual std::pair<kwiver::vital::image_container_sptr, kwiver::vital::image_container_sptr>
    compute(kwiver::vital::mesh_sptr mesh, kwiver::vital::camera_sptr camera, int width, int height, int utm_zone) const;
};

}
}
}
#endif // KWIVER_ARROWS_CORE_COMPUTE_MESH_DEPTHMAP_H
