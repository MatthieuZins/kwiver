#ifndef KWIVER_ARROWS_CORE_MESH_IO_H
#define KWIVER_ARROWS_CORE_MESH_IO_H

#include <arrows/core/kwiver_algo_core_export.h>
#include <arrows/core/mesh_uv_parameterization.h>
#include <vital/algo/mesh_io.h>
#include <vital/types/image_container.h>
#include <vital/types/mesh.h>

namespace kwiver {
namespace arrows {
namespace core {


class KWIVER_ALGO_CORE_EXPORT mesh_io
    : public vital::algorithm_impl<mesh_io, vital::algo::mesh_io>
{
public:

    /// Name of the algorithm
    static constexpr char const* name = "core";

    /// Description of the algorithm
    static constexpr char const* description =
      "A reader / writer for OBJ mesh.";

    // No configuration for this class yet
    /// \cond DoxygenSuppress
    virtual void set_configuration(vital::config_block_sptr /*config*/) { }
    virtual bool check_configuration(vital::config_block_sptr /*config*/) const { return true; }
    /// \endcond


    mesh_io();

    virtual ~mesh_io() {}

    /**
     * @brief load an OBJ mesh (only vertices and faces)
     * @param filename
     * @return
     */
    virtual kwiver::vital::mesh_sptr load_(const std::string& filename) const;

    virtual void save_(const std::string& filename, vital::mesh_sptr data,
                       unsigned int tex_width=1, unsigned int tex_height=1,
                       bool flip_v_axis=false) const;
};

}
}
}
#endif // KWIVER_ARROWS_CORE_MESH_IO_H
