#ifndef KWIVER_ARROWS_CORE_MESH_IO_H
#define KWIVER_ARROWS_CORE_MESH_IO_H

#include <arrows/core/kwiver_algo_core_export.h>
#include <vital/types/mesh.h>

namespace kwiver {
namespace arrows {
namespace core {


class KWIVER_ALGO_CORE_EXPORT mesh_io
{
public:

    virtual ~mesh_io() {}

    /**
     * @brief load a simple version of OBJ mesh (only vertices and faces)
     * @param filename
     * @return
     */
    virtual vital::mesh_sptr load(const std::string& filename) const;

    virtual void save(const std::string& filename, vital::mesh_sptr data) const;

};

}
}
}
#endif // KWIVER_ARROWS_CORE_MESH_IO_H
