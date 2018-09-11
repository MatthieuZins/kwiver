#include "compute_mesh_cameras_ratings.h"

#include <vital/types/camera_rpc.h>
#include <vital/types/geodesy.h>


using namespace kwiver::vital;

namespace kwiver {
namespace arrows {
namespace core {


void compute_mesh_cameras_ratings(mesh_sptr mesh, const camera_sptr_list& cameras,
                                  std::vector<std::vector<float> > &ratings)
{
    unsigned int nb_faces = mesh->num_faces();
    unsigned int nb_vertices = mesh->num_verts();
    unsigned int nb_cameras = cameras.size();

    // resize output scores
    ratings.resize(nb_cameras, std::vector<float>(nb_faces, 0.0));

    kwiver::vital::mesh_vertex_array<3>& vertices = dynamic_cast< kwiver::vital::mesh_vertex_array<3>& >(mesh->vertices());
    vital::mesh_regular_face_array<3>& faces = dynamic_cast< vital::mesh_regular_face_array<3>& >(mesh->faces());


    std::vector< vector_2d> latlong_vertices;
    for (unsigned int cam_id=0; cam_id < nb_cameras; ++cam_id)
    {
        // project all points on image
        std::vector<vector_2d> points_uvs(nb_vertices);
        if (dynamic_cast<camera_rpc*>(cameras[cam_id].get()))
        {
            // RPC cameras expect lat/long points, we do the conversion only once
            unsigned int utm_zone = dynamic_cast<camera_rpc*>(cameras[cam_id].get())->get_utm_zone();
            if (latlong_vertices.size() != nb_vertices)
            {
                latlong_vertices.resize(nb_vertices);
                for (unsigned int i=0; i < nb_vertices; ++i)
                {
                    latlong_vertices[i] = vital::geo_conv({vertices[i][0], vertices[i][1]},
                                                          kwiver::vital::SRID::UTM_WGS84_north + utm_zone,
                                                          kwiver::vital::SRID::lat_lon_WGS84);
                }
            }
            for (unsigned int i=0; i < nb_vertices; ++i)
            {
                points_uvs[i] = cameras[cam_id]->project({latlong_vertices[i][0],
                                                          latlong_vertices[i][1],
                                                          vertices[i][2]});
            }
        }
        else
        {
            int i=0;
            for (auto pt3d : vertices)
            {
                points_uvs[i++] = cameras[cam_id]->project(pt3d);
            }
        }


        for (unsigned int f_id=0; f_id < nb_faces; ++f_id)
        {
            vital::mesh_regular_face<3> f = faces[f_id];
            const vector_2d& a_uv = points_uvs[f[0]];
            const vector_2d& b_uv = points_uvs[f[1]];
            const vector_2d& c_uv = points_uvs[f[2]];

            // TODO  check that the projected triangle is inside the image

            // compute the area (in sq. pixels) of the projection
            vector_2d ab = b_uv - a_uv;
            vector_2d ac = c_uv - a_uv;
            ratings[cam_id][f_id] = static_cast<float>(std::max(0.0, -(ab[0] * ac[1] - ab[1] * ac[0])/ 2.0));
        }
    }
}

}
}
}
