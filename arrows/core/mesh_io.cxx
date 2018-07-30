#include "mesh_io.h"

#include <fstream>
#include <iostream>
#include <sstream>

kwiver::vital::mesh_sptr kwiver::arrows::core::mesh_io::load(const std::string &filename) const
{
    std::vector<Eigen::Vector3d> verts;
    std::vector< kwiver::vital::mesh_regular_face<3> > faces;
    std::ifstream input_file(filename);
    std::string line;
    while (std::getline(input_file, line))
    {
        if (line[0] == '#')
        {
            continue;
        }
        else if (line[0] == 'v')
        {
            std::stringstream extractor(line.substr(1));
            double x, y, z;
            extractor >> x >> y >> z;
            verts.push_back({x, y, z});
        }
        else if (line[0] == 'f')
        {
            // check that the imput mesh is the simplest version with only 3 vertices per face
            // (no normals nor texture coordinates)
            if (line.find('/') == std::string::npos)
            {
                std::stringstream extractor(line.substr(1));
                unsigned int v1, v2, v3;
                extractor >> v1 >> v2 >> v3;
                faces.push_back(kwiver::vital::mesh_regular_face<3>({v1-1, v2-1, v3-1}));
            }
            else
            {
                std::cerr << "Unhandled format" << std::endl;
            }
        }
    }
    std::unique_ptr<kwiver::vital::mesh_vertex_array_base> vertices_array_ptr(new kwiver::vital::mesh_vertex_array<3>(verts));
    std::unique_ptr<kwiver::vital::mesh_face_array_base> faces_array_ptr(new kwiver::vital::mesh_regular_face_array<3>(faces));
    kwiver::vital::mesh_sptr mesh(new kwiver::vital::mesh(std::move(vertices_array_ptr),
                                                          std::move(faces_array_ptr)));
    return mesh;
}

void kwiver::arrows::core::mesh_io::save(const std::string &filename, kwiver::vital::mesh_sptr data) const
{
    std::cerr << "mesh_io::save not yet implemented" << std::endl;
}
