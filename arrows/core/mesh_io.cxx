#include "mesh_io.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>


using namespace kwiver::vital;

namespace kwiver {
namespace arrows {
namespace core {

mesh_sptr mesh_io::load(const std::string &filename) const
{
    std::vector<vector_3d> verts;
    std::vector<mesh_regular_face<3> > faces;
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
            // check that the input mesh is the simplest version with only 3 vertices per face
            // (no normals, nor texture coordinates)
            if (line.find('/') == std::string::npos)
            {
                std::stringstream extractor(line.substr(1));
                unsigned int v1, v2, v3;
                extractor >> v1 >> v2 >> v3;
                faces.push_back(mesh_regular_face<3>({v1-1, v2-1, v3-1}));
            }
            else
            {
                std::cerr << "Unhandled format" << std::endl;
            }
        }
    }
    std::unique_ptr<mesh_vertex_array_base> vertices_array_ptr(new mesh_vertex_array<3>(verts));
    std::unique_ptr<mesh_face_array_base> faces_array_ptr(new mesh_regular_face_array<3>(faces));
    mesh_sptr mesh(new kwiver::vital::mesh(std::move(vertices_array_ptr), std::move(faces_array_ptr)));
    return mesh;
}


void mesh_io::save(const std::string &filename, mesh_sptr mesh,
                   const uv_parameterization_t *tcoords, vector_2i texture_size) const
{
    unsigned int nb_faces = mesh->num_faces();
    if (tcoords && tcoords->face_mapping.size() != nb_faces)
    {
        std::cerr << "Warning: texture coordinates are ignored (not the correct number of faces)" << std::endl;
        tcoords = nullptr;
    }

    mesh_regular_face_array<3>& faces = dynamic_cast< mesh_regular_face_array<3>& >(mesh->faces());
    mesh_vertex_array<3>& vertices = dynamic_cast< mesh_vertex_array<3>& >(mesh->vertices());

    std::ofstream file(filename, std::ios_base::out);
    file << "# Mesh generated with Kwiver\n";
    file << "mtllib " << filename << ".mtl\n";

    int nb_vertices=0;
    for (auto vert: vertices)
    {
        file << std::setprecision(15) <<  "v " << vert[0] << " " << vert[1] << " " << vert[2] << std::endl;
        nb_vertices++;
    }
    std::cout << "write " << nb_vertices << " vertices " << std::endl;
    if (tcoords)
    {
        for (auto tcoord: tcoords->tcoords)
        {
            file << std::setprecision(15) << "vt " << (tcoord[0])/texture_size[0]
                 << " " << 1.0 - (tcoord[1]/texture_size[1]) << std::endl;
        }
    }
    file << "usemtl mat\n";

    for (unsigned int f_id=0; f_id < nb_faces; ++f_id)
    {
        if (tcoords)
        {
            file << std::setprecision(15) << "f " << faces(f_id, 0)+1  << "/" << tcoords->face_mapping[f_id][0]+1 << " "
                 << faces(f_id, 1)+1 << "/" << tcoords->face_mapping[f_id][1]+1 << " "
                 << faces(f_id, 2)+1 << "/" << tcoords->face_mapping[f_id][2]+1 << std::endl;
        }
        else
        {
            file << std::setprecision(15) << "f " << faces(f_id, 0)+1 << " " << faces(f_id, 1)+1 << " " << faces(f_id, 2)+1 << std::endl;
        }
    }
    file.close();


    // Write material file
    std::ofstream mtl_file(filename + ".mtl");
    mtl_file << "newmtl mat\n";
    mtl_file << "Ka 1.0 1.0 1.0\n";
    mtl_file << "Kd 1.0 1.0 1.0\n";
    mtl_file << "Ks 1 1 1\n";
    mtl_file << "d 1\n";
    mtl_file << "Ns 75\n";
    mtl_file << "illum 1\n";
    mtl_file << "map_Kd " << filename << ".png\n";
    mtl_file.close();
}


}
}
}
