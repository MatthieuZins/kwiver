#include "mesh_io.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>


using namespace kwiver::vital;

namespace kwiver {
namespace arrows {
namespace core {

mesh_io::mesh_io()
{
  attach_logger("algo.mesh_io");
}

mesh_sptr mesh_io::load_(const std::string &filename) const
{
  std::vector<vector_3d> verts;
  std::vector<mesh_regular_face<3> > faces;
  std::vector<vector_2d> tcoords;
  std::vector<vector_3d> normals;
  std::vector<Eigen::Vector3i> faces_tcoords_ids;
  std::vector<Eigen::Vector3i> faces_normals_ids;

  std::ifstream input_file(filename);
  std::string line;
  while (std::getline(input_file, line))
  {
    if (line[0] == '#')
    {
      // comments
      continue;
    }
    else if (line[0] == 'v' && line[1] == ' ')
    {
      // vertices
      std::stringstream extractor(line.substr(2));
      double x, y, z;
      extractor >> x >> y >> z;
      verts.push_back({x, y, z});
    }
    else if (line[0] == 'f')
    {
      // faces
      std::stringstream extractor(line.substr(2));
      int vertices_ids[3] = {0, 0, 0};
      int tcoords_ids[3] = {0, 0, 0};
      int normals_ids[3] = {0, 0, 0};
      bool has_tcoords = false;
      bool has_normals = false;
      for (int i=0; i <3; ++i)
      {
        std::string v_attrib;
        extractor >> v_attrib ;
        int first_sep = v_attrib.find_first_of('/');
        int last_sep = v_attrib.find_last_of('/');
        std::replace(v_attrib.begin(), v_attrib.end(), '/', ' ');
        std::stringstream attribute_extractor(v_attrib);
        attribute_extractor >> vertices_ids[i];
        --vertices_ids[i];
        if (first_sep != std::string::npos && last_sep != std::string::npos)
        {
          if (first_sep == last_sep || last_sep - first_sep > 1)
          {
            attribute_extractor >> tcoords_ids[i];
            --tcoords_ids[i];
            has_tcoords = true;
          }
          if (first_sep != last_sep)
          {
            attribute_extractor >> normals_ids[i];
            --normals_ids[i];
            has_normals = true;
          }
        }
      }
      faces.push_back(mesh_regular_face<3>({static_cast<unsigned int>(vertices_ids[0]),
                                            static_cast<unsigned int>(vertices_ids[1]),
                                            static_cast<unsigned int>(vertices_ids[2])}));
      if (has_tcoords)
      {
        faces_tcoords_ids.push_back(Eigen::Map<Eigen::Vector3i>(tcoords_ids));
      }
      if (has_normals)
      {
        faces_normals_ids.push_back(Eigen::Map<Eigen::Vector3i>(normals_ids));
      }
    }
    else if (line[0] == 'v' && line[1] == 't')
    {
      // tex coords
      std::stringstream extractor(line.substr(2));
      double x, y;
      extractor >> x >> y;
      tcoords.push_back({x, y});
    }
    else if (line[0] == 'v' && line[1] == 'n')
    {
      // vertex normals
      std::stringstream extractor(line.substr(2));
      double x, y, z;
      extractor >> x >> y >> z;
      normals.push_back({x, y, z});
    }
  }
  std::unique_ptr<mesh_vertex_array_base> vertices_array_ptr(new mesh_vertex_array<3>(verts));
  std::unique_ptr<mesh_face_array_base> faces_array_ptr(new mesh_regular_face_array<3>(faces));

  // sort tcoords by face: [vt1_face1, vt2_face1, vt3_face1, vt1_face2, ...]
  std::vector<vector_2d> sorted_tcoords(faces_tcoords_ids.size() * 3);
  for (int i=0; i < faces_tcoords_ids.size(); ++i)
  {
    sorted_tcoords[i * 3 + 0] = tcoords[faces_tcoords_ids[i][0]];
    sorted_tcoords[i * 3 + 1] = tcoords[faces_tcoords_ids[i][1]];
    sorted_tcoords[i * 3 + 2] = tcoords[faces_tcoords_ids[i][2]];
  }

  // average the vertices normals for each face
  std::vector<vector_3d> faces_normals(faces_normals_ids.size());
  for (int i=0; i < faces_normals_ids.size(); i++)
  {
    auto n1 = normals[faces_normals_ids[i][0]];
    auto n2 = normals[faces_normals_ids[i][1]];
    auto n3 = normals[faces_normals_ids[i][2]];
    vector_3d avg_normal = n1 + n2 + n3;
    avg_normal.normalize();
    faces_normals[i] = avg_normal;
  }

  mesh_sptr mesh(new kwiver::vital::mesh(std::move(vertices_array_ptr), std::move(faces_array_ptr)));
  mesh->set_tex_coords(sorted_tcoords);
  mesh->faces().set_normals(faces_normals);

  return mesh;
}


void mesh_io::save_(const std::string &filename, mesh_sptr mesh,
                    unsigned int tex_width, unsigned int tex_height, bool flip_v_axis) const
{
  unsigned int nb_faces = mesh->num_faces();

  mesh_regular_face_array<3>& faces = dynamic_cast< mesh_regular_face_array<3>& >(mesh->faces());
  mesh_vertex_array<3>& vertices = dynamic_cast< mesh_vertex_array<3>& >(mesh->vertices());
  const std::vector<vector_2d>& tcoords = mesh->tex_coords();

  std::ofstream file(filename, std::ios_base::out);
  file << "# Mesh generated with Kwiver\n";
  if (mesh->has_tex_coords())
  {
    file << "mtllib " << filename << ".mtl\n";
  }

  int nb_vertices=0;
  // vertices
  for (auto vert: vertices)
  {
    file << std::setprecision(15) <<  "v " << vert[0] << " " << vert[1] << " " << vert[2] << std::endl;
    nb_vertices++;
  }

  // normals
  if (mesh->faces().has_normals())
  {
    auto normals = mesh->faces().normals();
    for (auto n: normals)
    {
      file << std::setprecision(15) << "vn " << n[0] << " " << n[1] << " " << n[2] << std::endl;
    }
  }

  // tex coords
  if (mesh->has_tex_coords())
  {
    for (auto tcoord: tcoords)
    {
      file << std::setprecision(15) << "vt " << (tcoord[0])/tex_width
          << " " << (flip_v_axis ? 1.0 - (tcoord[1]/tex_height) : tcoord[1]/tex_height)
          << std::endl;
    }
  }
  if (mesh->has_tex_coords())
  {
    file << "usemtl mat\n";
  }

  // faces
  for (unsigned int f_id=0; f_id < nb_faces; ++f_id)
  {
    file << std::setprecision(15) << "f ";
    for (int k=0; k < 3; ++k)
    {
      file << faces(f_id, k) + 1;
      if (mesh->has_tex_coords())
      {
        file << "/" << f_id * 3 + k + 1;
      }
      if (mesh->faces().has_normals())
      {
        if (!mesh->has_tex_coords())
        {
          file << "/";
        }
        file << "/" << f_id + 1;
      }
      file << " ";
    }
    file << std::endl;
  }
  file.close();

  if (mesh->has_tex_coords())
  {
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
}
