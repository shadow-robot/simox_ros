#pragma once

#include <vector>

//-------------------------------------------------------------------------------

namespace sr_grasp_mesh_planner
{

class ReadPLY
{
public:
  ReadPLY();

  int load(const char *filename);

  struct PlyVertex{
    float x;
    float y;
    float z;
  };

  struct PlyTriangle{
    int n1;
    int n2;
    int n3;
  };

  std::vector<PlyVertex>   vertices_;
  std::vector<PlyTriangle> triangles_;

  int total_vertices_;
  int total_triangles_;

  // Yi Li: Do NOT remove! Parsing will fail if removed.
  float* Vertex_Buffer;
};

} // end of namespace sr_grasp_mesh_planner

//-------------------------------------------------------------------------------


