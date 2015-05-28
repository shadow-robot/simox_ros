/*
 * Copyright (c) 2013 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

/**
 * @file   read_ply.hpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  A simple PLY reader.
 **/

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


