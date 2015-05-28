/*
 * Copyright (c) 2013 Shadow Robot Company Ltd.
 * All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 * prior authorisation and agreement from Shadow Robot Company Ltd.
 */

/**
 * @file   read_ply.cpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  A simple PLY reader.
 **/

#include "sr_grasp_mesh_planner/read_ply.hpp"

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <string>
#include <ros/ros.h>

using namespace sr_grasp_mesh_planner;

ReadPLY::ReadPLY()
{
}

int ReadPLY::load(const char* filename)
{
  triangles_.clear();
  vertices_.clear();

  const char* pch = strstr(filename,".ply");
  if (pch == NULL)
  {
    ROS_ERROR_STREAM("File " << filename << " does not have a .PLY extension.");
    return -1;
  }

  FILE* file = fopen(filename,"r");

  try
  {
    Vertex_Buffer = (float*) malloc (ftell(file));
  }
  catch (char* )
  {
    ROS_DEBUG_STREAM("Function malloc failed in ReadPLY::load.");
    return -1;
  }
  if (Vertex_Buffer == NULL)
  {
    ROS_DEBUG_STREAM("Vertex_Buffer is NULL in ReadPLY::load.");
    return -1;
  }
  fseek(file,0,SEEK_SET);

  if (!file)
  {
    ROS_ERROR_STREAM("File " << filename << "can't be opened.\n");
    return -1;
  }

  int i = 0;
  int temp = 0;
  int quads_index = 0;
  int triangle_index = 0;
  int normal_index = 0;
  char buffer[1000];

  // PLY
  if (fgets(buffer,300,file) == NULL)
  {
    ROS_ERROR_STREAM("Failed to read file " << filename << ".");
    return -1;
  }

  // READ HEADER

  // Find number of vertexes
  while ( strncmp( "element vertex", buffer,strlen("element vertex")) != 0 )
  {
    // format
    if (fgets(buffer,300,file) == NULL)
    {
      ROS_ERROR_STREAM("Failed to read file " << filename << ".");
      return -1;
    }
  }
  strcpy(buffer, buffer+strlen("element vertex"));
  sscanf(buffer,"%i", &this->total_vertices_);

  // Find number of vertexes
  while ( strncmp( "element face", buffer,strlen("element face")) != 0 )
  {
    // format
    if (fgets(buffer,300,file) == NULL)
    {
      ROS_ERROR_STREAM("Failed to read file " << filename << ".");
      return -1;
    }
  }
  strcpy(buffer, buffer+strlen("element face"));
  sscanf(buffer,"%i", &this->total_triangles_);

  // go to end_header
  while ( strncmp( "end_header", buffer,strlen("end_header")) != 0 )
  {
    // format
    if (fgets(buffer,300,file) == NULL)
    {
      ROS_ERROR_STREAM("Failed to read file " << filename << ".");
      return -1;
    }
  }

  // read vertices
  i = 0;
  for (int iterator = 0; iterator < this->total_vertices_; iterator++)
  {
    if (fgets(buffer,300,file) == NULL)
    {
      ROS_ERROR_STREAM("Failed to read file " << filename << ".");
      return -1;
    }

    PlyVertex v;
    sscanf(buffer,"%f %f %f", &v.x, &v.y, &v.z);
    vertices_.push_back(v);

    i += 3;
  }

  // read faces
  i = 0;
  for (int iterator = 0; iterator < this->total_triangles_; iterator++)
  {
    if (fgets(buffer,300,file) == NULL)
    {
      ROS_ERROR_STREAM("Failed to read file " << filename << ".");
      return -1;
    }

    if (buffer[0] == '3')
    {
      PlyTriangle t;
      buffer[0] = ' ';
      sscanf(buffer,"%i%i%i", &t.n1, &t.n2, &t.n3);
      triangles_.push_back(t);
    }

    i += 3;
  }

  fclose(file);

  return 0;
}
