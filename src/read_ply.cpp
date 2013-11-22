#include "sr_grasp_mesh_planner/read_ply.hpp"

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <string>

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
    printf("File does not have a .PLY extension. ");
    return -1;
  }

  FILE* file = fopen(filename,"r");

  try
  {
    Vertex_Buffer = (float*) malloc (ftell(file));
  }
  catch (char* )
  {
    return -1;
  }
  if (Vertex_Buffer == NULL) return -1;
  fseek(file,0,SEEK_SET);

  if (file)
  {
    int i = 0;
    int temp = 0;
    int quads_index = 0;
    int triangle_index = 0;
    int normal_index = 0;
    char buffer[1000];

    fgets(buffer,300,file); // ply

    // READ HEADER
    // -----------------

    // Find number of vertexes
    while (  strncmp( "element vertex", buffer,strlen("element vertex")) != 0  )
    {
      fgets(buffer,300,file); // format
    }
    strcpy(buffer, buffer+strlen("element vertex"));
    sscanf(buffer,"%i", &this->total_vertices_);

    // Find number of vertexes
    while (  strncmp( "element face", buffer,strlen("element face")) != 0  )
    {
      fgets(buffer,300,file); // format
    }
    strcpy(buffer, buffer+strlen("element face"));
    sscanf(buffer,"%i", &this->total_triangles_);

    // go to end_header
    while (  strncmp( "end_header", buffer,strlen("end_header")) != 0  )
    {
      fgets(buffer,300,file); // format
    }

    //----------------------

    // read verteces
    i = 0;
    for (int iterator = 0; iterator < this->total_vertices_; iterator++)
    {
      fgets(buffer,300,file);

      PlyVertex v;
      sscanf(buffer,"%f %f %f", &v.x, &v.y, &v.z);
      vertices_.push_back(v);

      i += 3;
    }

    // read faces
    i =0;
    for (int iterator = 0; iterator < this->total_triangles_; iterator++)
    {
      fgets(buffer,300,file);

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
  }
  else
  {
    printf("File can't be opened\n");
  }

  return 0;
}


