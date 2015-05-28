/*
 * Copyright (c) 2013 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

/**
 * @file   mesh_obstacle.cpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  Mesh obstacle.
 **/

#include "sr_grasp_mesh_planner/mesh_obstacle.hpp"

#include <string>
#include <iostream>
#include <boost/assign/list_of.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

//-------------------------------------------------------------------------------

using namespace sr_grasp_mesh_planner;

//-------------------------------------------------------------------------------

const float MeshObstacle::SKY_BOX_SIZE2_ = 0.050; // Meter

//-------------------------------------------------------------------------------

MeshObstacle::MeshObstacle(const std::string &name,
                           VisualizationNodePtr visualization,
                           CollisionModelPtr collisionModel,
                           const SceneObject::Physics &p,
                           CollisionCheckerPtr colChecker)
  : Obstacle(name, visualization, collisionModel, p, colChecker)
{
}

//-------------------------------------------------------------------------------

MeshObstacle::~MeshObstacle()
{
}

//-------------------------------------------------------------------------------

ObstaclePtr MeshObstacle::create_mesh_obstacle(TriMeshModelPtr model,
                                               bool showNormals,
                                               Eigen::Matrix4f pose,
                                               std::string visualizationType,
                                               CollisionCheckerPtr colChecker)
{
  ObstaclePtr result;
  VisualizationFactoryPtr visualizationFactory;
  if (visualizationType.empty())
    visualizationFactory=VisualizationFactory::first(NULL);
  else
    visualizationFactory = VisualizationFactory::fromName(visualizationType, NULL);
  if (!visualizationFactory)
  {
    VR_ERROR << "Could not create factory for visu type " << visualizationType << endl;
    return result;
  }

  VisualizationNodePtr visu = visualizationFactory->createTriMeshModelVisualization(model,
                                                                                    showNormals,
                                                                                    pose);
  if (!visu)
  {
    VR_ERROR << "Could not create box visualization with visu type " << visualizationType << endl;
    return result;
  }

  int id = idCounter;
  idCounter++;

  std::stringstream ss;
  ss << "MeshObstacle_" << id;
  std::string name = ss.str();

  CollisionModelPtr colModel(new CollisionModel(visu, name, colChecker, id));
  result.reset(new MeshObstacle(name, visu, colModel, SceneObject::Physics(), colChecker));
  result->initialize();

  return result;
}

//-------------------------------------------------------------------------------

/**
 * Create a simple TriMeshModel without input.
 **/
TriMeshModelPtr MeshObstacle::create_tri_mesh_skybox(void)
{
  TriMeshModelPtr triMeshModel(new TriMeshModel());

  // Coordinates of vertices for sky box.
  Eigen::Vector3f p0 ( -SKY_BOX_SIZE2_, -SKY_BOX_SIZE2_, -SKY_BOX_SIZE2_ );
  Eigen::Vector3f p1 (  SKY_BOX_SIZE2_, -SKY_BOX_SIZE2_, -SKY_BOX_SIZE2_ );
  Eigen::Vector3f p2 ( -SKY_BOX_SIZE2_,  SKY_BOX_SIZE2_, -SKY_BOX_SIZE2_ );
  Eigen::Vector3f p3 (  SKY_BOX_SIZE2_,  SKY_BOX_SIZE2_, -SKY_BOX_SIZE2_ );
  Eigen::Vector3f p4 ( -SKY_BOX_SIZE2_, -SKY_BOX_SIZE2_,  SKY_BOX_SIZE2_ );
  Eigen::Vector3f p5 (  SKY_BOX_SIZE2_, -SKY_BOX_SIZE2_,  SKY_BOX_SIZE2_ );
  Eigen::Vector3f p6 ( -SKY_BOX_SIZE2_,  SKY_BOX_SIZE2_,  SKY_BOX_SIZE2_ );
  Eigen::Vector3f p7 (  SKY_BOX_SIZE2_,  SKY_BOX_SIZE2_,  SKY_BOX_SIZE2_ );

  // http://goo.gl/kcpDIL
  triMeshModel->addTriangleWithFace(p0, p2, p1);
  triMeshModel->addTriangleWithFace(p1, p2, p3);
  triMeshModel->addTriangleWithFace(p2, p6, p3);
  triMeshModel->addTriangleWithFace(p3, p6, p7);
  triMeshModel->addTriangleWithFace(p4, p7, p6);
  triMeshModel->addTriangleWithFace(p7, p4, p5);
  triMeshModel->addTriangleWithFace(p0, p1, p5);
  triMeshModel->addTriangleWithFace(p0, p5, p4);
  triMeshModel->addTriangleWithFace(p2, p0, p4);
  triMeshModel->addTriangleWithFace(p2, p4, p6);
  triMeshModel->addTriangleWithFace(p7, p1, p3);
  triMeshModel->addTriangleWithFace(p7, p5, p1);

  return triMeshModel;
}

//-------------------------------------------------------------------------------

TriMeshModelPtr MeshObstacle::create_tri_mesh(const shape_msgs::Mesh &mesh_msg)
{
  TriMeshModelPtr triMeshModel(new TriMeshModel());

  std::vector<Eigen::Vector3f> nodes;
  for (size_t i = 0; i < mesh_msg.vertices.size(); i++)
  {
    const geometry_msgs::Point &p = mesh_msg.vertices[i];
    nodes.push_back( Eigen::Vector3f(p.x, p.y, p.z) );
  }

  for (size_t i = 0; i < mesh_msg.triangles.size(); i++)
  {
    const shape_msgs::MeshTriangle &triangle = mesh_msg.triangles[i];
    const unsigned int idx0 = triangle.vertex_indices[0];
    const unsigned int idx1 = triangle.vertex_indices[1];
    const unsigned int idx2 = triangle.vertex_indices[2];
    triMeshModel->addTriangleWithFace(nodes[idx0], nodes[idx1], nodes[idx2]);
  }

  // This method checks if all normals of the model point inwards or outwards and
  // flippes the faces which have a wrong orientation.
  bool inverted = true;
  unsigned int no_of_flipped_faces = triMeshModel->checkAndCorrectNormals(!inverted);
  std::cout << "Number of flipped faces = " << no_of_flipped_faces << "." << std::endl;
  //write_tri_mesh(triMeshModel, "latest.off");

  return triMeshModel;
}

//-------------------------------------------------------------------------------

void MeshObstacle::write_tri_mesh(TriMeshModelPtr model, std::string filename)
{
  std::ofstream outf(filename.c_str());
  if (!outf)
  {
    VR_ERROR << "Can't open file for writing: " << filename << endl;
    return;
  }
  std::vector<Eigen::Vector3f> verts = model->vertices;
  std::vector<MathTools::TriangleFace> faces = model->faces;
  outf << "OFF" << "\n";
  outf << verts.size() << " " << faces.size() << " 0\n";
  for (size_t i=0; i < verts.size(); ++i)
    outf << verts.at(i).x() << " " << verts.at(i).y() << " " << verts.at(i).z() << "\n";
  for (size_t i=0; i<faces.size(); ++i)
    outf << "3 " << faces.at(i).id1 << " " << faces.at(i).id2 << " " << faces.at(i).id3 << "\n";
  outf.close();
  VR_INFO << "Wrote file: " << filename << endl;
}
