/*
 * Copyright (c) 2013 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

/**
 * @file   mesh_obstacle.hpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  Mesh obstacle.
 **/

#pragma once

//-------------------------------------------------------------------------------

#include <geometry_msgs/Point.h>
#include <shape_msgs/MeshTriangle.h>
#include <shape_msgs/Mesh.h>
#include <pcl_msgs/PolygonMesh.h>

#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>

//-------------------------------------------------------------------------------

using namespace VirtualRobot;

//-------------------------------------------------------------------------------

namespace sr_grasp_mesh_planner
{

/**
 * A mesh obstacle is based on an obstacle, that is an object that owns a visualization
 * and a collision model. It can be moved around and used for collision detection.
 **/
class MeshObstacle : public Obstacle
{
public:
  MeshObstacle(const std::string &name,
               VisualizationNodePtr visualization = VisualizationNodePtr(),
               CollisionModelPtr collisionModel = CollisionModelPtr(),
               const SceneObject::Physics &p = SceneObject::Physics(),
               CollisionCheckerPtr colChecker = CollisionCheckerPtr());

  virtual ~MeshObstacle();

  static ObstaclePtr create_mesh_obstacle(TriMeshModelPtr model,
                                          bool showNormals = false,
                                          Eigen::Matrix4f pose = Eigen::Matrix4f::Identity(),
                                          std::string visualizationType = "",
                                          CollisionCheckerPtr colChecker = CollisionCheckerPtr());

  static TriMeshModelPtr create_tri_mesh_skybox(void);
  static TriMeshModelPtr create_tri_mesh(const shape_msgs::Mesh &mesh_msg);

  /**
   * Debug function to write a tri mesh to an OFF file.
   * http://segeval.cs.princeton.edu/public/off_format.html
   */
  static void write_tri_mesh(TriMeshModelPtr model, std::string filename);

private:
  // Size of the skybox divided by two.
  static const float SKY_BOX_SIZE2_;
};

} // end of namespace sr_grasp_mesh_planner

//-------------------------------------------------------------------------------
