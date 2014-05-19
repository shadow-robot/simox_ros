/*
 * Copyright (c) 2013 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

/**
 * @file   sr_approach_movement_bounding_box.cpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  Bounding box based approach movement generator.
 **/

#include "sr_grasp_mesh_planner/sr_approach_movement_bounding_box.hpp"
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/BoundingBox.h>
#include <VirtualRobot/SceneObjectSet.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <ros/ros.h>

namespace sr_grasp_mesh_planner
{

SrApproachMovementBoundingBox::SrApproachMovementBoundingBox(VirtualRobot::SceneObjectPtr object,
                                                             VirtualRobot::EndEffectorPtr eef,
                                                             const std::string &graspPreshape,
                                                             float maxRandDist)
  : ApproachMovementSurfaceNormal(object, eef, graspPreshape, maxRandDist)
{
  name = "SrApproachMovementBoundingBox";

  constructBoundingBoxObject(object);
}

SrApproachMovementBoundingBox::~SrApproachMovementBoundingBox()
{
}

void SrApproachMovementBoundingBox::constructBoundingBoxObject(VirtualRobot::SceneObjectPtr object)
{
  // An axis oriented bounding box.
  VirtualRobot::BoundingBox bb = object->getCollisionModel()->getBoundingBox();
  std::vector <Eigen::Vector3f> bb_points = bb.getPoints();

  // We get 8 vertices. From these vertices, we can construct 12 triangles (that cover the surface
  // of the bounding box), 2 triangles per face.
  Eigen::MatrixXi triangles = Eigen::MatrixXi::Zero(12, 3);
  triangles << 3, 1, 5,
    3, 5, 7,
    4, 0, 2,
    4, 2, 6,
    7, 5, 4,
    7, 4, 6,
    1, 3, 0,
    3, 2, 0,
    3, 7, 6,
    3, 6, 2,
    5, 1, 0,
    5, 0, 4;

  for (size_t row = 0; row < triangles.rows(); ++row)
  {
    int idx1 = triangles(row, 0);
    int idx2 = triangles(row, 1);
    int idx3 = triangles(row, 2);

    Eigen::Vector3f vertex1 = bb_points[idx1];
    Eigen::Vector3f vertex2 = bb_points[idx2];
    Eigen::Vector3f vertex3 = bb_points[idx3];

    Eigen::Vector3f normal = (vertex2 - vertex1).cross(vertex3 - vertex1);
    normal.normalize();
    bb_object_.addTriangleWithFace(vertex1, vertex2, vertex3, normal);
  }
}

Eigen::Matrix4f SrApproachMovementBoundingBox::createNewApproachPose()
{
  // store current pose
  Eigen::Matrix4f pose = getEEFPose();
  openHand();
  Eigen::Vector3f position;
  Eigen::Vector3f approachDir;
  if (!this->getPositionOnObjectWithFocalPoint(position, approachDir))
  {
    ROS_ERROR_STREAM("no position on object?!");
    return pose;
  }

  // set new pose
  this->setEEFToApproachPose(position, approachDir);

  // move away until valid
  this->moveEEFAway(approachDir,3.0f);

  Eigen::Matrix4f poseB = this->getEEFPose();

  // restore original pose
  this->setEEFPose(pose);

  return poseB;
}

bool SrApproachMovementBoundingBox::getPositionOnObjectWithFocalPoint(Eigen::Vector3f &storePos,
                                                                      Eigen::Vector3f &storeApproachDir)
{
  if (bb_object_.faces.size() == 0)
    return false;

  int nRandFace = rand() % bb_object_.faces.size();
  int nVert1 = (bb_object_.faces[nRandFace]).id1;
  int nVert2 = (bb_object_.faces[nRandFace]).id2;
  int nVert3 = (bb_object_.faces[nRandFace]).id3;

  storePos = VirtualRobot::MathTools::randomPointInTriangle(bb_object_.vertices[nVert1],
                                                            bb_object_.vertices[nVert2],
                                                            bb_object_.vertices[nVert3]);

  storeApproachDir = (bb_object_.faces[nRandFace]).normal;

  return true;
}

} // end of namespace sr_grasp_mesh_planner
