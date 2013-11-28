#include "sr_grasp_mesh_planner/sr_approach_movement_surface_normal.hpp"
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/SceneObjectSet.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>

#include <cstdio>
#include <cstring>
#include <iostream>

//-------------------------------------------------------------------------------

using namespace sr_grasp_mesh_planner;

//-------------------------------------------------------------------------------

SrApproachMovementSurfaceNormal::SrApproachMovementSurfaceNormal(VirtualRobot::SceneObjectPtr object,
                                                                 VirtualRobot::EndEffectorPtr eef,
                                                                 const std::string &graspPreshape,
                                                                 float maxRandDist)
  : ApproachMovementSurfaceNormal(object, eef, graspPreshape, maxRandDist)
{
  name = "SrApproachMovementSurfaceNormal";

  approach_direction_ = Eigen::Vector3f(-1.0, 0.0, 0.0);
  approach_direction_.normalize();
  // Eigen::Vector3f com = object->getCollisionModel()->getTriMeshModel()->getCOM();
}

//-------------------------------------------------------------------------------

SrApproachMovementSurfaceNormal::~SrApproachMovementSurfaceNormal()
{
}

//-------------------------------------------------------------------------------

Eigen::Matrix4f SrApproachMovementSurfaceNormal::createNewApproachPose()
{
  // store current pose
  Eigen::Matrix4f pose = getEEFPose();
  openHand();
  Eigen::Vector3f position;
  Eigen::Vector3f approachDir;
  if (!this->getPositionOnObjectWithFocalPoint(position,approachDir))
  {
    GRASPSTUDIO_ERROR << "no position on object?!" << std::endl;
    return pose;
  }

  // set new pose
  this->setEEFToApproachPose(position,approachDir);

  // move away until valid
  this->moveEEFAway(approachDir,3.0f);

  Eigen::Matrix4f poseB = this->getEEFPose();

  /*
  // check if a random distance is wanted
  if (randomDistanceMax > 0)
  {
    float d = (float)(rand()%10000)*0.0001f * randomDistanceMax;
    Eigen::Vector3f delta = approachDir * d;
    this->updateEEFPose(delta);
    if (!eef_cloned->getCollisionChecker()->checkCollision(object,eef->createSceneObjectSet()))
    {
      poseB = this->getEEFPose();
    } // else remain at original pose
  }
  */

  // restore original pose
  this->setEEFPose(pose);

  return poseB;
}

//-------------------------------------------------------------------------------

bool SrApproachMovementSurfaceNormal::getPositionOnObjectWithFocalPoint(Eigen::Vector3f &storePos,
                                                                        Eigen::Vector3f &storeApproachDir)
{
  if (!object || objectModel->faces.size()==0)
    return false;

  int nRandFace = rand() % objectModel->faces.size();
  int nVert1 = (objectModel->faces[nRandFace]).id1;
  int nVert2 = (objectModel->faces[nRandFace]).id2;
  int nVert3 = (objectModel->faces[nRandFace]).id3;

  storePos = VirtualRobot::MathTools::randomPointInTriangle(objectModel->vertices[nVert1],
                                                            objectModel->vertices[nVert2],
                                                            objectModel->vertices[nVert3]);

  // storeApproachDir = (objectModel->faces[nRandFace]).normal;
  storeApproachDir = approach_direction_;

  return true;
}

//-------------------------------------------------------------------------------
