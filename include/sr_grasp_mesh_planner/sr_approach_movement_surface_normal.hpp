/*
 * Copyright (c) 2013 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

/**
 * @file   sr_approach_movement_surface_normal.hpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  Object surface normal based approach movement generator.
 **/

#pragma once

#include <GraspPlanning/ApproachMovementSurfaceNormal.h>

//-------------------------------------------------------------------------------

namespace sr_grasp_mesh_planner
{

class SrApproachMovementSurfaceNormal : public GraspStudio::ApproachMovementSurfaceNormal
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SrApproachMovementSurfaceNormal(VirtualRobot::SceneObjectPtr object,
                                  VirtualRobot::EndEffectorPtr eef,
                                  const std::string &graspPreshape = "",
                                  float maxRandDist = 0.0f);

  virtual ~SrApproachMovementSurfaceNormal();

  //! Creates a new pose for approaching
  virtual Eigen::Matrix4f createNewApproachPose();

  //! Returns a position with normal on the surface of the object
  bool getPositionOnObjectWithFocalPoint(Eigen::Vector3f &storePos,
                                         Eigen::Vector3f &storeApproachDir);
};

} // end of namespace sr_grasp_mesh_planner

//-------------------------------------------------------------------------------
