#pragma once

#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <GraspPlanning/ApproachMovementSurfaceNormal.h>

namespace sr_grasp_mesh_planner
{

class SrApproachMovementBoundingBox : public GraspStudio::ApproachMovementSurfaceNormal
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SrApproachMovementBoundingBox(VirtualRobot::SceneObjectPtr object,
                                VirtualRobot::EndEffectorPtr eef,
                                const std::string &graspPreshape = "",
                                float maxRandDist = 0.0f);

  virtual ~SrApproachMovementBoundingBox();

  //! Creates a new pose for approaching
  virtual Eigen::Matrix4f createNewApproachPose();

  //! Returns a position with normal on the surface of the object
  bool getPositionOnObjectWithFocalPoint(Eigen::Vector3f &storePos,
                                         Eigen::Vector3f &storeApproachDir);

private:
  void constructBoundingBoxObject(VirtualRobot::SceneObjectPtr object);

  //! A triangle mesh model contructed from the object's (axis oriented) bounding box.
  VirtualRobot::TriMeshModel bb_object_;

  //! From the object and outward.
  Eigen::Vector3f approach_direction_;
};

} // end of namespace sr_grasp_mesh_planner
