#pragma once

//-------------------------------------------------------------------------------

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <sr_grasp_msgs/graspMeshAction.h>

#include "sr_grasp_mesh_planner/grasp_planner_window.hpp"

//-------------------------------------------------------------------------------

using namespace VirtualRobot;

//-------------------------------------------------------------------------------

class GraspActionServer
{
protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.

  // Auto start the action server?
  static const bool auto_start_;

  std::string action_name_;

  actionlib::SimpleActionServer<sr_grasp_msgs::graspMeshAction> as_mesh_;
  boost::shared_ptr<sr_grasp_msgs::graspMeshFeedback> feedback_mesh_;
  boost::shared_ptr<sr_grasp_msgs::graspMeshResult> result_mesh_;

  boost::shared_ptr<GraspPlannerWindow> grasp_win_;

public:
  // This constructor uses actionlib!
  // Note that node_name is used as the action name.
  GraspActionServer(std::string node_name,
                    boost::shared_ptr<GraspPlannerWindow> grasp_win);

  virtual ~GraspActionServer();

private:
  void goal_cb_mesh(const sr_grasp_msgs::graspMeshGoalConstPtr &goal);
};

//-------------------------------------------------------------------------------
