/*
 * Copyright (c) 2013 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

/**
 * @file   grasp_action_server.hpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  A grasp action server.
 **/

#pragma once

//-------------------------------------------------------------------------------

#include "sr_grasp_mesh_planner/grasp_planner_window.hpp"
#include <sr_grasp_mesh_planner/PlannerConfig.h>
#include <sr_grasp_msgs/PlanGraspAction.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>

#include <VirtualRobot/Visualization/TriMeshModel.h>

//-------------------------------------------------------------------------------

namespace sr_grasp_mesh_planner
{

using namespace VirtualRobot;

class GraspActionServer
{
protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.

  // Auto start the action server?
  static const bool auto_start_;

  std::string action_name_;

  actionlib::SimpleActionServer<sr_grasp_msgs::PlanGraspAction> as_mesh_;
  boost::shared_ptr<sr_grasp_msgs::PlanGraspFeedback> feedback_mesh_;
  boost::shared_ptr<sr_grasp_msgs::PlanGraspResult> result_mesh_;

  boost::shared_ptr<GraspPlannerWindow> grasp_win_;

  bool time_to_quit_;

  float timeout_total_;
  float timeout_one_grasp_;
  bool force_closure_;
  float min_quality_;
  int max_grasps_;

  dynamic_reconfigure::Server<sr_grasp_mesh_planner::PlannerConfig> config_server_;

public:
  // This constructor uses actionlib!
  // Note that node_name is used as the action name.
  GraspActionServer(std::string node_name,
                    boost::shared_ptr<GraspPlannerWindow> grasp_win);

  virtual ~GraspActionServer();

private:
  void goal_cb_(const sr_grasp_msgs::PlanGraspGoalConstPtr &goal);

  void config_cb_(sr_grasp_mesh_planner::PlannerConfig &config, uint32_t level);

  void timer_cb_(const ros::WallTimerEvent& event);
};

} // end of namespace sr_grasp_mesh_planner

//-------------------------------------------------------------------------------
