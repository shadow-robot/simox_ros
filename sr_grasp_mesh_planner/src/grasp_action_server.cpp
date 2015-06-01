/*
 * Copyright (c) 2013 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

/**
 * @file   grasp_action_server.cpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  A grasp action server.
 **/

#include "sr_grasp_mesh_planner/grasp_action_server.hpp"
#include "sr_grasp_mesh_planner/grasp_planner_window.hpp"

#include <string>
#include <iostream>
#include <boost/assign/list_of.hpp>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/RuntimeEnvironment.h>

//-------------------------------------------------------------------------------

using namespace sr_grasp_mesh_planner;

//-------------------------------------------------------------------------------

const bool GraspActionServer::auto_start_ = true;

//-------------------------------------------------------------------------------

/*
 * This constructor uses actionlib!
 * Note that node_name is used as the action name.
 * Parameters max_grasps_ etc will be set in GraspActionServer::config_cb_.
 */
GraspActionServer::GraspActionServer(std::string node_name,
                                     boost::shared_ptr<GraspPlannerWindow> grasp_win)
  : nh_("~"),
    action_name_(node_name),
    as_mesh_(nh_,
             action_name_,
             boost::bind(&GraspActionServer::goal_cb_, this, _1),
             !GraspActionServer::auto_start_),
    grasp_win_(grasp_win)
{
  // Set up dynamic_reconfigure.
  config_server_.setCallback( boost::bind(&GraspActionServer::config_cb_, this, _1, _2) );

  feedback_mesh_.reset(new sr_robot_msgs::PlanGraspFeedback);
  result_mesh_.reset(new sr_robot_msgs::PlanGraspResult);

  as_mesh_.start();
  ROS_INFO_STREAM("Action server " << action_name_ << " just started.");
}

//-------------------------------------------------------------------------------

GraspActionServer::~GraspActionServer()
{
}

//-------------------------------------------------------------------------------

void GraspActionServer::config_cb_(sr_grasp_mesh_planner::PlannerConfig &config,
                                  uint32_t level)
{
  max_grasps_        = config.max_grasps;
  timeout_one_grasp_ = config.timeout_one_grasp;
  min_quality_       = config.min_quality;
  force_closure_     = config.force_closure;
  approach_movement_ = config.approach_movement;
}

//-------------------------------------------------------------------------------

void GraspActionServer::goal_cb_(const sr_robot_msgs::PlanGraspGoalConstPtr &goal)
{
  bool success = true;

  // Construct an object from the given triangle mesh model (for the grasp planner).
  grasp_win_->loadObject(goal->object, approach_movement_);
  grasp_win_->buildVisu();

  // Init the actionlib feedback and result data.
  feedback_mesh_->number_of_synthesized_grasps = 0;
  result_mesh_->grasps.clear();

  // publish info to the console for the user
  ROS_INFO_STREAM("Action " << action_name_ << ": Executing GraspActionServer::goal_cb_");

  // *** start executing the action ***

  /*
   * GraspPlannerWindow::plan can generate multiple grasps.
   * However, we always generate a single grasp in the method.
   * Therefore the number grasp sets is equal to the number of grasps.
   */
  const int num_of_desired_grasp_sets = max_grasps_;
  for (size_t i = 0; i < num_of_desired_grasp_sets; i++)
  {
    // Synthesize grasps.
    grasp_win_->plan(force_closure_,
                     timeout_one_grasp_,
                     min_quality_,
                     feedback_mesh_,
                     result_mesh_);

    // check that preempt has not been requested by the client
    if (as_mesh_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_mesh_.setPreempted();
      success = false;
      ROS_INFO("%s: Preempted", action_name_.c_str());
      break;
    }

    // publish the feedback
    as_mesh_.publishFeedback(*feedback_mesh_);
    ROS_INFO_STREAM("feedback_mesh_->number_of_synthesized_grasps = " <<
                    feedback_mesh_->number_of_synthesized_grasps);
  }

  if (success)
  {
    // set the action state to succeeded
    as_mesh_.setSucceeded(*result_mesh_);
    ROS_INFO_STREAM("Action " << action_name_ << ": Succeeded");
  }
}

//-------------------------------------------------------------------------------
