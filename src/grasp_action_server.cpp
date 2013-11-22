#include "sr_grasp_mesh_planner/grasp_action_server.hpp"
#include "sr_grasp_mesh_planner/grasp_planner_window.hpp"

#include <string>
#include <iostream>
#include <boost/assign/list_of.hpp>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/RuntimeEnvironment.h>

//-------------------------------------------------------------------------------

const bool GraspActionServer::auto_start_ = true;

//-------------------------------------------------------------------------------

// This constructor uses actionlib!
// Note that node_name is used as the action name.
GraspActionServer::GraspActionServer(std::string node_name,
                                     boost::shared_ptr<GraspPlannerWindow> grasp_win)
  : as_mesh_(nh_,
             node_name,
             boost::bind(&GraspActionServer::goal_cb_mesh, this, _1),
             !GraspActionServer::auto_start_),
    action_name_(node_name),
    grasp_win_(grasp_win)
{
  feedback_mesh_.reset(new sr_grasp_mesh_planner::graspMeshFeedback);
  result_mesh_.reset(new sr_grasp_mesh_planner::graspMeshResult);

  as_mesh_.start();
  ROS_INFO_STREAM("Action server " << action_name_ << " just started.");
}

//-------------------------------------------------------------------------------

GraspActionServer::~GraspActionServer()
{
}

//-------------------------------------------------------------------------------

void GraspActionServer::goal_cb_mesh(const sr_grasp_mesh_planner::graspMeshGoalConstPtr &goal)
{
  bool success = true;

  // Construct an object from the given triangle mesh model (for the grasp planner).
  grasp_win_->loadObject(goal->obj_mesh);
  grasp_win_->buildVisu();

  // Init the actionlib feedback and result data.
  feedback_mesh_->no_of_stable_grasps = 0;
  result_mesh_->grasps.clear();

  // publish info to the console for the user
  ROS_INFO_STREAM(action_name_ << ": Executing GraspActionServer::goal_cb_mesh");

  // start executing the action
  // Note GraspPlannerWindow::plan may generate multiple grasps
  // if the user chooses to do so through GUI.
  const int num_of_desired_grasp_sets = 3; /////////////
  for (size_t i = 0; i < num_of_desired_grasp_sets; i++)
  {
    // Synthesize grasps.
    grasp_win_->plan(feedback_mesh_, result_mesh_);

    // check that preempt has not been requested by the client
    if (as_mesh_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_mesh_.setPreempted();
      success = false;
      break;
    }

    // publish the feedback
    as_mesh_.publishFeedback(*feedback_mesh_);
    ROS_INFO_STREAM("feedback_mesh_->no_of_stable_grasps = " << feedback_mesh_->no_of_stable_grasps);
  }

  if (success)
  {
    // set the action state to succeeded
    as_mesh_.setSucceeded(*result_mesh_);
    ROS_INFO_STREAM(action_name_ << ": Succeeded");
  }
}

//-------------------------------------------------------------------------------
