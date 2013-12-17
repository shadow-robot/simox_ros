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

// This constructor uses actionlib!
// Note that node_name is used as the action name.
GraspActionServer::GraspActionServer(std::string node_name,
                                     boost::shared_ptr<GraspPlannerWindow> grasp_win)
  : action_name_(node_name),
    as_mesh_(nh_,
             action_name_,
             boost::bind(&GraspActionServer::goal_cb_, this, _1),
             !GraspActionServer::auto_start_),
    grasp_win_(grasp_win),
    timeout_(60.0),
    force_closure_(true),
    min_quality_(0.0),
    max_grasps_(1)
{
  // Set up dynamic_reconfigure.
  config_server_.setCallback( boost::bind(&GraspActionServer::config_cb_, this, _1, _2) );

  feedback_mesh_.reset(new sr_grasp_msgs::PlanGraspFeedback);
  result_mesh_.reset(new sr_grasp_msgs::PlanGraspResult);

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
  max_grasps_    = config.max_grasps;
  timeout_       = config.timeout;
  min_quality_   = config.min_quality;
  force_closure_ = config.force_closure;
}

//-------------------------------------------------------------------------------

void GraspActionServer::goal_cb_(const sr_grasp_msgs::PlanGraspGoalConstPtr &goal)
{
  bool success = true;

  // Construct an object from the given triangle mesh model (for the grasp planner).
  grasp_win_->loadObject(goal->object);
  grasp_win_->buildVisu();

  // Init the actionlib feedback and result data.
  feedback_mesh_->number_of_synthesized_grasps = 0;
  result_mesh_->grasps.clear();

  // publish info to the console for the user
  ROS_INFO_STREAM("Action " << action_name_ << ": Executing GraspActionServer::goal_cb_");

  // *** start executing the action ***

  // Start the timer.
  time_to_quit_ = false;
  ros::WallTimer timer;
  if (timeout_ > 0.0)
  {
    timer = nh_.createWallTimer(ros::WallDuration(timeout_),
                                &GraspActionServer::timer_cb_,
                                this);
    timer.start();
  }

  // Note GraspPlannerWindow::plan may generate multiple grasps
  // if the user chooses to do so through GUI.
  const int num_of_desired_grasp_sets = max_grasps_;
  for (size_t i = 0; i < num_of_desired_grasp_sets; i++)
  {
    // Synthesize grasps.
    grasp_win_->plan(force_closure_, min_quality_, feedback_mesh_, result_mesh_);

    // check that preempt has not been requested by the client
    if (as_mesh_.isPreemptRequested() || !ros::ok() || time_to_quit_)
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_mesh_.setPreempted();
      success = false;
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

void GraspActionServer::timer_cb_(const ros::WallTimerEvent& event)
{
  time_to_quit_ = true;
}

//-------------------------------------------------------------------------------
