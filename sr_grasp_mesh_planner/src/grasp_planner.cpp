#include "sr_grasp_mesh_planner/grasp_planner_window.hpp"
#include "sr_grasp_mesh_planner/grasp_action_server.hpp"
#include "sr_grasp_mesh_planner/sr_approach_movement_surface_normal.hpp"
#include "sr_grasp_mesh_planner/mesh_obstacle.hpp"

#include <ros/ros.h>
#include <ros/package.h>

//-------------------------------------------------------------------------------

using namespace sr_grasp_mesh_planner;

//-------------------------------------------------------------------------------

void ros_spin(void)
{
  ros::spin();
}

//-------------------------------------------------------------------------------

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sr_grasp_mesh_planner");

  SoDB::init();
  SoQt::init(argc, argv, "sr_grasp_mesh_planner");
  ROS_INFO_STREAM(" --- START --- ");

  // You can choose a different robot / endeffector / preshape with command-line arguments:
  // --robot robots/iCub/iCub.xml --endeffector "Left Hand" --preshape "Grasp Preshape"
  std::string robot = ros::package::getPath("sr_grasp_description");
  robot.append("/simox/shadowhand.xml");
  std::string eef("SHADOWHAND");
  std::string preshape("Grasp Preshape");

  if (!VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robot))
  {
    ROS_FATAL("Shadow hand not found");
    return EXIT_FAILURE;
  }

  VirtualRobot::RuntimeEnvironment::considerKey("robot");
  VirtualRobot::RuntimeEnvironment::considerKey("endeffector");
  VirtualRobot::RuntimeEnvironment::considerKey("preshape");
  VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
  VirtualRobot::RuntimeEnvironment::print();

  std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");
  if (!robFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
    robot = robFile;

  std::string eefname = VirtualRobot::RuntimeEnvironment::getValue("endeffector");
  if (!eefname.empty())
    eef = eefname;

  std::string ps = VirtualRobot::RuntimeEnvironment::getValue("preshape");
  if (!ps.empty())
    preshape = ps;

  ROS_INFO_STREAM("-----------------");
  ROS_INFO_STREAM("Using robot from " << robot);
  ROS_INFO_STREAM("End effector: " << eef);
  ROS_INFO_STREAM("Preshape: " << preshape);
  ROS_INFO_STREAM("-----------------");

  TriMeshModelPtr skybox = MeshObstacle::create_tri_mesh_skybox();

  boost::shared_ptr<GraspPlannerWindow> grasp_win(new GraspPlannerWindow(robot, eef, preshape, skybox));

  GraspActionServer grasp_as_("plan_grasp", grasp_win);
  boost::thread spin_thread(&ros_spin);

  // Start Qt!
  grasp_win->main();

  // Shutdown the node and join the thread back before exiting.
  ros::shutdown();
  spin_thread.join();

  return EXIT_SUCCESS;
}

//-------------------------------------------------------------------------------
