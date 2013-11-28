#include "sr_grasp_mesh_planner/grasp_planner_window.hpp"
#include "sr_grasp_mesh_planner/grasp_action_server.hpp"
#include "sr_grasp_mesh_planner/sr_approach_movement_surface_normal.hpp"
#include "sr_grasp_mesh_planner/mesh_obstacle.hpp"

#include <cmath>
#include <ctime>
#include <iostream>
#include <sstream>
#include <vector>

#include <ros/ros.h>

#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <Eigen/Geometry>
#include <QFileDialog>

#include <Inventor/actions/SoLineHighlightRenderAction.h>
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoTransformation.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoUnits.h>

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
  std::cout << " --- START --- " << std::endl;

  // --robot robots/iCub/iCub.xml --endeffector "Left Hand" --preshape "Grasp Preshape"
  std::string robot("robots/ArmarIII/ArmarIII.xml");
  VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robot);
  std::string eef("Hand R");
  std::string preshape("");

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

  std::cout << "-----------------" << std::endl;
  std::cout << "Using robot from " << robot <<std:: endl;
  std::cout << "End effector: " << eef << std::endl;
  std::cout << "Preshape: " << preshape << std::endl;
  std::cout << "-----------------" << std::endl;

  TriMeshModelPtr skybox = MeshObstacle::create_tri_mesh_skybox();

  boost::shared_ptr<GraspPlannerWindow> grasp_win(new GraspPlannerWindow(robot, eef, preshape, skybox));

  GraspActionServer grasp_as_("sr_grasp_mesh_planner", grasp_win);
  boost::thread spin_thread(&ros_spin);

  // Start Qt!
  grasp_win->main();

  // Shutdown the node and join the thread back before exiting.
  ros::shutdown();
  spin_thread.join();

  return 0;
}

//-------------------------------------------------------------------------------
