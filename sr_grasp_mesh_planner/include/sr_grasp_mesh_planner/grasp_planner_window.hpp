/*
 * Copyright (c) 2013 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

/**
 * @file   grasp_planner_window.hpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  The main Qt window for the Simox-based grasp planner.
 **/

#pragma once

//-------------------------------------------------------------------------------

#include "sr_grasp_mesh_planner/coin_viewer.hpp"
#include <sr_robot_msgs/PlanGraspAction.h>
#include <shape_msgs/Mesh.h>

#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/Grasping/Grasp.h>
#include <VirtualRobot/Grasping/GraspSet.h>
#include <VirtualRobot/CollisionDetection/CDManager.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>

#include <GraspPlanning/GraspStudio.h>
#include <GraspPlanning/ContactConeGenerator.h>
#include <GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h>
#include <GraspPlanning/GraspPlanner/GenericGraspPlanner.h>
#include <GraspPlanning/ApproachMovementSurfaceNormal.h>
#include <GraspPlanning/Visualization/CoinVisualization/CoinConvexHullVisualization.h>

#include <string>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>

#include "ui_GraspPlanner.h"

//-------------------------------------------------------------------------------

namespace sr_grasp_mesh_planner
{

class GraspPlannerWindow : public QMainWindow
{
  Q_OBJECT

  public:
  GraspPlannerWindow(std::string &robotFile,
                     std::string &eefName,
                     std::string &preshape,
                     VirtualRobot::TriMeshModelPtr triMeshModel,
                     Qt::WFlags flags = 0);
  ~GraspPlannerWindow();

  /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
  int main();

public slots:
  /*! Closes the window and exits SoQt runloop. */
  void quit();

  /*!< Overriding the close event, so we know when the window was closed by the user. */
  void closeEvent(QCloseEvent *event);

  void resetSceneryAll();

  void closeEEF();
  void openEEF();
  void colModel();
  void frictionConeVisu();
  void showGrasps();

  void buildVisu();

  void plan(bool force_closure,
            float timeout,
            float min_quality,
            boost::shared_ptr<sr_robot_msgs::PlanGraspFeedback> feedback_mesh,
            boost::shared_ptr<sr_robot_msgs::PlanGraspResult> result_mesh);
  void plan(bool force_closure,
            float timeout,
            float min_quality);
  void save();

  void loadRobot();
  void loadObject(const object_recognition_msgs::RecognizedObject &object,
                  int approach_movement);
  void loadObject(VirtualRobot::TriMeshModelPtr triMeshModel,
                  int approach_movement);

  void setupUI();

  static double diffclock(clock_t clock1, clock_t clock2);

protected:
  Ui::GraspPlanner UI_;
  CoinViewer *viewer_; /*!< Viewer to display the 3D model of the robot and the environment. */

  SoSeparator *sceneSep_;
  SoSeparator *robotSep_;
  SoSeparator *objectSep_;
  SoSeparator *frictionConeSep_;
  SoSeparator *graspsSep_;

  SoSeparator *sphereSep_;

  VirtualRobot::RobotPtr robot_;
  VirtualRobot::RobotPtr eefCloned_;
  VirtualRobot::ObstaclePtr object_;
  VirtualRobot::EndEffectorPtr eef_;

  VirtualRobot::GraspSetPtr grasps_;

  VirtualRobot::EndEffector::ContactInfoVector contacts_;

  std::string robotFile_;
  std::string eefName_;
  std::string preshape_;

  SoSeparator *eefVisu_;

  GraspStudio::GraspQualityMeasureWrenchSpacePtr qualityMeasure_;
  GraspStudio::ApproachMovementSurfaceNormalPtr approach_;
  GraspStudio::GenericGraspPlannerPtr planner_;

  boost::shared_ptr<VirtualRobot::CoinVisualization> visualizationRobot_;

  // Not used.
  // boost::shared_ptr<VirtualRobot::CoinVisualization> visualizationObject_;

  boost::shared_ptr<sr_robot_msgs::PlanGraspFeedback> feedback_mesh_;
  boost::shared_ptr<sr_robot_msgs::PlanGraspResult> result_mesh_;

  unsigned short grasp_counter_;
};

} // end of namespace sr_grasp_mesh_planner

//-------------------------------------------------------------------------------
