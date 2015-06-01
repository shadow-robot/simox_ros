/*
 * Copyright (c) 2013 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

/**
 * @file   grasp_planner_window.cpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  The main Qt window for the Simox-based grasp planner.
 **/

#include "sr_grasp_mesh_planner/grasp_planner_window.hpp"
#include "sr_grasp_mesh_planner/sr_approach_movement_bounding_box.hpp"
#include "sr_grasp_mesh_planner/sr_approach_movement_surface_normal.hpp"
#include "sr_grasp_mesh_planner/mesh_obstacle.hpp"
#include "sr_grasp_mesh_planner/PlannerConfig.h"

#include <cmath>
#include <ctime>
#include <iostream>
#include <sstream>
#include <vector>

#include <boost/lexical_cast.hpp>
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

#include <ros/ros.h>

//-------------------------------------------------------------------------------

using namespace std;
using namespace VirtualRobot;
using namespace GraspStudio;
using namespace sr_grasp_mesh_planner;

//-------------------------------------------------------------------------------

GraspPlannerWindow::GraspPlannerWindow(string &robFile,
                                       string &eefName,
                                       string &preshape,
                                       VirtualRobot::TriMeshModelPtr triMeshModel,
                                       Qt::WFlags flags)
  : QMainWindow(NULL),

  viewer_(NULL), /*!< Viewer to display the 3D model of the robot and the environment. */

  sceneSep_(new SoSeparator),
  robotSep_(new SoSeparator),
  objectSep_(new SoSeparator),
  frictionConeSep_(new SoSeparator),
  graspsSep_(new SoSeparator),
  sphereSep_(new SoSeparator),

  robotFile_(robFile),
  eefName_(eefName),
  preshape_(preshape),
  eefVisu_(NULL),

  grasp_counter_(0)
{
  VR_INFO << " Start GraspPlannerWindow " << endl;

  // init the random number generator
  srand(time(NULL));

  sceneSep_->ref();
  graspsSep_->ref();

  sceneSep_->addChild(robotSep_);
  sceneSep_->addChild(objectSep_);
  sceneSep_->addChild(sphereSep_);
  sceneSep_->addChild(frictionConeSep_);

  setupUI();

  loadRobot();

  // Load a temporary object.
  int approach_movement = 0;
  loadObject(triMeshModel, approach_movement);

  buildVisu();

  viewer_->viewAll();
}

//-------------------------------------------------------------------------------

GraspPlannerWindow::~GraspPlannerWindow()
{
  sceneSep_->unref();
  graspsSep_->unref();
  if (eefVisu_)
    eefVisu_->unref();
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::setupUI()
{
  UI_.setupUi(this);
  viewer_ = new CoinViewer(UI_.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

  viewer_->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
  viewer_->setAccumulationBuffer(true);
  viewer_->setGLRenderAction(new SoLineHighlightRenderAction);
  viewer_->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
  viewer_->setFeedbackVisibility(true);
  viewer_->setSceneGraph(sceneSep_);
  viewer_->viewAll();

  connect(UI_.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEF()));
  connect(UI_.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEF()));
  connect(UI_.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
  connect(UI_.checkBoxCones, SIGNAL(clicked()), this, SLOT(frictionConeVisu()));
  connect(UI_.checkBoxGrasps, SIGNAL(clicked()), this, SLOT(showGrasps()));
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::resetSceneryAll()
{
  grasps_->removeAllGrasps();
  graspsSep_->removeAllChildren();
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::closeEvent(QCloseEvent *event)
{
  quit();
  QMainWindow::closeEvent(event);
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::buildVisu()
{
  viewer_->lock();

  robotSep_->removeAllChildren();
  SceneObject::VisualizationType colModel = (UI_.checkBoxColModel->isChecked() ?
                                             SceneObject::Collision : SceneObject::Full);
  if (eefCloned_)
  {
    visualizationRobot_ = eefCloned_->getVisualization<CoinVisualization>(colModel);
    SoNode* visualisationNode = visualizationRobot_->getCoinVisualization();
    if (visualisationNode)
    {
      robotSep_->addChild(visualisationNode);
      visualizationRobot_->highlight(UI_.checkBoxHighlight->isChecked());
    }
  }


  objectSep_->removeAllChildren();
  if (object_)
  {
    SceneObject::VisualizationType colModel2 = (UI_.checkBoxColModel->isChecked() ?
                                                SceneObject::CollisionData : SceneObject::Full);
    SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(object_, colModel2);
    if (visualisationNode)
    {
      objectSep_->addChild(visualisationNode);
    }
  }

  frictionConeSep_->removeAllChildren();
  bool fc = (UI_.checkBoxCones->isChecked());
  if (fc && contacts_.size()>0 && qualityMeasure_)
  {
    ContactConeGeneratorPtr cg = qualityMeasure_->getConeGenerator();
    float radius = cg->getConeRadius();
    float height = cg->getConeHeight();
    float scaling = 30.0f;
    SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(contacts_,
                                                                               height*scaling,
                                                                               radius*scaling,
                                                                               true);
    if (visualisationNode)
      frictionConeSep_->addChild(visualisationNode);

    // add approach dir visu
    for (size_t i=0;i<contacts_.size();i++)
    {
      SoSeparator *s = new SoSeparator;
      Eigen::Matrix4f ma;
      ma.setIdentity();
      ma.block(0,3,3,1) = contacts_[i].contactPointFingerGlobal;
      SoMatrixTransform *m = CoinVisualizationFactory::getMatrixTransformScaleMM2M(ma);
      s->addChild(m);
      s->addChild(CoinVisualizationFactory::CreateArrow(contacts_[i].approachDirectionGlobal,10.0f,1.0f));
      frictionConeSep_->addChild(s);
    }
  }

  if (UI_.checkBoxGrasps->isChecked() && sceneSep_->findChild(graspsSep_)<0)
    sceneSep_->addChild(graspsSep_);
  if (!UI_.checkBoxGrasps->isChecked() && sceneSep_->findChild(graspsSep_)>=0)
    sceneSep_->removeChild(graspsSep_);

  viewer_->scheduleRedraw();

  viewer_->unlock();
}

//-------------------------------------------------------------------------------

int GraspPlannerWindow::main()
{
  SoQt::show(this);
  SoQt::mainLoop();
  return 0;
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::quit()
{
  ROS_INFO("GraspPlannerWindow: Closing");
  this->close();
  SoQt::exitMainLoop();
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::loadObject(const object_recognition_msgs::RecognizedObject &object,
                                    int approach_movement)
{
  const shape_msgs::Mesh& obj_mesh = object.bounding_mesh;
  TriMeshModelPtr triMeshModel = MeshObstacle::create_tri_mesh(obj_mesh);
  this->loadObject(triMeshModel, approach_movement);
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::loadObject(VirtualRobot::TriMeshModelPtr triMeshModel,
                                    int approach_movement)
{
  viewer_->lock();

  // Simox uses MM while ROS uses M. So convert from M to MM.
  for (unsigned int i = 0; i < triMeshModel->vertices.size(); i++)
  {
    float x_MM = triMeshModel->vertices[i].x() * 1000.0; // M to MM
    float y_MM = triMeshModel->vertices[i].y() * 1000.0; // M to MM
    float z_MM = triMeshModel->vertices[i].z() * 1000.0; // M to MM
    triMeshModel->vertices[i] << x_MM, y_MM, z_MM;
  }

  const bool show_normals = true;
  object_ = MeshObstacle::create_mesh_obstacle(triMeshModel, !show_normals);

  Eigen::Vector3f minS, maxS;
  object_->getCollisionModel()->getTriMeshModel()->getSize(minS, maxS);
  ROS_INFO_STREAM("TriMeshModel minS: [" << minS[0] << ", " << minS[1] << ", " << minS[2] << "]");
  ROS_INFO_STREAM("TriMeshModel MaxS: [" << maxS[0] << ", " << maxS[1] << ", " << maxS[2] << "]");

  qualityMeasure_.reset(new GraspStudio::GraspQualityMeasureWrenchSpace(object_));
  // qualityMeasure_->setVerbose(true);
  qualityMeasure_->calculateObjectProperties();

  /*
   * Set approach movement generator.
   * Planner_bounding_box : Bounding box based approach movement generator.
   * Planner_surface_normal : Object surface normal based approach movement generator.
   * See cfg/Planner.cfg.
   */
  if (approach_movement == Planner_bounding_box)
  {
    approach_.reset(new SrApproachMovementBoundingBox(object_, eef_));
    ROS_INFO_STREAM("Choose the Bounding box based approach movement generator.");
  }
  else if (approach_movement == Planner_surface_normal)
  {
    approach_.reset(new SrApproachMovementSurfaceNormal(object_, eef_));
    ROS_INFO_STREAM("Choose the Object surface normal based approach movement generator.");
  }

  eefCloned_ = approach_->getEEFRobotClone();
  if (robot_ && eef_)
  {
    string name = "Grasp Planner - ";
    name += eef_->getName();
    grasps_.reset(new GraspSet(name, robot_->getType(), eefName_));
  }

  planner_.reset(new GraspStudio::GenericGraspPlanner(grasps_, qualityMeasure_, approach_));
  planner_->setVerbose(true);

  viewer_->unlock();
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::loadRobot()
{
  robot_.reset();
  robot_ = RobotIO::loadRobot(robotFile_);
  if (!robot_)
  {
    VR_ERROR << " no robot at " << robotFile_ << endl;
    return;
  }
  eef_ = robot_->getEndEffector(eefName_);
  if (!preshape_.empty())
  {
    eef_->setPreshape(preshape_);
  }

  eefVisu_ = CoinVisualizationFactory::CreateEndEffectorVisualization(eef_);
  eefVisu_->ref();
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::plan(bool force_closure,
                              float timeout,
                              float min_quality,
                              boost::shared_ptr<sr_robot_msgs::PlanGraspFeedback> feedback_mesh,
                              boost::shared_ptr<sr_robot_msgs::PlanGraspResult> result_mesh)
{
  feedback_mesh_ = feedback_mesh;
  result_mesh_ = result_mesh;
  this->resetSceneryAll();
  this->plan(force_closure, timeout, min_quality);
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::plan(bool force_closure,
                              float timeout,
                              float min_quality)
{
  viewer_->lock();

  // Start!
  clock_t begin = clock();

  /*
   * Parameter num_of_desired_grasp_sets is set in class GraspActionServer.
   * For every set, we generate ONE grasp.
   */
  const int nrDesiredGrasps = 1;
  const float timeout_ms =  timeout * 1000.0f; // second -> millisecond.

  planner_.reset(new GraspStudio::GenericGraspPlanner(grasps_,
                                                      qualityMeasure_,
                                                      approach_,
                                                      min_quality,
                                                      force_closure));

  int nrComputedGrasps = planner_->plan(nrDesiredGrasps, timeout_ms);
  grasps_->setPreshape(preshape_);
  for (size_t i=0; i < grasps_->getSize(); i++)
  {
    // m is the pose of the grasp applied to the global object pose,
    // resulting in the global TCP pose which is related to the grasp.
    Eigen::Matrix4f m = grasps_->getGrasp(i)->getTcpPoseGlobal(object_->getGlobalPose());
    SoMatrixTransform *mt = CoinVisualizationFactory::getMatrixTransformScaleMM2M(m);
    SoSeparator *grasp_sep = new SoSeparator();
    grasp_sep->addChild(mt);
    grasp_sep->addChild(eefVisu_);
    graspsSep_->addChild(grasp_sep);
  }

  //--------------------------------------------------------

  for (size_t i=0; i < grasps_->getSize(); i++)
  {
    // m is the pose of the grasp applied to the global object pose,
    // resulting in the global TCP pose which is related to the grasp.
    Eigen::Matrix4f m = grasps_->getGrasp(i)->getTcpPoseGlobal(object_->getGlobalPose());

    moveit_msgs::Grasp grasp_msg;

    // nrComputedGrasps should be one.
    feedback_mesh_->number_of_synthesized_grasps += nrComputedGrasps;

    // A name for this grasp.
    grasp_msg.id = string("grasp_") + boost::lexical_cast<string>(grasp_counter_);
    grasp_counter_++;

    // The position of the end-effector for the grasp.
    grasp_msg.grasp_pose.header.stamp = ros::Time::now();

    // The internal posture of the hand before the grasp.
    // positions and efforts (not set here) are used.
    if (eef_->hasPreshape("Grasp Preshape"))
    {
      trajectory_msgs::JointTrajectory pre_grasp_posture;
      pre_grasp_posture.header.stamp = grasp_msg.grasp_pose.header.stamp;
      // Get the configuration of the grasp.
      trajectory_msgs::JointTrajectoryPoint pre_grasp_point;

      map<string, float> robotNodeJointValueMap = eef_->getPreshape("Grasp Preshape")->getRobotNodeJointValueMap();
      for (map<string, float>::const_iterator it = robotNodeJointValueMap.begin();
           it != robotNodeJointValueMap.end();
           ++it)
      {
        // Set joint name.
        pre_grasp_posture.joint_names.push_back(it->first);
        // Set position (i.e., angle).
        pre_grasp_point.positions.push_back(it->second); // Unit is radian
      }
      if (!pre_grasp_posture.joint_names.empty())
        pre_grasp_posture.points.push_back(pre_grasp_point);
      // Set the pre-grasp posture.
      grasp_msg.pre_grasp_posture = pre_grasp_posture;
    }

    // The internal posture of the hand for the grasp.
    // positions and efforts (not set here) are used.
    trajectory_msgs::JointTrajectory grasp_posture;
    grasp_posture.header.stamp = grasp_msg.grasp_pose.header.stamp;
    // Get the configuration of the grasp.
    trajectory_msgs::JointTrajectoryPoint grasp_point;

    map<string, float> configuration = grasps_->getGrasp(i)->getConfiguration();
    for (map<string, float>::const_iterator it = configuration.begin();
         it != configuration.end();
         ++it)
    {
      // Set joint name.
      grasp_posture.joint_names.push_back(it->first);
      // Set position (i.e., angle).
      grasp_point.positions.push_back(it->second); // Unit is radian
    }
    if (!grasp_posture.joint_names.empty())
      grasp_posture.points.push_back(grasp_point);
    // Set the grasp posture.
    grasp_msg.grasp_posture = grasp_posture;

    // Get the transformation of this grasp.
    // The transformation is given in the coordinate system of the tcp,
    // whereas the tcp belongs to the eef.
    // This transformation specifies the tcp to object relation.
    Eigen::Matrix4f poseTcp = grasps_->getGrasp(i)->getTransformation();

    // Now the eef can be set to a position so that it's TCP is at m.
    // m is the pose of the grasp applied to the global object pose,
    // resulting in the global TCP pose which is related to the grasp.
    eefCloned_->setGlobalPoseForRobotNode(eefCloned_->getEndEffector(eefName_)->getTcp(), m);
    Eigen::Matrix4f poseGlobal = eefCloned_->getGlobalPose();

    // We obtain the translation from the root to the TCP, to be able to get the grasp pose in the root (forearm) frame
    // We do it this way because the TCP we have defined doesn't match any of the robot links' frames
    Eigen::Matrix4f globalPoseTcp = eef_->getTcp()->getGlobalPose();

    // Here we choose between poseTcp and poseGlobal to save.
    Eigen::Matrix4f poseToSave = poseTcp;
    // Set pose.position.
    grasp_msg.grasp_pose.header.frame_id = "forearm";
    grasp_msg.grasp_pose.pose.position.x = (poseToSave(0,3) + globalPoseTcp(0,3)) / 1000.0; // /1000 as ros msg is in meters instead of mm
    grasp_msg.grasp_pose.pose.position.y = (poseToSave(1,3) + globalPoseTcp(1,3)) / 1000.0;
    grasp_msg.grasp_pose.pose.position.z = (poseToSave(2,3) + globalPoseTcp(2,3)) / 1000.0;
    // Set pose.orientation.
    MathTools::Quaternion q = MathTools::eigen4f2quat(poseToSave);
    grasp_msg.grasp_pose.pose.orientation.x = q.x;
    grasp_msg.grasp_pose.pose.orientation.y = q.y;
    grasp_msg.grasp_pose.pose.orientation.z = q.z;
    grasp_msg.grasp_pose.pose.orientation.w = q.w;

    // The estimated probability of success for this grasp.
    grasp_msg.grasp_quality = grasps_->getGrasp(i)->getQuality();

    // Save the current moveit_msgs::Grasp.
    result_mesh_->grasps.push_back(grasp_msg);
  }

  //--------------------------------------------------------

  // Set to the last valid grasp.
  if (grasps_->getSize()>0 && eefCloned_ && eefCloned_->getEndEffector(eefName_))
  {
    int last_idx = grasps_->getSize() - 1;
    // last_m is the pose of the grasp applied to the global object pose,
    // resulting in the global TCP pose which is related to the grasp.
    Eigen::Matrix4f last_m = grasps_->getGrasp(last_idx)->getTcpPoseGlobal(object_->getGlobalPose());
    // Now the eef can be set to a position so that it's TCP is at last_m.
    eefCloned_->setGlobalPoseForRobotNode(eefCloned_->getEndEffector(eefName_)->getTcp(), last_m);

    this->openEEF();
    this->closeEEF();
  }

  clock_t end = clock();
  ROS_INFO_STREAM("Grasp planning took " << static_cast<double>(diffclock(end, begin)) << " ms.");

  viewer_->unlock();
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::openEEF()
{
  contacts_.clear();
  if (eefCloned_ && eefCloned_->getEndEffector(eefName_))
  {
    eefCloned_->getEndEffector(eefName_)->openActors();
  }
  this->buildVisu();
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::closeEEF()
{
  contacts_.clear();
  if (eefCloned_ && eefCloned_->getEndEffector(eefName_))
  {
    contacts_ = eefCloned_->getEndEffector(eefName_)->closeActors(object_);

    // The last computed quality is retrieved with
    float qual = qualityMeasure_->getGraspQuality();
    bool isFC = qualityMeasure_->isGraspForceClosure();

    stringstream ss;
    ss << setprecision(3);
    ss << "Grasp Nr " << grasps_->getSize();
    ss << "\nQuality (wrench space): ";
    ss << "\n  " << qual;
    ss << "\nForce closure: ";
    if (isFC)
      ss << "yes";
    else
      ss << "no";
    UI_.labelInfo->setText(QString (ss.str().c_str()));
  }
  this->buildVisu();
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::frictionConeVisu()
{
  this->buildVisu();
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::colModel()
{
  this->buildVisu();
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::showGrasps()
{
  this->buildVisu();
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::save()
{
  if (!object_)
    return;

  ManipulationObjectPtr objectM(new ManipulationObject(object_->getName(),
                                                       object_->getVisualization()->clone(),
                                                       object_->getCollisionModel()->clone()));
  objectM->addGraspSet(grasps_);
  QString fi = QFileDialog::getSaveFileName(this,
                                            tr("Save ManipulationObject"),
                                            QString(),
                                            tr("XML Files (*.xml)"));
  string objectFile = fi.toStdString();
  bool ok = false;
  try
  {
    ok = ObjectIO::saveManipulationObject(objectM, objectFile);
  }
  catch (VirtualRobotException &e)
  {
    ROS_ERROR_STREAM(" ERROR while saving object");
    ROS_ERROR_STREAM(e.what());
    return;
  }

  if (!ok)
  {
    ROS_ERROR_STREAM(" ERROR while saving object");
    return;
  }
}

//-------------------------------------------------------------------------------

double GraspPlannerWindow::diffclock(clock_t clock1, clock_t clock2)
{
  double diffticks=clock1-clock2;
  double diffms=(diffticks*1000)/CLOCKS_PER_SEC;
  return diffms;
}

//-------------------------------------------------------------------------------
