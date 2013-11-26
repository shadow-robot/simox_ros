#include "sr_grasp_mesh_planner/grasp_planner_window.hpp"
#include "sr_grasp_mesh_planner/sr_approach_movement_surface_normal.hpp"
#include "sr_grasp_mesh_planner/mesh_obstacle.hpp"

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

//-------------------------------------------------------------------------------

using namespace std;
using namespace VirtualRobot;
using namespace GraspStudio;

//-------------------------------------------------------------------------------

GraspPlannerWindow::GraspPlannerWindow(std::string &robFile,
                                       std::string &eefName,
                                       std::string &preshape,
                                       VirtualRobot::TriMeshModelPtr triMeshModel,
                                       Qt::WFlags flags)
  : QMainWindow(NULL),
    grasp_counter_(0)
{
  VR_INFO << " Start GraspPlannerWindow " << endl;

  // init the random number generator
  srand(time(NULL));

  this->robotFile_ = robFile;
  this->eefName_ = eefName;
  this->preshape_ = preshape;
  eefVisu_ = NULL;

  sceneSep_ = new SoSeparator;
  sceneSep_->ref();
  robotSep_ = new SoSeparator;
  objectSep_ = new SoSeparator;
  sphereSep_ = new SoSeparator;
  frictionConeSep_ = new SoSeparator;
  graspsSep_ = new SoSeparator;
  graspsSep_->ref();

#if 0
  SoSeparator *s = CoinVisualizationFactory::CreateCoordSystemVisualization();
  sceneSep_->addChild(s);
#endif
  sceneSep_->addChild(robotSep_);
  sceneSep_->addChild(objectSep_);
  sceneSep_->addChild(sphereSep_);
  sceneSep_->addChild(frictionConeSep_);

  setupUI();

  loadRobot();
  loadObject(triMeshModel);

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
#ifdef WIN32
  viewer_->setAntialiasing(true, 8);
#endif
  viewer_->setGLRenderAction(new SoLineHighlightRenderAction);
  viewer_->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
  viewer_->setFeedbackVisibility(true);
  viewer_->setSceneGraph(sceneSep_);
  viewer_->viewAll();

  connect(UI_.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
  connect(UI_.pushButtonPlan, SIGNAL(clicked()), this, SLOT(plan()));
  connect(UI_.pushButtonSave, SIGNAL(clicked()), this, SLOT(save()));
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
  std::cout << "GraspPlannerWindow: Closing" << std::endl;
  this->close();
  SoQt::exitMainLoop();
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::loadObject(const shape_msgs::Mesh& obj_mesh)
{
  TriMeshModelPtr triMeshModel = MeshObstacle::create_tri_mesh(obj_mesh);
  this->loadObject(triMeshModel);
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::loadObject(VirtualRobot::TriMeshModelPtr triMeshModel)
{
  viewer_->lock();

  const bool show_normals = true;
  object_ = MeshObstacle::create_mesh_obstacle(triMeshModel, !show_normals);

  Eigen::Vector3f minS, maxS;
  object_->getCollisionModel()->getTriMeshModel()->getSize(minS, maxS);
  cout << "TriMeshModel minS:\n" << minS << endl;
  cout << "TriMeshModel MaxS:\n" << maxS << endl;

  qualityMeasure_.reset(new GraspStudio::GraspQualityMeasureWrenchSpace(object_));
  // qualityMeasure_->setVerbose(true);
  qualityMeasure_->calculateObjectProperties();

  approach_.reset(new SrApproachMovementSurfaceNormal(object_, eef_));
  eefCloned_ = approach_->getEEFRobotClone();
  if (robot_ && eef_)
  {
    std::string name = "Grasp Planner - ";
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

void GraspPlannerWindow::plan(boost::shared_ptr<sr_grasp_msgs::GraspMeshFeedback> feedback_mesh,
                              boost::shared_ptr<sr_grasp_msgs::GraspMeshResult> result_mesh)
{
  feedback_mesh_ = feedback_mesh;
  result_mesh_ = result_mesh;
  this->resetSceneryAll();
  this->plan();
}

//-------------------------------------------------------------------------------

void GraspPlannerWindow::plan()
{
  viewer_->lock();

  // Start!
  clock_t begin = clock();

  float timeout = UI_.spinBoxTimeOut->value() * 1000.0f;
  bool forceClosure = UI_.checkBoxFoceClosure->isChecked();
  float quality = static_cast<float>(UI_.doubleSpinBoxQuality->value());
  int nrDesiredGrasps = UI_.spinBoxGraspNumber->value();

  planner_.reset(new GraspStudio::GenericGraspPlanner(grasps_,
                                                      qualityMeasure_,
                                                      approach_,
                                                      quality,
                                                      forceClosure));

  int nrComputedGrasps = planner_->plan(nrDesiredGrasps, timeout);
  grasps_->setPreshape(preshape_);
  for (int i=0; i<static_cast<int>(grasps_->getSize()); i++)
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

  for (int i=0; i<static_cast<int>(grasps_->getSize()); i++)
  {
    // m is the pose of the grasp applied to the global object pose,
    // resulting in the global TCP pose which is related to the grasp.
    Eigen::Matrix4f m = grasps_->getGrasp(i)->getTcpPoseGlobal(object_->getGlobalPose());

    moveit_msgs::Grasp grasp_msg;

    // nrComputedGrasps should be one.
    feedback_mesh_->no_of_stable_grasps += nrComputedGrasps;

    // A name for this grasp.
    grasp_msg.id = std::string("grasp_") + boost::lexical_cast<std::string>(grasp_counter_);
    grasp_counter_++;

    // The position of the end-effector for the grasp.
    grasp_msg.grasp_pose.header.stamp = ros::Time::now();

    // The internal posture of the hand for the grasp.
    // positions and efforts (not set here) are used.
    trajectory_msgs::JointTrajectory grasp_posture;
    grasp_posture.header.stamp = ros::Time::now();
    // Get the configuration of the grasp.
    std::map<std::string, float> config= grasps_->getGrasp(i)->getConfiguration();
    std::map<std::string, float>::const_iterator iter = config.begin();
    while (iter != config.end())
    {
      std::string node_name = iter->first;
      float value = iter->second; // Unit is radian.
      // Set joint name.
      grasp_posture.joint_names.push_back(node_name);
      // Set position (i.e., angle).
      trajectory_msgs::JointTrajectoryPoint jtPoint;
      jtPoint.positions.push_back(value);
      grasp_posture.points.push_back(jtPoint);
      iter++;
    }
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

    // Here we choose between poseTcp and poseGlobal to save.
    Eigen::Matrix4f poseToSave = poseGlobal;
    // Set pose.position.
    grasp_msg.grasp_pose.pose.position.x = poseToSave(0,3);
    grasp_msg.grasp_pose.pose.position.y = poseToSave(1,3);
    grasp_msg.grasp_pose.pose.position.z = poseToSave(2,3);
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
  cout << "Grasp planning took " << static_cast<double>(diffclock(end, begin)) << " ms."<< endl;

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

    std::stringstream ss;
    ss << std::setprecision(3);
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
  std::string objectFile = std::string(fi.toAscii());
  bool ok = false;
  try
  {
    ok = ObjectIO::saveManipulationObject(objectM, objectFile);
  }
  catch (VirtualRobotException &e)
  {
    cout << " ERROR while saving object" << endl;
    cout << e.what();
    return;
  }

  if (!ok)
  {
    cout << " ERROR while saving object" << endl;
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
