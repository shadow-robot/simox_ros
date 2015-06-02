/*
 * Copyright (c) 2013 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

/**
 * @file   urdf_to_simox_xml.cpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  Convert from a hand description in URDF format to Simox XML.
 **/

//-------------------------------------------------------------------------------

#include "urdf_to_simox_xml/urdf_to_simox_xml.hpp"

#include <exception>
#include <iomanip>
#include <iostream>
#include <set>
#include <string>
#include <sstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/iter_find.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/lexical_cast.hpp>
#include <ros/console.h>
#include <ros/package.h>

//-------------------------------------------------------------------------------

using namespace gsc;
using boost::property_tree::ptree;

//-------------------------------------------------------------------------------

const std::string UrdfToSimoxXml::model_dir_name_ = std::string("/model");

// The name of the robot node inside dms.urdf.
// Will be used later when checking which hand model we are actually parsing.
const std::string UrdfToSimoxXml::robot_name_in_dms_urdf_ = std::string("four_dof_thumb");

// The name of the robot ndoe inside shadowhand.urdf.
// Will be used later when checking which hand model we are actually parsing.
const std::string UrdfToSimoxXml::robot_name_in_shadowhand_urdf_ = std::string("shadowhand");

//-------------------------------------------------------------------------------

UrdfToSimoxXml::UrdfToSimoxXml(const bool urdf_init_param,
                               const std::string urdf_file,
                               const std::string output_dir,
                               const double scale)
  : urdf_model_(new urdf::Model()),
    output_dir_(output_dir),
    scale_(scale)
{
  // Init Inventor.
  const char input[] = "UrdfToSimoxXml";
  window_ = SoQt::init(input);
  if (window_ == NULL)
  {
    ROS_ERROR_STREAM("Failed to init Inventor.");
    exit (EXIT_FAILURE);
  }

  if (urdf_init_param)
  {
    // Parse the the robot_description parameter and then construct the model.
    const std::string rd_param("robot_description");
    if (!urdf_model_->initParam(rd_param))
    {
      ROS_ERROR_STREAM("Failed to parse param " << rd_param << ".");
      exit (EXIT_FAILURE);
    }
  }
  else
  {
    // Parse the URDF file and then construct the model.
    if (!urdf_model_->initFile(urdf_file))
    {
      ROS_ERROR_STREAM("Failed to parse urdf file " << urdf_file << ".");
      exit (EXIT_FAILURE);
    }
  }

  // Get all materials in the model.
  // Simox does not support materials though.
  materials_ = urdf_model_->materials_;

  // Get all links in the model.
  urdf_model_->getLinks(links_);
  if (links_.empty())
  {
    ROS_ERROR_STREAM("There are no links in " << urdf_file << ".");
    exit (EXIT_FAILURE);
  }

  // Set the base link.
  this->set_base_link_();

  // Get all joints in the model.
  joints_.clear();
  BOOST_FOREACH(boost::shared_ptr<urdf::Link> link, links_)
  {
    std::vector< boost::shared_ptr<urdf::Joint> > child_joints;
    child_joints = link->child_joints;
    BOOST_FOREACH(boost::shared_ptr<urdf::Joint> child_joint, child_joints)
    {
      joints_.push_back(child_joint);
    }
  }

  // Sort the joints by their names.
  std::sort(joints_.begin(), joints_.end(), UrdfToSimoxXml::compareUrdfJoint);
}

//-------------------------------------------------------------------------------

UrdfToSimoxXml::~UrdfToSimoxXml()
{
}

//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

void UrdfToSimoxXml::write_xml(const std::string& output_dir,
                               const std::string& simox_xml_filename)
{
  // No link has been converted to Simox.
  simox_links_.clear();

  // Obtain the name of the hand from simox_xml_filename.
  std::list<std::string> stringList;
  boost::iter_split(stringList, simox_xml_filename, boost::first_finder("."));
  if (stringList.size() != 2)
  {
    ROS_ERROR_STREAM(simox_xml_filename << " should be something like dms.xml or shadowhand.xml.");
    exit (EXIT_FAILURE);
  }

  std::string hand_name(stringList.front());
  std::string hand_name_upper_case = boost::to_upper_copy(hand_name);
  std::string hand_name_lower_case = boost::to_lower_copy(hand_name);

  // Create empty property tree object
  ptree pt;

  // TCP = Tool Center Point
  //   TCP is the point in relation to which all robot positioning is defined.
  // GCP = Grasp Center Point.
  //   GCP defines the favorite grasping position and an approach direction.
  std::string hand_base(hand_name_lower_case + "_hand_base");
  std::string hand_tcp(hand_name_lower_case + "_hand_tcp");
  std::string hand_gcp(hand_name_lower_case + "_hand_gcp");
  std::string base_link(base_link_->name);

  // Create the ${hand_name_upper_case} node.
  boost::property_tree::ptree hand_node;
  hand_node.put("<xmlattr>.Type", hand_name_upper_case);
  hand_node.put("<xmlattr>.RootNode", hand_base);

  // Add RobotNode name="${hand_name_lower_case}_hand_base".
  this->add_hand_base_node_(hand_node,
                            hand_base,
                            hand_tcp,
                            hand_gcp,
                            base_link);

  // Add RobotNode name="${hand_name_lower_case}_hand_tcp".
  this->add_hand_tcp_node_(hand_node, hand_tcp);

  // Add RobotNode name="${hand_name_lower_case}_hand_gcp".
  this->add_hand_gcp_node_(hand_node, hand_gcp);

  // Add RobotNode for the base link.
  this->add_link_node_(hand_node, base_link_);

  // Add Endeffector name="${hand_name_upper_case}" base="${hand_name_lower_case}_hand_base"
  // tcp="${hand_name_lower_case}_hand_tcp" gcp="${hand_name_lower_case}_hand_gcp".
  this->add_endeffector_node_(hand_node,
                              hand_name_upper_case,
                              hand_base,
                              hand_tcp,
                              hand_gcp,
                              base_link);

  // Add RobotNodeSet name="${hand_name_upper_case} Joints".
  this->add_hand_joints_node_(hand_node, hand_name_upper_case);

  // Add the ${hand_name_upper_case} to the tree.
  pt.add_child("Robot", hand_node);

  // Write property tree to XML file
  // http://stackoverflow.com/questions/6572550/boostproperty-tree-xml-pretty-printing
  std::string simox_xml_file = output_dir + "/" + simox_xml_filename;
  boost::property_tree::xml_writer_settings<char> settings('\t', 1);
  boost::property_tree::write_xml(simox_xml_file, pt, std::locale(), settings);
}

//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

void UrdfToSimoxXml::add_hand_base_node_(boost::property_tree::ptree & hand_node,
                                         const std::string & hand_base,
                                         const std::string & hand_tcp,
                                         const std::string & hand_gcp,
                                         const std::string & base_link)
{
  boost::property_tree::ptree hand_base_node;
  hand_base_node.put("<xmlattr>.name", hand_base);

  boost::property_tree::ptree Child_node_1;
  boost::property_tree::ptree Child_node_2;
  boost::property_tree::ptree Child_node_3;
  Child_node_1.put("<xmlattr>.name", hand_tcp);
  Child_node_2.put("<xmlattr>.name", hand_gcp);
  Child_node_3.put("<xmlattr>.name", base_link);

  hand_base_node.add_child("Child", Child_node_1);
  hand_base_node.add_child("Child", Child_node_2);
  hand_base_node.add_child("Child", Child_node_3);

  hand_node.add_child("RobotNode", hand_base_node);
}

//-------------------------------------------------------------------------------

void UrdfToSimoxXml::add_hand_tcp_node_(boost::property_tree::ptree & hand_node,
                                        const std::string & hand_tcp)
{
  boost::property_tree::ptree hand_tcp_node;
  hand_tcp_node.put("<xmlattr>.name", hand_tcp);
  hand_tcp_node.put("<xmlcomment>", "Translation values were set manually!");

  boost::property_tree::ptree Translation_node;

  const std::string model_name = urdf_model_->getName();
  if (model_name.compare(robot_name_in_dms_urdf_) == 0) // DMS Hand
  {
    this->set_translation_node_(Translation_node, 0.0, 0.0, 0.12);
  }
  else if (model_name.compare(robot_name_in_shadowhand_urdf_) == 0) // Shadow Hand
  {
    this->set_translation_node_(Translation_node, 0.0, 0.0, 0.375);
  }
  else // Other Hand
  {
    this->set_translation_node_(Translation_node, 0.0, 0.0, 0.0);
  }

  boost::property_tree::ptree Transform_node;
  Transform_node.add_child("Translation", Translation_node);
  hand_tcp_node.add_child("Transform", Transform_node);
  hand_node.add_child("RobotNode", hand_tcp_node);
}

//-------------------------------------------------------------------------------

void UrdfToSimoxXml::add_hand_gcp_node_(boost::property_tree::ptree & hand_node,
                                        const std::string & hand_gcp)
{
  boost::property_tree::ptree hand_gcp_node;
  hand_gcp_node.put("<xmlattr>.name", hand_gcp);
  hand_gcp_node.put("<xmlcomment>", "Translation and rollpitchyaw values were set manually!");

  boost::property_tree::ptree Translation_node;
  boost::property_tree::ptree rollpitchyaw_node;

  const std::string model_name = urdf_model_->getName();
  if (model_name.compare(robot_name_in_dms_urdf_) == 0) // DMS Hand
  {
    this->set_translation_node_(Translation_node, -0.01, -0.035, 0.07);
    this->set_rollpitchyaw_node_(rollpitchyaw_node, 1.0, 0.0, 0.0);
  }
  else if (model_name.compare(robot_name_in_shadowhand_urdf_) == 0) // Shadow Hand
  {
    this->set_translation_node_(Translation_node, 0.0, -0.05, 0.30);
    this->set_rollpitchyaw_node_(rollpitchyaw_node, 0.75, 0.0, 0.0);
  }
  else // Other Hand
  {
    this->set_translation_node_(Translation_node, 0.0, 0.0, 0.0);
    this->set_rollpitchyaw_node_(rollpitchyaw_node, 1.0, 0.0, 0.0);
  }

  boost::property_tree::ptree Transform_node;
  Transform_node.add_child("Translation", Translation_node);
  Transform_node.add_child("rollpitchyaw", rollpitchyaw_node);
  hand_gcp_node.add_child("Transform", Transform_node);
  hand_node.add_child("RobotNode", hand_gcp_node);
}

//-------------------------------------------------------------------------------

std::string UrdfToSimoxXml::parse_geometry(boost::shared_ptr<const urdf::Link> link,
                                           boost::shared_ptr<urdf::Geometry> geometry)
{
  std::string simox_filename;

  if (geometry->type == urdf::Geometry::MESH)
  {
    boost::shared_ptr<urdf::Mesh> mesh = boost::dynamic_pointer_cast<urdf::Mesh>(geometry);

    // Note that variable scale_ may be reset inside method convert_mesh_.
    simox_filename = this->convert_mesh_(mesh->filename);

    // <mesh filename="package://sr_grasp_description/meshes/TH3_z.dae" scale="0.1 0.1 0.1" />
    const double scale_x = mesh->scale.x * scale_;
    const double scale_y = mesh->scale.y * scale_;
    const double scale_z = mesh->scale.z * scale_;
    if (scale_x != 1.0 || scale_y != 1.0 || scale_z != 1.0)
      this->scale_wrl_scene_(simox_filename, scale_x, scale_y, scale_z);
  }
  else if (geometry->type == urdf::Geometry::SPHERE)
  {
    boost::shared_ptr<urdf::Sphere> sphere = boost::dynamic_pointer_cast<urdf::Sphere>(geometry);
    simox_filename = this->convert_sphere_(link->name, sphere->radius);
  }
  else if (geometry->type == urdf::Geometry::BOX)
  {
    boost::shared_ptr<urdf::Box> box = boost::dynamic_pointer_cast<urdf::Box>(geometry);

    std::string urdf_filename;
    simox_filename = this->convert_cube_(link->name, box->dim.x, box->dim.y, box->dim.z);
  }
  else if (geometry->type == urdf::Geometry::CYLINDER)
  {
    boost::shared_ptr<urdf::Cylinder> cylinder = boost::dynamic_pointer_cast<urdf::Cylinder>(geometry);

    std::string urdf_filename;
    simox_filename = this->convert_cylinder_(link->name, cylinder->length, cylinder->radius);
  }
  else
  {
    ROS_ERROR_STREAM("SPHERE, BOX, CYLINDER, MESH are the 4 supported urdf::Geometry types.");
    exit (EXIT_FAILURE);
  }

  return simox_filename;
}

//-------------------------------------------------------------------------------

void UrdfToSimoxXml::add_visual_node(boost::shared_ptr<urdf::Visual> visual,
                                     boost::shared_ptr<const urdf::Link> link,
                                     boost::property_tree::ptree &link_node)
{
  urdf::Pose pose = visual->origin;

  boost::property_tree::ptree Translation_node;
  this->set_translation_node_(Translation_node, pose.position);

  boost::property_tree::ptree rollpitchyaw_node;
  this->set_rollpitchyaw_node_(rollpitchyaw_node, pose.rotation);

  boost::property_tree::ptree Transform_node;
  Transform_node.add_child("Translation", Translation_node);
  Transform_node.add_child("rollpitchyaw", rollpitchyaw_node);
  link_node.add_child("Transform", Transform_node);

  // *** Parse the visual node ***
  // Note that collision node is NOT parsed, because inside URDF,
  // a visual node and a collision node can have different positions
  // and orientations. But in Simox XML files, the two nodes share
  // the same position and the same orientation.
  std::string simox_visua_filename;
  std::string simox_colli_filename;
  boost::shared_ptr<urdf::Geometry> geometry = visual->geometry;
  if (geometry)
  {
    simox_visua_filename = this->parse_geometry(link, geometry);
    simox_colli_filename = simox_visua_filename;
  }

  // Add the visualization node.
  if (!simox_visua_filename.empty())
  {
    boost::property_tree::ptree Visualization_File_node;
    Visualization_File_node.put("<xmlattr>.type", "Inventor");
    Visualization_File_node.put("<xmltext>", simox_visua_filename);

    boost::property_tree::ptree Visualization_node;
    Visualization_node.put("<xmlattr>.enable", "true");
    Visualization_node.add_child("File", Visualization_File_node);
    link_node.add_child("Visualization", Visualization_node);
  }

  // Add the collision model node.
  if (!simox_colli_filename.empty())
  {
    // Note that the collision model node is identical to the visualization node.
    boost::property_tree::ptree CollisionModel_File_node;
    CollisionModel_File_node.put("<xmlattr>.type", "Inventor");
    CollisionModel_File_node.put("<xmltext>", simox_colli_filename);

    boost::property_tree::ptree CollisionModel_node;
    CollisionModel_node.add_child("File", CollisionModel_File_node);
    link_node.add_child("CollisionModel", CollisionModel_node);
  }
}

//-------------------------------------------------------------------------------

void UrdfToSimoxXml::add_link_node_(boost::property_tree::ptree & hand_node,
                                    boost::shared_ptr<const urdf::Link> link)
{
  simox_links_.push_back(link);

  boost::property_tree::ptree link_node;
  link_node.put("<xmlattr>.name", link->name);

  // Add the visual node.
  boost::shared_ptr<urdf::Visual> visual = link->visual;
  if (visual)
    this->add_visual_node(visual, link, link_node);

  std::vector< boost::shared_ptr<urdf::Joint> > child_joints;
  child_joints = link->child_joints;
  BOOST_FOREACH(boost::shared_ptr<const urdf::Joint> child_joint, child_joints)
  {
    boost::property_tree::ptree Child_node;
    Child_node.put("<xmlattr>.name", child_joint->name);
    link_node.add_child("Child", Child_node);
  }

  hand_node.add_child("RobotNode", link_node);

  // Add the child joints.
  BOOST_FOREACH(boost::shared_ptr<const urdf::Joint> child_joint, child_joints)
  {
    this->add_joint_node_(hand_node, child_joint);
  }
}

//-------------------------------------------------------------------------------

void UrdfToSimoxXml::add_joint_node_(boost::property_tree::ptree & hand_node,
                                     boost::shared_ptr<const urdf::Joint> child_joint)
{
  simox_joints_.push_back(child_joint);

  boost::property_tree::ptree child_joint_node;
  child_joint_node.put("<xmlattr>.name", child_joint->name);

  urdf::Pose pose = child_joint->parent_to_joint_origin_transform;

  boost::property_tree::ptree Translation_node;
  this->set_translation_node_(Translation_node, pose.position);

  boost::property_tree::ptree rollpitchyaw_node;
  this->set_rollpitchyaw_node_(rollpitchyaw_node, pose.rotation);

  boost::property_tree::ptree Transform_node;
  Transform_node.add_child("Translation", Translation_node);
  Transform_node.add_child("rollpitchyaw", rollpitchyaw_node);
  child_joint_node.add_child("Transform", Transform_node);

  if (child_joint->type == urdf::Joint::REVOLUTE)
  {
    boost::property_tree::ptree Axis_node;
    this->set_axis_node_(Axis_node, child_joint->axis);

    boost::property_tree::ptree Limits_node;
    this->set_joint_limits_node_(Limits_node, child_joint->limits);

    boost::property_tree::ptree Joint_node;
    Joint_node.put("<xmlattr>.type", "revolute");
    Joint_node.add_child("Axis", Axis_node);
    Joint_node.add_child("Limits", Limits_node);
    child_joint_node.add_child("Joint", Joint_node);
  }
  else if (child_joint->type == urdf::Joint::FIXED)
  {
    boost::property_tree::ptree Joint_node;
    Joint_node.put("<xmlattr>.type", "fixed");
    child_joint_node.add_child("Joint", Joint_node);
  }
  else
  {
    ROS_ERROR_STREAM("Only revolute and fixed joints are support at moment.");
    exit (EXIT_FAILURE);
  }

  boost::property_tree::ptree Child_node;
  boost::shared_ptr<const urdf::Link> child_link = urdf_model_->getLink(child_joint->child_link_name);
  Child_node.put("<xmlattr>.name", child_link->name);
  // Add to child_joint_node iff child_link contains a visual node.
  if (child_link->visual)
    child_joint_node.add_child("Child", Child_node);

  hand_node.add_child("RobotNode", child_joint_node);

  // Continue with the child link iff it contains a visual node.
  if (child_link->visual)
    this->add_link_node_(hand_node, child_link);
}

//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

void UrdfToSimoxXml::add_endeffector_node_(boost::property_tree::ptree & hand_node,
                                           const std::string & hand_name_upper_case,
                                           const std::string & hand_base,
                                           const std::string & hand_tcp,
                                           const std::string & hand_gcp,
                                           const std::string & base_link)
{
  boost::property_tree::ptree Endeffector_node;
  Endeffector_node.put("<xmlcomment>", "This node is for Simox (e.g., GraspPlanner in Simox)!");
  Endeffector_node.put("<xmlattr>.name", hand_name_upper_case);
  Endeffector_node.put("<xmlattr>.base", hand_base);
  Endeffector_node.put("<xmlattr>.tcp", hand_tcp);
  Endeffector_node.put("<xmlattr>.gcp", hand_gcp);

  boost::property_tree::ptree Preshape_node;
  Preshape_node.put("<xmlattr>.name", "Grasp Preshape");
  Preshape_node.put("<xmlcomment>", "This is just a template. Please set values manually!");

  BOOST_FOREACH(boost::shared_ptr<const urdf::Joint> joint, joints_)
  {
    boost::property_tree::ptree Node_node;
    Node_node.put("<xmlattr>.name", joint->name);
    Node_node.put("<xmlattr>.unit", "radian");
    Node_node.put("<xmlattr>.value", "0.0");
    Preshape_node.add_child("Node", Node_node);
  }
  Endeffector_node.add_child("Preshape", Preshape_node);

  {
    boost::property_tree::ptree Node_node;
    Node_node.put("<xmlattr>.name", base_link);
    boost::property_tree::ptree Static_node;
    Static_node.add_child("Node", Node_node);
    Endeffector_node.add_child("Static", Static_node);
  }

  /*
   * Assume that the first characters of the names of the joints
   * and the names of the links that belong to one finger are unique,
   * and can be used to identify the finger.
   */
  std::map<int, bool> actors;
  this->get_actors(actors);
  for(std::map<int,bool>::iterator iter = actors.begin(); iter != actors.end(); ++iter)
  {
    char actor_name = toupper(static_cast<char>(iter->first));

    boost::property_tree::ptree Actor_node;
    Actor_node.put("<xmlattr>.name", actor_name);
    Actor_node.put("<xmlcomment>", "This is just a template. Please set values manually!");
    Actor_node.put("<xmlcomment>", "Note that considerCollisions = None, Actors, or All!");

    BOOST_FOREACH(boost::shared_ptr<const urdf::Link> link, simox_links_)
    {
      const char& first_char = link->name.at(0);
      if (actor_name == toupper(first_char))
      {
        boost::property_tree::ptree Node_node;
        Node_node.put("<xmlattr>.name", link->name);
        Node_node.put("<xmlattr>.considerCollisions", "None");
        Actor_node.add_child("Node", Node_node);
      }
    }

    BOOST_FOREACH(boost::shared_ptr<const urdf::Joint> joint, simox_joints_)
    {
      const char& first_char = joint->name.at(0);
      if (actor_name == toupper(first_char))
      {
        boost::property_tree::ptree Node_node;
        Node_node.put("<xmlattr>.name", joint->name);
        Node_node.put("<xmlattr>.considerCollisions", "None");
        Actor_node.add_child("Node", Node_node);
      }
    }

    Endeffector_node.add_child("Actor", Actor_node);
  }

  hand_node.add_child("Endeffector", Endeffector_node);
}

void UrdfToSimoxXml::add_hand_joints_node_(boost::property_tree::ptree & hand_node,
                                           const std::string & hand_name_upper_case)
{
  boost::property_tree::ptree hand_joints_node;
  hand_joints_node.put("<xmlcomment>", "This node is for Simox (e.g., GraspPlanner in Simox)!");
  hand_joints_node.put("<xmlattr>.name", hand_name_upper_case + " Joints");

  BOOST_FOREACH(boost::shared_ptr<const urdf::Joint> joint, joints_)
  {
    boost::property_tree::ptree Node_node;
    Node_node.put("<xmlattr>.name", joint->name);
    hand_joints_node.add_child("Node", Node_node);
  }

  hand_node.add_child("RobotNodeSet", hand_joints_node);
}

//-------------------------------------------------------------------------------

// Once the keys are converted back to char, actors contain 'f', 'l', 'm', 't'.
void UrdfToSimoxXml::get_actors(std::map<int, bool>& actors)
{
  actors.clear();
  BOOST_FOREACH(boost::shared_ptr<const urdf::Joint> joint, simox_joints_)
  {
    const char& first_char = joint->name.at(0);
    const int first_int = toupper(static_cast<int>(first_char));
    if (actors.find(first_int) == actors.end())
      actors[first_int] = true;
  }
}

//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

void UrdfToSimoxXml::set_joint_limits_node_(boost::property_tree::ptree & Limits_node,
                                            boost::shared_ptr<urdf::JointLimits> limits)
{
  Limits_node.put("<xmlattr>.unit", "radian");
  Limits_node.put("<xmlattr>.lo", this->to_string_(limits->lower));
  Limits_node.put("<xmlattr>.hi", this->to_string_(limits->upper));
}

//-------------------------------------------------------------------------------

void UrdfToSimoxXml::set_axis_node_(boost::property_tree::ptree & Axis_node,
                                    urdf::Vector3 axis)
{
  this->set_axis_node_(Axis_node,
                       axis.x,
                       axis.y,
                       axis.z);
}

//-------------------------------------------------------------------------------

void UrdfToSimoxXml::set_axis_node_(boost::property_tree::ptree & Axis_node,
                                    double x, double y, double z)
{
  Axis_node.put("<xmlattr>.x", this->to_string_(x));
  Axis_node.put("<xmlattr>.y", this->to_string_(y));
  Axis_node.put("<xmlattr>.z", this->to_string_(z));
}

//-------------------------------------------------------------------------------

void UrdfToSimoxXml::set_translation_node_(boost::property_tree::ptree & Translation_node,
                                           urdf::Vector3 position)
{
  this->set_translation_node_(Translation_node,
                              position.x,
                              position.y,
                              position.z);
}

//-------------------------------------------------------------------------------

void UrdfToSimoxXml::set_translation_node_(boost::property_tree::ptree & Translation_node,
                                           double x, double y, double z)
{
  Translation_node.put("<xmlattr>.x", this->to_string_(x));
  Translation_node.put("<xmlattr>.y", this->to_string_(y));
  Translation_node.put("<xmlattr>.z", this->to_string_(z));
  Translation_node.put("<xmlattr>.unitsLength", "m");
}

//-------------------------------------------------------------------------------

void UrdfToSimoxXml::set_rollpitchyaw_node_(boost::property_tree::ptree & Translation_node,
                                            urdf::Rotation rotation)
{
  double roll, pitch, yaw;
  rotation.getRPY(roll, pitch, yaw);
  this->set_rollpitchyaw_node_(Translation_node,
                               roll,
                               pitch,
                               yaw);
}

//-------------------------------------------------------------------------------

void UrdfToSimoxXml::set_rollpitchyaw_node_(boost::property_tree::ptree & Translation_node,
                                            double roll, double pitch, double yaw)
{
  Translation_node.put("<xmlattr>.roll", this->to_string_(roll));
  Translation_node.put("<xmlattr>.pitch", this->to_string_(pitch));
  Translation_node.put("<xmlattr>.yaw", this->to_string_(yaw));
  Translation_node.put("<xmlattr>.unitsAngle", "radian");
}

//-------------------------------------------------------------------------------

// Write the scene to a iv file.
std::string UrdfToSimoxXml::write_to_iv_file_(const std::string & file_name,
                                       SoSeparator *scene_with_shape)
{
  std::string simox_filename;
  std::string model_dir = output_dir_ + UrdfToSimoxXml::model_dir_name_;
  if (!boost::filesystem::exists(model_dir))
    boost::filesystem::create_directories(model_dir);
  simox_filename = model_dir + "/" + file_name;

  SoWriteAction writeAction;
  writeAction.getOutput()->openFile(simox_filename.c_str());
  writeAction.getOutput()->setBinary(FALSE);
  writeAction.apply(scene_with_shape);
  writeAction.getOutput()->closeFile();

  return simox_filename;
}

//-------------------------------------------------------------------------------

std::string UrdfToSimoxXml::convert_cube_(const std::string & link_name,
                                          const double & width,
                                          const double & height,
                                          const double & depth)
{
  // Make a scene containing a cube.
  SoSeparator *cube_scene = new SoSeparator;
  SoUnits *cube_units = new SoUnits;
  SoMaterial *cube_color = new SoMaterial;
  SoTranslation *cube_trans = new SoTranslation;
  SoCube *cube = new SoCube;
  cube_scene->ref();
  cube_units->units = SoUnits::METERS;
  cube_scene->addChild(cube_units);
  cube_color->diffuseColor.setValue(1.0, 1.0, 1.0);
  cube_scene->addChild(cube_color);
  SoSFVec3f cube_trans_vec;
  cube_trans_vec.setValue(0.0, 0.0, 0.0);
  cube_trans->translation = cube_trans_vec;
  cube_scene->addChild(cube_trans);
  cube->width  = width;
  cube->height = height;
  cube->depth  = depth;
  cube_scene->addChild(cube);

  // Write the scene to a iv file.
  std::string cube_name(link_name + "_cube.iv");
  std::string simox_filename = this->write_to_iv_file_(cube_name, cube_scene);
  return simox_filename;
}

//-------------------------------------------------------------------------------

std::string UrdfToSimoxXml::convert_cylinder_(const std::string & link_name,
                                              const double & height,
                                              const double & radius)
{
  // Make a scene containing a cylinder.
  SoSeparator *cylinder_scene = new SoSeparator;
  SoUnits *cylinder_units = new SoUnits;
  SoMaterial *cylinder_color = new SoMaterial;
  SoTranslation *cylinder_trans = new SoTranslation;
  SoCylinder *cylinder = new SoCylinder;
  cylinder_scene->ref();
  cylinder_units->units = SoUnits::METERS;
  cylinder_scene->addChild(cylinder_units);
  cylinder_color->diffuseColor.setValue(1.0, 1.0, 1.0);
  cylinder_scene->addChild(cylinder_color);
  SoSFVec3f cylinder_trans_vec;
  cylinder_trans_vec.setValue(0.0, 0.0, 0.0);
  cylinder_trans->translation = cylinder_trans_vec;
  cylinder_scene->addChild(cylinder_trans);
  cylinder->height = height;
  cylinder->radius = radius;
  cylinder_scene->addChild(cylinder);

  // Write the scene to a iv file.
  std::string cylinder_name(link_name + "_cylinder.iv");
  std::string simox_filename = this->write_to_iv_file_(cylinder_name, cylinder_scene);
  return simox_filename;
}

//-------------------------------------------------------------------------------

std::string UrdfToSimoxXml::convert_sphere_(const std::string & link_name,
                                            const double & radius)
{
  // Make a scene containing a sphere.
  SoSeparator *sphere_scene = new SoSeparator;
  SoUnits *sphere_units = new SoUnits;
  SoMaterial *sphere_color = new SoMaterial;
  SoTranslation *sphere_trans = new SoTranslation;
  SoSphere *sphere = new SoSphere;
  sphere_scene->ref();
  sphere_units->units = SoUnits::METERS;
  sphere_scene->addChild(sphere_units);
  sphere_color->diffuseColor.setValue(1.0, 1.0, 1.0);
  sphere_scene->addChild(sphere_color);
  SoSFVec3f sphere_trans_vec;
  sphere_trans_vec.setValue(0.0, 0.0, 0.0);
  sphere_trans->translation = sphere_trans_vec;
  sphere_scene->addChild(sphere_trans);
  sphere->radius = radius;
  sphere_scene->addChild(sphere);

  // Write the scene to a iv file.
  std::string sphere_name(link_name + "_sphere.iv");
  std::string simox_filename = this->write_to_iv_file_(sphere_name, sphere_scene);
  return simox_filename;
}

//-------------------------------------------------------------------------------

// urdf_filename looks like "package://dms_description/meshes/base_link.STL"
std::string UrdfToSimoxXml::convert_mesh_(const std::string & urdf_filename)
{
  std::string urdf_filename_copy(urdf_filename);
  std::string packagePrefix("package://");
  size_t pos1 = urdf_filename_copy.find(packagePrefix, 0);
  if (pos1 != std::string::npos)
  {
    size_t repLen = packagePrefix.size();
    urdf_filename_copy.erase(pos1, repLen);
  }
  else
  {
    ROS_ERROR_STREAM("The prefix of " << urdf_filename << " is NOT package://.");
    exit (EXIT_FAILURE);
  }

  std::list<std::string> stringList;
  boost::iter_split(stringList, urdf_filename_copy, boost::first_finder("/"));
  if (stringList.size() < 2)
  {
    ROS_ERROR_STREAM(urdf_filename << " is either empty or too short.");
    exit (EXIT_FAILURE);
  }

  std::string package_name = stringList.front();
  std::string original_filename = ros::package::getPath(package_name);
  unsigned short n = 0;
  BOOST_FOREACH(std::string token, stringList)
  {
    if (n++ != 0)
      original_filename += ("/" + token);
  }

  // Is the input file a .dae file?
  // Note that pos must be defined as std::size_t, not unsigned int.
  std::size_t pos = original_filename.find(".dae");
  if (pos != std::string::npos)
  {
    std::string dot_str = original_filename.substr(pos);
    if (dot_str.compare(".dae") == 0)
      this->read_dae_file_(original_filename);
  }

  std::string simox_filename;
  size_t sp = urdf_filename.find_first_of( '/' );
  if ( sp != std::string::npos ) {
    sp = urdf_filename.find_last_of( '/' );
    if ( sp != std::string::npos ) {
      std::string last_part( urdf_filename.begin()+sp+1, urdf_filename.end() );
      simox_filename = last_part;
    }
  }
  // Convert from for example "base_link.STL" to "base_link.wrl".
  simox_filename = simox_filename.substr(0, simox_filename.find_first_of('.'));
  simox_filename.append(".wrl");

  std::string model_dir = output_dir_ + UrdfToSimoxXml::model_dir_name_;
  if (!boost::filesystem::exists(model_dir))
    boost::filesystem::create_directories(model_dir);

  // Call meshlabserver to convert meshes to wrl.
  // http://en.wikipedia.org/wiki/VRML
  std::stringstream stream;
  simox_filename = model_dir + "/" + simox_filename;
  stream <<"meshlabserver -i " << original_filename << " -o " << simox_filename;
  FILE *fp = popen(stream.str().c_str(), "r");

  // Look for error meshlabserver messages.
  char * line = NULL;
  size_t len = 0;
  while (getline(&line, &len, fp) != -1) {
    std::string str(line);
    std::size_t found = str.find("loaded has 0 vn");
    if (found!=std::string::npos)
    {
      ROS_ERROR_STREAM("The following system call failed. Check URDF data.");
      ROS_ERROR_STREAM(stream.str());
      exit (EXIT_FAILURE);
    }
  }
  if (line)
    free(line);

  pclose(fp);

  return simox_filename;
}

//-------------------------------------------------------------------------------

void UrdfToSimoxXml::scale_wrl_scene_(const std::string & filename,
                                      const double & scale_x,
                                      const double & scale_y,
                                      const double & scale_z)
{
  SoSeparator *root = new SoSeparator;
  SoInput scene;

  // Try to open the file.
  if (!scene.openFile(filename.c_str()))
  {
    ROS_ERROR("Could not open %s for reading\n", filename.c_str());
    exit (EXIT_FAILURE);
  }

  // Check if the file is valid.
  if (!scene.isValidFile())
  {
    ROS_ERROR("File %s is not a valid Inventor file\n", filename.c_str());
    exit (EXIT_FAILURE);
  }

  // Try to read the file.
  root = SoDB::readAll(&scene);
  if (root == NULL)
  {
    ROS_ERROR("Problem reading file %s\n", filename.c_str());
    scene.closeFile();
    exit (EXIT_FAILURE);
  }

  // Close the file.
  scene.closeFile();

  // Reference root node so that the graph will not be deleted.
  root->ref();

  // Scale and then save to file.
  for (int i = 0; i < root->getNumChildren(); i++)
  {
    SoNode *child = root->getChild(i);
    if (child->isOfType(SoVRMLTransform::getClassTypeId()))
    {
      SoVRMLTransform *tran = (SoVRMLTransform*)child;
      SoSFVec3f scale_factor;
      scale_factor.setValue(scale_x, scale_y, scale_z);
      tran->scale = scale_factor;

      SoToVRML2Action tovrml2;
      tovrml2.apply(root);
      SoVRMLGroup *newroot = tovrml2.getVRML2SceneGraph();
      newroot->ref();
      root->unref();

      SoOutput out;
      out.openFile(filename.c_str());
      out.setHeaderString("#VRML V2.0 utf8");
      SoWriteAction wra(&out);
      wra.apply(newroot);
      out.closeFile();

      newroot->unref();
      return;
    }
  }

  ROS_ERROR_STREAM("UrdfToSimoxXml::scale_wrl_scene_ failed. Should not reach here.");
  exit (EXIT_FAILURE);
}

//-------------------------------------------------------------------------------

// Set the base link.
void UrdfToSimoxXml::set_base_link_(void)
{
  unsigned short n = 0;
  BOOST_FOREACH(boost::shared_ptr<urdf::Link> link, links_)
  {
    boost::shared_ptr<urdf::Link> parent_link = link->getParent();
    if (!parent_link) {
      base_link_ = link;
      ROS_INFO_STREAM("The base link is set to " << base_link_->name);
      n++;
    }
  }
  if (n == 0)
  {
    ROS_ERROR_STREAM("Failed to set the base link.");
    exit (EXIT_FAILURE);
  }
  if (n > 1)
  {
    ROS_ERROR_STREAM("There are multiple base links.");
    exit (EXIT_FAILURE);
  }
}

//-------------------------------------------------------------------------------

void UrdfToSimoxXml::read_dae_file_(const std::string & dae_filename)
{
  // Create an empty property tree object.
  ptree pt;

  // Load the XML file into the property tree. If reading fails
  // (cannot open file, parse error), an exception is thrown.
  read_xml(dae_filename, pt);

  // Get the unit. There should be one and only one "COLLADA.asset.unit".
  BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("COLLADA.asset.unit"))
  {
    std::string unit_name = v.second.get<std::string>("name");
    double to_meter = boost::lexical_cast<double>(v.second.get<std::string>("meter"));
    scale_ = to_meter;
    return;
  }
}

//-------------------------------------------------------------------------------

std::string UrdfToSimoxXml::to_string_(double x)
{
  // http://stackoverflow.com/questions/5016464/boostlexical-cast-conversion-double-to-string-c
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3);
  ss << x;
  std::string s = ss.str();
  return s;
}

//-------------------------------------------------------------------------------

bool UrdfToSimoxXml::compareUrdfJoint(boost::shared_ptr<urdf::Joint> j1,
                                      boost::shared_ptr<urdf::Joint> j2)
{
  return (j1->name < j2->name);
}

//-------------------------------------------------------------------------------
