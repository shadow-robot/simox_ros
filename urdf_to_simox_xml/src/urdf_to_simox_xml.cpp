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
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <ros/console.h>

//-------------------------------------------------------------------------------

using namespace gsc;
using boost::property_tree::ptree;

//-------------------------------------------------------------------------------

UrdfToSimoxXml::UrdfToSimoxXml(const bool urdf_init_param,
                               const std::string& urdf_file)
  : urdf_model_(new urdf::Model())
{
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

  // Get all links in the model.
  urdf_model_->getLinks(links_);
  if (links_.empty())
  {
    ROS_ERROR_STREAM("There are no links in " << urdf_file << ".");
    exit (EXIT_FAILURE);
  }

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

void UrdfToSimoxXml::write_xml(const std::string& simox_xml_file)
{
  // Create empty property tree object
  ptree pt;

  ///////////////////////////////////////////////////////////////////////////////

  std::string dms_hand_base("dms_hand_base");
  std::string dms_hand_tcp("dms_hand_tcp");
  std::string dms_hand_gcp("dms_hand_gcp");
  std::string base_link(links_[0]->name);

  ///////////////////////////////////////////////////////////////////////////////

  // Create the DMSHand node.
  boost::property_tree::ptree DMSHand_node;
  DMSHand_node.put("<xmlattr>.Type", "DMSHand");
  DMSHand_node.put("<xmlattr>.RootNode", dms_hand_base);

  // Add RobotNode name="dms_hand_base".
  this->add_dms_hand_base_node_(DMSHand_node, dms_hand_base, dms_hand_tcp,
                                dms_hand_gcp, base_link);

  // Add RobotNode name="dms_hand_tcp".
  this->add_dms_hand_tcp_node_(DMSHand_node, dms_hand_tcp);

  // Add RobotNode name="dms_hand_gcp".
  this->add_dms_hand_gcp_node_(DMSHand_node, dms_hand_gcp);

  // Add RobotNode for the base/first link.
  this->add_link_node_(DMSHand_node, links_[0]);

  // Add Endeffector name="DMS Hand" base="dms_hand_base" tcp="dms_hand_tcp" gcp="dms_hand_gcp".
  this->add_endeffector_node_(DMSHand_node, dms_hand_base, dms_hand_tcp,
                              dms_hand_gcp, base_link);

  // Add RobotNodeSet name="DMS Hand Joints".
  this->add_dms_hand_joints_node_(DMSHand_node);

  // Add the DMSHand node to the tree.
  pt.add_child("Robot", DMSHand_node);

  ///////////////////////////////////////////////////////////////////////////////

  // Write property tree to XML file
  // http://stackoverflow.com/questions/6572550/boostproperty-tree-xml-pretty-printing
  boost::property_tree::xml_writer_settings<char> settings('\t', 1);
  boost::property_tree::write_xml(simox_xml_file, pt, std::locale(), settings);
}

//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

void UrdfToSimoxXml::add_dms_hand_base_node_(boost::property_tree::ptree & DMSHand_node,
                                             const std::string & dms_hand_base,
                                             const std::string & dms_hand_tcp,
                                             const std::string & dms_hand_gcp,
                                             const std::string & base_link)
{
  boost::property_tree::ptree dms_hand_base_node;
  dms_hand_base_node.put("<xmlattr>.name", dms_hand_base);

  boost::property_tree::ptree Child_node_1;
  boost::property_tree::ptree Child_node_2;
  boost::property_tree::ptree Child_node_3;
  Child_node_1.put("<xmlattr>.name", dms_hand_tcp);
  Child_node_2.put("<xmlattr>.name", dms_hand_gcp);
  Child_node_3.put("<xmlattr>.name", base_link);

  dms_hand_base_node.add_child("Child", Child_node_1);
  dms_hand_base_node.add_child("Child", Child_node_2);
  dms_hand_base_node.add_child("Child", Child_node_3);

  DMSHand_node.add_child("RobotNode", dms_hand_base_node);
}

//-------------------------------------------------------------------------------

void UrdfToSimoxXml::add_dms_hand_tcp_node_(boost::property_tree::ptree & DMSHand_node,
                                            const std::string & dms_hand_tcp)
{
  boost::property_tree::ptree dms_hand_tcp_node;
  dms_hand_tcp_node.put("<xmlattr>.name", dms_hand_tcp);
  dms_hand_tcp_node.put("<xmlcomment>", "Translation values were set manually!");

  boost::property_tree::ptree Translation_node;
  this->set_translation_node_(Translation_node, -0.01, -0.035, 0.07);

  boost::property_tree::ptree Transform_node;
  Transform_node.add_child("Translation", Translation_node);
  dms_hand_tcp_node.add_child("Transform", Transform_node);
  DMSHand_node.add_child("RobotNode", dms_hand_tcp_node);
}

//-------------------------------------------------------------------------------

void UrdfToSimoxXml::add_dms_hand_gcp_node_(boost::property_tree::ptree & DMSHand_node,
                                            const std::string & dms_hand_gcp)
{
  boost::property_tree::ptree dms_hand_gcp_node;
  dms_hand_gcp_node.put("<xmlattr>.name", dms_hand_gcp);
  dms_hand_gcp_node.put("<xmlcomment>", "Translation and rollpitchyaw values were set manually!");

  boost::property_tree::ptree Translation_node;
  this->set_translation_node_(Translation_node, -0.01, -0.035, 0.07);

  boost::property_tree::ptree rollpitchyaw_node;
  this->set_rollpitchyaw_node_(rollpitchyaw_node, 1.0, 0.0, 0.0);

  boost::property_tree::ptree Transform_node;
  Transform_node.add_child("Translation", Translation_node);
  Transform_node.add_child("rollpitchyaw", rollpitchyaw_node);
  dms_hand_gcp_node.add_child("Transform", Transform_node);
  DMSHand_node.add_child("RobotNode", dms_hand_gcp_node);
}

//-------------------------------------------------------------------------------

void UrdfToSimoxXml::add_link_node_(boost::property_tree::ptree & DMSHand_node,
                                    boost::shared_ptr<const urdf::Link> link)
{
  boost::property_tree::ptree link_node;
  link_node.put("<xmlattr>.name", link->name);

  boost::shared_ptr<urdf::Visual> visual = link->visual;
  urdf::Pose pose = visual->origin;

  boost::property_tree::ptree Translation_node;
  this->set_translation_node_(Translation_node, pose.position);

  boost::property_tree::ptree rollpitchyaw_node;
  this->set_rollpitchyaw_node_(rollpitchyaw_node, pose.rotation);

  boost::property_tree::ptree Transform_node;
  Transform_node.add_child("Translation", Translation_node);
  Transform_node.add_child("rollpitchyaw", rollpitchyaw_node);
  link_node.add_child("Transform", Transform_node);

  boost::shared_ptr<urdf::Geometry> geometry = visual->geometry;
  if (geometry->type != urdf::Geometry::MESH)
  {
    ROS_ERROR_STREAM("MESH is the only supported urdf::Geometry type.");
    exit (EXIT_FAILURE);
  }
  boost::shared_ptr<urdf::Mesh> mesh = boost::dynamic_pointer_cast<urdf::Mesh>(geometry);

  boost::property_tree::ptree Visualization_File_node;
  Visualization_File_node.put("<xmlattr>.type", "Inventor");
  Visualization_File_node.put("<xmltext>", this->convert_filename_(mesh->filename));

  boost::property_tree::ptree Visualization_node;
  Visualization_node.put("<xmlattr>.enable", "true");
  Visualization_node.add_child("File", Visualization_File_node);
  link_node.add_child("Visualization", Visualization_node);

  boost::property_tree::ptree CollisionModel_File_node;
  CollisionModel_File_node.put("<xmlattr>.type", "Inventor");
  CollisionModel_File_node.put("<xmltext>", this->convert_filename_(mesh->filename));

  boost::property_tree::ptree CollisionModel_node;
  CollisionModel_node.add_child("File", CollisionModel_File_node);
  link_node.add_child("CollisionModel", CollisionModel_node);

  std::vector< boost::shared_ptr<urdf::Joint> > child_joints;
  child_joints = link->child_joints;
  BOOST_FOREACH(boost::shared_ptr<const urdf::Joint> child_joint, child_joints)
  {
    boost::property_tree::ptree Child_node;
    Child_node.put("<xmlattr>.name", child_joint->name);
    link_node.add_child("Child", Child_node);
  }

  DMSHand_node.add_child("RobotNode", link_node);

  // Add the child joints.
  BOOST_FOREACH(boost::shared_ptr<const urdf::Joint> child_joint, child_joints)
  {
    this->add_joint_node_(DMSHand_node, child_joint);
  }
}

//-------------------------------------------------------------------------------

void UrdfToSimoxXml::add_joint_node_(boost::property_tree::ptree & DMSHand_node,
                                     boost::shared_ptr<const urdf::Joint> child_joint)
{
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

  boost::property_tree::ptree Axis_node;
  this->set_axis_node_(Axis_node, child_joint->axis);

  boost::property_tree::ptree Limits_node;
  this->set_joint_limits_node_(Limits_node, child_joint->limits);

  boost::property_tree::ptree Joint_node;
  if (child_joint->type != urdf::Joint::REVOLUTE)
  {
    ROS_ERROR_STREAM("Only revolute joints are support at moment.");
    exit (EXIT_FAILURE);
  }
  Joint_node.put("<xmlattr>.type", "revolute");
  Joint_node.add_child("Axis", Axis_node);
  Joint_node.add_child("Limits", Limits_node);
  child_joint_node.add_child("Joint", Joint_node);

  boost::property_tree::ptree Child_node;
  boost::shared_ptr<const urdf::Link> child_link = urdf_model_->getLink(child_joint->child_link_name);
  Child_node.put("<xmlattr>.name", child_link->name);
  child_joint_node.add_child("Child", Child_node);

  DMSHand_node.add_child("RobotNode", child_joint_node);

  // Add the child link.
  this->add_link_node_(DMSHand_node, child_link);
}

//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

void UrdfToSimoxXml::add_endeffector_node_(boost::property_tree::ptree & DMSHand_node,
                                           const std::string & dms_hand_base,
                                           const std::string & dms_hand_tcp,
                                           const std::string & dms_hand_gcp,
                                           const std::string & base_link)
{
  boost::property_tree::ptree Endeffector_node;
  Endeffector_node.put("<xmlcomment>", "This node is for Simox (e.g., GraspPlanner in Simox)!");
  Endeffector_node.put("<xmlattr>.name", "DMS Hand");
  Endeffector_node.put("<xmlattr>.base", dms_hand_base);
  Endeffector_node.put("<xmlattr>.tcp", dms_hand_tcp);
  Endeffector_node.put("<xmlattr>.gcp", dms_hand_gcp);

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
    char actor_name = static_cast<char>(iter->first);

    boost::property_tree::ptree Actor_node;
    Actor_node.put("<xmlattr>.name", actor_name);
    Actor_node.put("<xmlcomment>", "This is just a template. Please set values manually!");
    Actor_node.put("<xmlcomment>", "Note that considerCollisions = None, Actors, or All!");

    BOOST_FOREACH(boost::shared_ptr<const urdf::Link> link, links_)
    {
      const char& first_char = link->name.at(0);
      if (actor_name == first_char)
      {
        boost::property_tree::ptree Node_node;
        Node_node.put("<xmlattr>.name", link->name);
        Node_node.put("<xmlattr>.considerCollisions", "None");
        Actor_node.add_child("Node", Node_node);
      }
    }

    BOOST_FOREACH(boost::shared_ptr<const urdf::Joint> joint, joints_)
    {
      const char& first_char = joint->name.at(0);
      if (actor_name ==  first_char)
      {
        boost::property_tree::ptree Node_node;
        Node_node.put("<xmlattr>.name", joint->name);
        Node_node.put("<xmlattr>.considerCollisions", "None");
        Actor_node.add_child("Node", Node_node);
      }
    }

    Endeffector_node.add_child("Actor", Actor_node);
  }

  DMSHand_node.add_child("Endeffector", Endeffector_node);
}

void UrdfToSimoxXml::add_dms_hand_joints_node_(boost::property_tree::ptree & DMSHand_node)
{
  boost::property_tree::ptree dms_hand_joints_node;
  dms_hand_joints_node.put("<xmlcomment>", "This node is for Simox (e.g., GraspPlanner in Simox)!");
  dms_hand_joints_node.put("<xmlattr>.name", "DMS Hand Joints");

  BOOST_FOREACH(boost::shared_ptr<const urdf::Joint> joint, joints_)
  {
    boost::property_tree::ptree Node_node;
    Node_node.put("<xmlattr>.name", joint->name);
    dms_hand_joints_node.add_child("Node", Node_node);
  }

  DMSHand_node.add_child("RobotNodeSet", dms_hand_joints_node);
}

//-------------------------------------------------------------------------------

// Once the keys are converted back to char, actors contain 'f', 'l', 'm', 't'.
void UrdfToSimoxXml::get_actors(std::map<int, bool>& actors)
{
  actors.clear();
  BOOST_FOREACH(boost::shared_ptr<const urdf::Joint> joint, joints_)
  {
    const char& first_char = joint->name.at(0);
    const int first_int = static_cast<int>(first_char);
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

// urdf_filename  = "package://dms_description/meshes/base_link.STL"
// simox_filename = "base_link.wrl"
std::string UrdfToSimoxXml::convert_filename_(const std::string & urdf_filename)
{
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

  // For example "src/dms_description/meshes/wrl/base_link.wrl"
  simox_filename = "src/dms_description/meshes/wrl/" + simox_filename;

  return simox_filename;
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
//-------------------------------------------------------------------------------

bool UrdfToSimoxXml::compareUrdfJoint(boost::shared_ptr<urdf::Joint> j1, boost::shared_ptr<urdf::Joint> j2)
{
  return (j1->name < j2->name);
}

//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

