/*
 * Copyright (c) 2013 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

/**
 * @file   main.cpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  Calll class UrdfToSimoxXml to convert from a hand description
 *         in URDF format to Simox XML.
 **/

//-------------------------------------------------------------------------------

#include "urdf_to_simox_xml/urdf_to_simox_xml.hpp"

#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <ros/ros.h>
#include <ros/package.h>

//-------------------------------------------------------------------------------

namespace po = boost::program_options;

//-------------------------------------------------------------------------------

int main(int argc, char** argv)
{
  ros::init(argc, argv, "urdf_to_simox_xml");

  std::string package_name("sr_grasp_description");
  std::string package_path = ros::package::getPath(package_name);
  if (package_path.empty())
  {
    ROS_ERROR_STREAM("Failed to obtain the path of package " << package_name << ".");
    return -1;
  }

  std::string urdf_filename_default = package_path + "/urdf/shadowhand.urdf";

  std::string output_dir_default = std::string(getenv("HOME")) + "/urdf_to_simox_xml_output";

  bool urdf_init_param;
  std::string urdf_filename;
  std::string output_dir;
  std::string simox_xml_filename;
  double scale;

  try {
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help",
       "produce help message")
      ("robot_description", po::value<bool>(&urdf_init_param)->default_value(false),
       "disables the urdf file as the input when true (use the robot_description parameter instead)")
      ("urdf", po::value<std::string>(&urdf_filename)->default_value(urdf_filename_default),
       "set the path to the urdf file (input)")
      ("output_dir", po::value<std::string>(&output_dir)->default_value(output_dir_default),
       "set the output directory")
      ("xml", po::value<std::string>(&simox_xml_filename)->default_value("shadowhand.xml"),
       "set the filename of the Simox XML file (output)")
      ("scale", po::value<double>(&scale)->default_value(1.0),
       "set the default scale (used when converting to WRL files)\n"
       "note that the units in VRML (i.e., .WRL files) are assumed to be meters.")
      ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
      std::cout << "Usage: rosrun urdf_to_simox_xml urdf_to_simox_xml [options]" << std::endl;
      std::cout << desc << "\n\n";
      exit (EXIT_SUCCESS);
    }

    if (urdf_init_param)
      std::cout << "Load from the robot_description parameter : " << urdf_init_param << std::endl;
    else
      std::cout << "Path to the URDF file:                      " << urdf_filename << std::endl;
    std::cout << "Name of the output file (Simox XML):        " << simox_xml_filename << std::endl;
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
    exit (EXIT_FAILURE);
  }

  if (!boost::filesystem::exists(output_dir))
    boost::filesystem::create_directories(output_dir);

  gsc::UrdfToSimoxXml urdf2xml(urdf_init_param, urdf_filename, output_dir, scale);

  urdf2xml.write_xml(output_dir, simox_xml_filename);

  return 0;
}
