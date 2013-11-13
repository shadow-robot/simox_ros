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
 * @brief  Calll class UrdfToSimoxXml to convert from a hand description in URDF format to Simox XML.
 **/

//-------------------------------------------------------------------------------

#include "urdf_to_simox_xml/urdf_to_simox_xml.hpp"

#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <ros/ros.h>

//-------------------------------------------------------------------------------

namespace po = boost::program_options;

//-------------------------------------------------------------------------------

int main(int argc, char** argv)
{
  bool urdf_init_param;
  std::string urdf_filename;
  std::string simox_xml_filename;

  try {
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help",
       "produce help message")
      ("robot_description", po::value<bool>(&urdf_init_param)->default_value(false),
       "disables the urdf file as the input when true (use the robot_description parameter instead)")
      ("urdf", po::value<std::string>(&urdf_filename)->default_value("src/dms_description/robots/urdf/dms.urdf"),
       "set the path to the urdf file (input)")
      ("xml", po::value<std::string>(&simox_xml_filename)->default_value("dms.xml"),
       "set the filename of the Simox XML file (output)")
      ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
      std::cout << "Usage: rosrun urdf_to_simox_xml urdf_to_simox_xml [options]" << std::endl;
      std::cout << desc << std::cout;
      return 0;
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
    return 1;
  }

  ros::init(argc, argv, "urdf_to_simox_xml");

  gsc::UrdfToSimoxXml urdf2xml(urdf_init_param, urdf_filename);

  std::string simox_xml_file(simox_xml_filename);
  urdf2xml.write_xml(simox_xml_file);

  return 0;
}
