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
#include <ros/package.h>

//-------------------------------------------------------------------------------

namespace po = boost::program_options;

//-------------------------------------------------------------------------------

int main(int argc, char** argv)
{
  ros::init(argc, argv, "urdf_to_simox_xml");

  std::string dms_description_path  = ros::package::getPath("dms_description");
  std::string urdf_filename_default = dms_description_path + "/robots/urdf/dms.urdf";

  std::string urdf_filename;
  std::string simox_xml_filename;

  try {
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help",
       "produce help message")
      ("urdf", po::value<std::string>(&urdf_filename)->default_value(urdf_filename_default),
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

    std::cout << "Path to the URDF file:               " << urdf_filename << std::endl;
    std::cout << "Name of the output file (Simox XML): " << simox_xml_filename << std::endl;
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
    return 1;
  }

  gsc::UrdfToSimoxXml urdf2xml(urdf_filename, dms_description_path);

  std::string simox_xml_file(simox_xml_filename);
  urdf2xml.write_xml(simox_xml_file);

  return 0;
}
