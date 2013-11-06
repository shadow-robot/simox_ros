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

#include "ros/ros.h"

#include <iostream>
#include <string>

//-------------------------------------------------------------------------------

int main(int argc, char** argv)
{
  ros::init(argc, argv, "urdf_to_simox_xml");
  if (argc != 3)
  {
    ROS_ERROR_STREAM("Need a urdf file as input and an xml filename as output.");
    exit (EXIT_FAILURE);
  }

  std::string urdf_filename = argv[1];
  std::string simox_xml_filename = argv[2];

  gsc::UrdfToSimoxXml urdf2xml(urdf_filename);

  std::string simox_xml_file(simox_xml_filename);
  urdf2xml.write_xml(simox_xml_file);

  return 0;
}
