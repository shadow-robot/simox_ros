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
 * @brief  Calll class UrdfToSimoxXml to onvert from a hand description in URDF format to Simox XML.
 **/

//-------------------------------------------------------------------------------

#include "urdf_to_simox_xml/urdf_to_simox_xml.hpp"

#include "ros/ros.h"

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <iostream>
#include <string>
#include <set>
#include <exception>

//-------------------------------------------------------------------------------

struct debug_settings
{
  std::string m_file;               // log filename
  int m_level;                      // debug level
  std::set<std::string> m_modules;  // modules where logging is enabled
  void load(const std::string &filename);
  void save(const std::string &filename);
};

void debug_settings::load(const std::string &filename)
{
  // Create empty property tree object
  using boost::property_tree::ptree;
  ptree pt;

  // Load XML file and put its contents in property tree.
  // No namespace qualification is needed, because of Koenig
  // lookup on the second argument. If reading fails, exception
  // is thrown.
  read_xml(filename, pt, boost::property_tree::xml_parser::trim_whitespace);

  // Get filename and store it in m_file variable. Note that
  // we specify a path to the value using notation where keys
  // are separated with dots (different separator may be used
  // if keys themselves contain dots). If debug.filename key is
  // not found, exception is thrown.
  m_file = pt.get<std::string>("debug.filename");

  // Get debug level and store it in m_level variable. This is
  // another version of get method: if debug.level key is not
  // found, it will return default value (specified by second
  // parameter) instead of throwing. Type of the value extracted
  // is determined by type of second parameter, so we can simply
  // write get(...) instead of get<int>(...).
  m_level = pt.get("debug.level", 0);

  // Iterate over debug.modules section and store all found
  // modules in m_modules set. get_child() function returns a
  // reference to child at specified path; if there is no such
  // child, it throws. Property tree iterator can be used in
  // the same way as standard container iterator. Category
  // is bidirectional_iterator.
  BOOST_FOREACH(ptree::value_type &v, pt.get_child("debug.modules"))
    m_modules.insert(v.second.data());
}

void debug_settings::save(const std::string &filename)
{
  // Create empty property tree object
  using boost::property_tree::ptree;
  ptree pt;

  // Put log filename in property tree
  pt.put("debug.filename", m_file);

  // Put debug level in property tree
  pt.put("debug.level", m_level);

  // Iterate over modules in set and put them in property
  // tree. Note that the add function places new key at the
  // end of list of keys. This is fine in most of the
  // situations. If you want to place item at some other
  // place (i.e. at front or somewhere in the middle),
  // this can be achieved using a combination of the insert
  // and put_value functions
  BOOST_FOREACH(const std::string &name, m_modules)
    pt.add("debug.modules.module", name);

  // Write property tree to XML file
  // http://stackoverflow.com/questions/6572550/boostproperty-tree-xml-pretty-printing
  boost::property_tree::xml_writer_settings<char> settings('\t', 1);
  write_xml(filename, pt, std::locale(), settings);
}

//-------------------------------------------------------------------------------

int main(int argc, char** argv)
{
  ros::init(argc, argv, "urdf_to_simox_xml");
  if (argc != 2)
  {
    ROS_ERROR_STREAM("Need a urdf file as argument");
    exit (EXIT_FAILURE);
  }

  std::string urdf_file = argv[1];
  gsc::UrdfToSimoxXml urdf2xml(urdf_file);

  std::string simox_xml_file("simox_model.xml");
  urdf2xml.write_xml(simox_xml_file);

  //-------------------------------------------------------------------------------

  /*
  std::string xml_file = argv[1];

  try
  {
    debug_settings ds;
    ds.load(xml_file);
    ds.save("debug_settings_out.xml");
    std::cout << "Success\n";
  }
  catch (std::exception &e)
  {
    std::cout << "Error: " << e.what() << "\n";
  }
  return 0;
  */

  //-------------------------------------------------------------------------------

  return 0;
}
