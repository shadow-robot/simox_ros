#!/usr/bin/env python

#
# Quick script to fix up the old fuerte hand urdf for use here.
# Fixes changed tags, re-writes filepaths, removes pr2/shadow specific gazebo
# stuff, etc.
#
# Input is the full hand urdf. Generated from the shadow_robot sr_description
# package under fuerte as follows:
#
# $ rosrun xacro xacro.py robots/shadowhand_motor_biotac.urdf.xacro > urdf/shadowhand.orig.urdf
#
# Then run over with this:
#
# $ ./contrib/fix_urdf.py urdf/shadowhand.orig.urdf urdf/shadowhand.urdf
#

import sys, os
import xml.etree.ElementTree as ET

def usage():
    print "Usage: " + sys.argv[0] + " URDF_FILE OUT_FILE\n"
    sys.exit(3)

if len(sys.argv) != 3:
    usage()

in_file = sys.argv[1]
out_file = sys.argv[2]

namespaces = {
    'controller': 'http://playerstage.sourceforge.net/gazebo/xmlschema/#controller',
    'sensor':     "http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor",
    'interface':  "http://playerstage.sourceforge.net/gazebo/xmlschema/#interface",
}
for prefix, uri in namespaces.iteritems():
    ET.register_namespace(prefix, uri)
tree = ET.parse(in_file)
root = tree.getroot()

# Dump tree
#for elem in tree.getiterator():
#    print elem.tag, elem.attrib

# Tweak robot name
root.find('.').attrib['name'] = "shadowhand"

# Re-write the filenames for the meshes
for mesh in root.iter('mesh'):
    fn = mesh.attrib['filename']
    (path, name) = os.path.split(fn)
    new_file = "package://sr_grasp_description/meshes/" + name
    #print fn + " -> " + new_file
    mesh.attrib['filename'] = new_file

# Re-move sr_gazebo_ros_controller_manager
for tag in root.findall('.//controller:sr_gazebo_ros_controller_manager/..', namespaces=namespaces):
    root.remove(tag)

# For now remove the sensor:contact as Gazebo doesn't like them
for tag in root.findall('.//sensor:contact/..', namespaces=namespaces):
    root.remove(tag)

# For now remove all gazebo tags!
for tag in root.findall('.//gazebo', namespaces=namespaces):
    root.remove(tag)

# remove all transmissions
for tag in root.findall('.//transmission', namespaces=namespaces):
    root.remove(tag)

# Write it back out
tree.write(out_file)

