How to run the URDF to Simox XML converter.

$ cd ~/catkin_ws
$ ls
build  devel src
$ ls src/
CMakeLists.txt  dms_description  README.md  urdf_to_simox_xml
$ catkin_make
$ source devel/setup.bash
$ rosrun urdf_to_simox_xml urdf_to_simox_xml src/dms_description/robots/urdf/dms.urdf

The converter generates simox_model.xml inside ~/catkin_ws

$ ls
build  devel  simox_model.xml  src
$ RobotViewer

Click on the "Select Robot File" button on the panel, and select simox_model.xml.

Note that RobotViewer is part of http://simox.sourceforge.net. If you cannot find it, make sure that you have installed SoQt library before installing Simox.
