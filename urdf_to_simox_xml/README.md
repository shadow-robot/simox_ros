How to run the URDF to Simox XML converter.

$ cd ~/catkin_ws
$ ls
build  devel src
$ ls src/
CMakeLists.txt  dms_description  README.md  urdf_to_simox_xml
$ catkin_make
$ source devel/setup.bash
$ rosrun urdf_to_simox_xml urdf_to_simox_xml src/dms_description/robots/urdf/dms.urdf dms.xml

The converter generates dms.xml and places it in ~/catkin_ws.

$ ls
build  devel  dms.xml  src
$ RobotViewer

Click on the "Select Robot File" button on the panel, and select dms.xml.

Note that RobotViewer is part of http://simox.sourceforge.net. If you cannot find it, make sure that you have installed SoQt library before installing Simox.

To grasp an object with the DMS hand, run GraspPlanner (also part of Simox).

$ GraspPlanner --robot dms.xml --endeffector "DMS Hand" --preshape "Grasp Preshape"

Note that all preshape values are set to zero by the converter. Update them before you run the grasp planner (especially the values for the thumb). Compare with the handcrafted file dms.xml inside package dms_description (dms_description/robots/simox_xml/dms.xml). Inside the Preshape node, update also the values of the Actor nodes.
