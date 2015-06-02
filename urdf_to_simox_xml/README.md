How to run the URDF to Simox XML converter.
```
catkin_make
source devel/setup.bash
rospack find urdf_to_simox_xml
rosrun urdf_to_simox_xml urdf_to_simox_xml --help
```
---------
The output_dir should point to the path where the *model* folder with the .wrl files are located. For example, to generate the model of the shadow hand:
rosrun urdf_to_simox_xml urdf_to_simox_xml --output_dir src/simox_ros/sr_grasp_description/simox

Use RobotViewer to verify the output (xml files such as shadowhand.xml and dms.xml):
```
RobotViewer
```
Click on the "Select Robot File" button on the panel, and select the xml file.

Note that RobotViewer is part of http://simox.sourceforge.net. If you cannot find it, make sure that you have installed SoQt library before installing Simox.

To grasp an object with the hand (e.g., the DMS hand), run GraspPlanner (also part of Simox).
```
GraspPlanner --robot dms.xml --endeffector "DMS Hand" --preshape "Grasp Preshape"
```
Note that all preshape values are set to zero by the converter. Update them before you run the grasp planner (especially the values for the thumb). Inside the Preshape node, update also the values of the Actor nodes.

---------
