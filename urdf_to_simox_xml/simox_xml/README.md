The .xml files converted from dms.urdf, but some modifications have been done by hand. For example, the pregrasp values were set by hand.

To test the .xml files with GraspPlanner (part of Simox):

$ GraspPlanner --robot dms.xml --endeffector DMS --preshape "Grasp Preshape"

$ GraspPlanner --robot shadowhand.xml --endeffector SHADOWHAND --preshape "Grasp Preshape"
