File dms.xml was converted from dms.urdf, but some modifications have been done by hand.
For example, the pregrasp values were set by hand.

To test dms.xml with GraspPlanner (part of Simox):

$ GraspPlanner --robot dms.xml --endeffector DMS --preshape "Grasp Preshape"
