# Grasp mesh planner

The grasp mesh planner is part of the [GraspStudio](http://sourceforge.net/p/simox/wiki/GraspStudio/) library and mainly has two functionalities: a generator for building grasping hypothesis and a grasp evaluation component.


## Launching the grasp planner interface
```bash
roslaunch sr_grasp_mesh_planner sr_grasp_planner.launch 
```

## Testing the grasp planner
To run a test:
```bash
catkin_make run_tests_sr_grasp_mesh_planner_rostest_test_test_grasp_mesh_planner.test 
```
