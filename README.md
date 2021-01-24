## UR3e_RG2_ER5 Arm Manipulation
This collection of ROS packages (Noetic) is intended for the manipulation and control of the universal_robotics UR3e tabletop robot arm. Currently these packages are configured to run in simulation and eventually will be modified to allow for real hardware manipulation.

The goal of these packages are to allow for sphereical objects to be spawned in the Gazebo simulator and the UR3e robot arm carry through a pick and place sequence on the sphereical object, sorting it left or right of the arm.

### Getting Started
Open a new terminal and launch the UR3e robot in Gazebo:
```
roslaunch UR3e_RG2_ER5_description gazebo.launch
```

In a new terminal tab, start Moveit planning execution for the UR3e arm:
```
roslaunch UR3e_RG2_ER5_moveit_config UR3e_RG2_ER5_moveit_planning_execution.launch
```


In a new tab, run the object spawning in Gazebo and pick and place function:

```
rosrun UR3e_RG2_ER5_pick_place pick_place
```

Now, in a new tab spawn a sphere into gazebo and the pick and place function should carry through:
```
rostopic pub /pick_place UR3e_RG2_ER5_pick_place/object_sphere '{name: "TEST1",pose: {position: {x: 0.4,y: 0,z: 0.75}, orientation:{x: 0.0, y: 0.0,z: 0.0,w: 0.0}},radius: 0.025}'
```

### Known Issues
Pick function within full_robot_pick_place.cpp located in UR3e_RG2_ER5_pick_place/src does not fully complete (appears to complete but does not return back to continue within main function). Meaning it never gets to the place function.

Robot arm does not go down far enough to pick the sphere. Will not find a trajectory (fails) if it tries to go any lower to pick the sphere. Maybe this feeds into the first problem?

### To Do
Allow for the caipability of placing the sphere left or right depending on what it is told to do.

Fix the stated issues
