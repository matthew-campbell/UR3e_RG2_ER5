Contains a deprecated controller for the RG2 gripper attached to the physical robot. 

The gripper can still be operated, however, by running:
	rosrun UR3e_RG2_ER5_gripper_control RG2Grip

This will launch a node that receives gripper width values (0-110), which can be passed over the `/gripper_controller/command topic`, and uses these as input to an RG2 hardware script, which it publishes to `/ur_hardware_interface/script_command`.
