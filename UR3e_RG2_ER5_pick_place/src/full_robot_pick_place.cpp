// Subs to topic _____ to recieve information about a percieved object from the camera.
// Recieves a msg of type object_sphere with a name, pose, and radius. The object
// is always assumed to be a sphere for now. The program will generate the sphere
// in Gazebo and check to see whether the sphere is grabable by means of a bounding
// box. If it is, then the robot will do a pick and place function using commands sent
// to Moveit!.

#include <string>
#include <iostream>
#include <fstream>
using namespace std;
#include <cstdio>
// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SpawnModelRequest.h>
#include <gazebo_msgs/SpawnModelResponse.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Message
#include <UR3e_RG2_ER5_pick_place/object_sphere.h>
#include <geometry_msgs/Pose.h>

// TinyXML2
//#include <tinyxml2.h>

// Global Variables for storing Callback information
geometry_msgs::Pose sphere_pose;
int once = 1;
double sphere_xyz[3];
double sphere_radius;
string sphere_name;
////////////////////////////////////////////////////////////////////////////////////


void objectCallback (const UR3e_RG2_ER5_pick_place::object_sphere::ConstPtr& msg)
{

  //Is the object within the bounding box?

  sphere_xyz[0] = msg->pose.position.x;
  sphere_xyz[1] = msg->pose.position.y;
  sphere_xyz[2] = msg->pose.position.z;
  sphere_pose = msg->pose;
  sphere_radius = msg->radius;
  sphere_name = msg->name;
  ROS_INFO ("looping");


  // ros::AsyncSpinner spinner(1);
  // spinner.start();
  //
  // ros::WallDuration(1.0).sleep();
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // moveit::planning_interface::MoveGroupInterface group("manipulator");
  // group.setPlanningTime(45.0);
  //
  // addCollisionObjects(planning_scene_interface);


    // Wait a bit for ROS things to initialize
  // ros::WallDuration(1.0).sleep();
  //
  // pick(group);
  //
  // ros::WallDuration(1.0).sleep();
  //
  // place(group);
  //
  // ros::waitForShutdown();

}

int BndBox_Check (double msg[3])
{
    //Check to see whether an onject is within the defined bounding box
    //Either return the object or nothing at all

  //Define a bounding box
  double bndBoxOrigin[3] = {0, -0.1, 0.65};   //{x,y,z} origin
  double bndBoxDim[3] = {0.5, 0.2, 1};   //{x,y,z} lengths; must be a positive number
  double bndBox[3] = {bndBoxOrigin[0]+bndBoxDim[0],bndBoxOrigin[1]+bndBoxDim[1],bndBoxOrigin[2]+bndBoxDim[2]};

  //Chacking the pose of the object wrt the bounding box
  if ((msg[0] >= bndBoxOrigin[0]) && (msg[0] <= bndBox[0])){
    ROS_INFO("Sphere fits in x");
    if ((msg[1] >= bndBoxOrigin[1]) && (msg[1] <= bndBox[1])){
      ROS_INFO("Sphere fits in y");
      if ((msg[2] >= bndBoxOrigin[2]) && (msg[2] <= bndBox[2])){
        ROS_INFO("Sphere fits in z");
      }
      else{
        ROS_WARN("Sphere does not fit in z");
        return 0;
      }
    }
    else{
      ROS_WARN("Sphere does not fit in y");
      return 0;
    }
  }
  else{
    ROS_WARN("Sphere does not fit in x");
    return 0;
  }
  return 1;
}

//////////////////////////////////////////////////////////////////////////////////
void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(1);
  posture.joint_names[0] = "gripper_joint";
  //posture.joint_names[0] = "wrist_3_joint";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = 2.0;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(1);
  posture.joint_names[0] = "gripper_joint";
  //posture.joint_names[0] = "wrist_3_joint";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = 0.1;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

////////////////////////////////////////////////////////////////////////////////////

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{

  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  grasps[0].grasp_pose.header.frame_id = "world";
  tf2::Quaternion orientation;
  orientation.setRPY(M_PI / 2, 0, 0);  //commented -M_PI / 2
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);  //commented
  grasps[0].grasp_pose.pose.position.x = sphere_xyz[0]; //0.4
  grasps[0].grasp_pose.pose.position.y = sphere_xyz[1];  //0.1
  grasps[0].grasp_pose.pose.position.z = 0.6;  //0.6
//  grasps[0].grasp_pose.pose.orientation.x = 0.5;
//  grasps[0].grasp_pose.pose.orientation.y = 0.5;
//  grasps[0].grasp_pose.pose.orientation.z = -0.5;
//  grasps[0].grasp_pose.pose.orientation.w = 0.5;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "world";
  /* Direction is set as negative z axis */
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = .01; //0.01;
  grasps[0].pre_grasp_approach.desired_distance = 0.0115; //0.0115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "world";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1;
  grasps[0].post_grasp_retreat.min_distance = .001; //0.01;
  grasps[0].post_grasp_retreat.desired_distance = .045; //0.025;


  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);


  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);

  // Set support surface as table1.
  move_group.setSupportSurfaceName("base"); //commented
  // Call pick to pick up the object using the grasps given
  move_group.pick(sphere_name, grasps);

}

////////////////////////////////////////////////////////////////////////////////////

void place(moveit::planning_interface::MoveGroupInterface& group)
{

  // Create a vector of placings to be attempted, currently only creating single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "world";
  tf2::Quaternion orientation;  //commneted
  orientation.setRPY(0, 0, 0);  //commented
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);  //commented

  /* While placing it is the exact location of the center of the object. */
  // And orientation is now the object's, not the joint
  place_location[0].place_pose.pose.position.x = 0.3;
  place_location[0].place_pose.pose.position.y = 0.35;
  place_location[0].place_pose.pose.position.z = 0.6;
//  place_location[0].place_pose.pose.orientation.x = 0.0;
//  place_location[0].place_pose.pose.orientation.y = 0.0;
//  place_location[0].place_pose.pose.orientation.z = 0.0;
//  place_location[0].place_pose.pose.orientation.w = 1.0;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "world";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.005;
  place_location[0].pre_place_approach.desired_distance = 0.0115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "world";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.001;
  place_location[0].post_place_retreat.desired_distance = 0.025;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  group.place(sphere_name, place_location);

}

////////////////////////////////////////////////////////////////////////////////////

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{

  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where we will be placing the cube.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.4;
  collision_objects[0].primitives[0].dimensions[1] = 0.2;
  collision_objects[0].primitives[0].dimensions[2] = 0.5;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.2;
  collision_objects[0].primitive_poses[0].position.y = 0.4;
  collision_objects[0].primitive_poses[0].position.z = 0.25;


  collision_objects[0].operation = collision_objects[0].ADD;

  // Add second table
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.5;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.2;
  collision_objects[1].primitive_poses[0].position.y = -0.4;
  collision_objects[1].primitive_poses[0].position.z = 0.25;

  collision_objects[1].operation = collision_objects[1].ADD;


  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "world";
  collision_objects[2].id = sphere_name;

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].SPHERE;
  collision_objects[2].primitives[0].dimensions.resize(1);
  collision_objects[2].primitives[0].dimensions[0] = sphere_radius;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = sphere_xyz[0];
  collision_objects[2].primitive_poses[0].position.y = sphere_xyz[1];
  collision_objects[2].primitive_poses[0].position.z = 0.6;

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{

  ros::init(argc, argv, "ur3e_arm_pick_place");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_urdf_model");
  gazebo_msgs::SpawnModel srv;
  ros::Subscriber sub = nh.subscribe("/pick_place", 1, objectCallback);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("manipulator");
  group.setPlanningTime(60.0);

  addCollisionObjects(planning_scene_interface);


  while (ros::ok()){
//    ROS_INFO ("%f %f %f %f", sphere[0], sphere[1], sphere[2], sphere[3]);
    if (BndBox_Check(sphere_xyz)){
      if ((sphere_radius < 1.5)&&(sphere_radius != 0)){   //Checks that the radius doesn't exceed a maximum value
        //Need to call a function to put the radius in a generic xml file
        string origin_xyz = "0 0 0";
        string sphere_urdf = "<robot name=\"sphere\"><link name=\"sphere\"><inertial><origin xyz=\"" + origin_xyz + "\" /><mass value=\"1.0\" /><inertia  ixx=\"1.0\" ixy=\"0.0\"  ixz=\"0.0\"  iyy=\"1.0\"  iyz=\"0.0\"  izz=\"1.0\" /></inertial><visual><origin xyz=\"" + origin_xyz + "\"/><geometry><sphere radius=\"" + to_string(sphere_radius) + "\" /></geometry></visual><collision><origin xyz=\"" + origin_xyz + "\"/><geometry><sphere radius=\"" + to_string(sphere_radius) + "\" /></geometry></collision></link><gazebo reference= \"sphere\"><material>Gazebo/Green</material></gazebo></robot>";
//        printf("%s\n",sphere_urdf.c_str());
        ROS_INFO("%s", sphere_urdf.c_str());

        srv.request.model_name = sphere_name;
        srv.request.model_xml = sphere_urdf;
        srv.request.robot_namespace = "sphere_spawner";
        srv.request.initial_pose = sphere_pose;
        srv.request.reference_frame = "world";

        if (once){
          if (client.call(srv)){
          once = 0;
          }
          else{
            ROS_WARN("didn't work!!");
          }
          ROS_INFO("Result: %s, code %u",srv.response.status_message.c_str(), srv.response.success);
//          return 0;
        }
        ros::WallDuration(1.0).sleep();

        pick(group);
        ROS_INFO("Picked");
        ros::WallDuration(1.0).sleep();

        place(group);
        ROS_INFO("Placed");
        ros::WallDuration(1.0).sleep();

      }
      else{   //sphere too big to grasp
        ROS_WARN ("Sphere is too large to grasp :(");
      }
    }
    ros::spinOnce();
  }

  return 0;
}
