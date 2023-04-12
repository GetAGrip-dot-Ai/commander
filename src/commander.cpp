#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


const double tau = 2 * M_PI;
static const std::string PLANNING_GROUP = "arm";
namespace rvt = rviz_visual_tools;


void multiframe(){
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // get current pose
  geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();

  // set orientation
  tf2::Quaternion q;
  q.setRPY(-M_PI/2,-M_PI/2,-M_PI/2); //kinova
  geometry_msgs::Quaternion quat;
  quat = tf2::toMsg(q);
  // geometry_msgs::Pose constrained_pose;
  // constrained_pose.orientation = quat;

  // pose 1
  geometry_msgs::Pose pose1;
  pose1.orientation = quat;
  pose1.position = current_pose.pose.position;
  // pose1.position.x -= (0.192 - 0.061525); // offset between our end-effector gripping point and tool_frame for robotiq ee
  pose1.position.z += 0.02;
  move_group_interface.setPoseTarget(pose1);
  // plan
  move_group_interface.setPlanningTime(15.0);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
  bool success1 = (move_group_interface.plan(my_plan1) == moveit::core::MoveItErrorCode::SUCCESS);
  visual_tools.publishTrajectoryLine(my_plan1.trajectory_, joint_model_group);
  visual_tools.trigger();
  // execute plan
  move_group_interface.move();


  // pose 2
  geometry_msgs::Pose pose2;
  pose2.orientation = quat;
  pose2.position = current_pose.pose.position;
  // pose2.position.x -= (0.192 - 0.061525); // offset between our end-effector gripping point and tool_frame for robotiq ee
  pose2.position.z += 0.02;
  pose2.position.y -= 0.02;
  move_group_interface.setPoseTarget(pose2);
  // plan
  move_group_interface.setPlanningTime(15.0);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  bool success2 = (move_group_interface.plan(my_plan2) == moveit::core::MoveItErrorCode::SUCCESS);
  visual_tools.publishTrajectoryLine(my_plan2.trajectory_, joint_model_group);
  visual_tools.trigger();
  // execute plan
  move_group_interface.move();


  // pose 3
  geometry_msgs::Pose pose3;
  pose3.orientation = quat;
  pose3.position = current_pose.pose.position;
  // pose3.position.x -= (0.192 - 0.061525); // offset between our end-effector gripping point and tool_frame for robotiq ee
  pose3.position.z -= 0.02;
  pose3.position.y -= 0.02;
  move_group_interface.setPoseTarget(pose3);
  // plan
  move_group_interface.setPlanningTime(15.0);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
  bool success3 = (move_group_interface.plan(my_plan3) == moveit::core::MoveItErrorCode::SUCCESS);
  visual_tools.publishTrajectoryLine(my_plan3.trajectory_, joint_model_group);
  visual_tools.trigger();
  // execute plan
  move_group_interface.move();


  // pose 4
  geometry_msgs::Pose pose4;
  pose4.orientation = quat;
  pose4.position = current_pose.pose.position;
  // pose4.position.x -= (0.192 - 0.061525); // offset between our end-effector gripping point and tool_frame for robotiq ee
  pose4.position.z -= 0.02;
  pose4.position.y += 0.02;
  move_group_interface.setPoseTarget(pose4);
  // plan
  move_group_interface.setPlanningTime(15.0);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan4;
  bool success4 = (move_group_interface.plan(my_plan4) == moveit::core::MoveItErrorCode::SUCCESS);
  visual_tools.publishTrajectoryLine(my_plan4.trajectory_, joint_model_group);
  visual_tools.trigger();
  // execute plan
  move_group_interface.move();


  // pose 5
  geometry_msgs::Pose pose5;
  pose5.orientation = quat;
  pose5.position = current_pose.pose.position;
  // pose5.position.x -= (0.192 - 0.061525); // offset between our end-effector gripping point and tool_frame for robotiq ee
  pose5.position.z += 0.02;
  pose5.position.y += 0.02;
  move_group_interface.setPoseTarget(pose5);
  // plan
  move_group_interface.setPlanningTime(15.0);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan5;
  bool success5 = (move_group_interface.plan(my_plan5) == moveit::core::MoveItErrorCode::SUCCESS);
  visual_tools.publishTrajectoryLine(my_plan5.trajectory_, joint_model_group);
  visual_tools.trigger();
  // execute plan
  move_group_interface.move();

}


void multiframe_cart(){

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  move_group_interface.setMaxVelocityScalingFactor(0.5);
  move_group_interface.setMaxAccelerationScalingFactor(0.1);

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
  geometry_msgs::Pose next_pose = current_pose.pose;

  next_pose.position.z += 0.02;
  waypoints.push_back(next_pose);

  next_pose.position.y -= 0.02;
  waypoints.push_back(next_pose);

  next_pose.position.z -= 0.04;
  waypoints.push_back(next_pose);  

  next_pose.position.y += 0.04;
  waypoints.push_back(next_pose);  

  next_pose.position.z += 0.04;
  waypoints.push_back(next_pose); 

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
  // sleep(15.0);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  // visual_tools.prompt("Execute multiframe move?");

  // Cartesian motions should often be slow, e.g. when approaching objects. The speed of cartesian
  // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
  // the trajectory manually, as described `here <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
  // Pull requests are welcome.
  //
  // You can execute a trajectory like this.
  move_group_interface.execute(trajectory);
}


// remove obstacles from scene
void removeObstacles(std::vector<std::string> object_ids){
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.loadRemoteControl();
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
  planning_scene_interface.removeCollisionObjects(object_ids);
  visual_tools.trigger();
}


// add obstacle to the scene
void addObstacle(geometry_msgs::Pose obstacle_pose, shape_msgs::SolidPrimitive primitive, std::string obstacle_id){
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // define a collision object ROS message for the robot to avoid
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();

  // The id of the object is used to identify it
  collision_object.id = obstacle_id;

  // add the collision object into the world
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(obstacle_pose);
  collision_object.operation = collision_object.ADD;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
  // visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'n' in the RvizVisualToolsGui window to once the collision object appears in RViz");
}


// move to target pose
void moveToPose(geometry_msgs::Pose target_pose)
{
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // // Getting Basic Information
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // print the name of the reference frame for this robot
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  //  print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
  // list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // // we call the planner to compute the plan and visualize it.
  // // move_group_interface.setPlanningTime(15.0);
  tf2::Quaternion q;
  q.setRPY(-M_PI/2,-M_PI/2,-M_PI/2); //kinova
  geometry_msgs::Quaternion quat;
  quat = tf2::toMsg(q);
  geometry_msgs::Pose constrained_pose;
  constrained_pose.orientation = quat;
  constrained_pose.position = target_pose.position;
  constrained_pose.position.x -= (0.192 - 0.061525); // offset between our end-effector gripping point and tool_frame for robotiq ee
  move_group_interface.setPoseTarget(constrained_pose);

  // make plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // // visualize plan
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // execute plan
  visual_tools.prompt("execute?");
  move_group_interface.move();
  
  // // print rpy
  // std::vector< double > rpy = move_group_interface.getCurrentRPY();
  // ROS_INFO_STREAM(rpy[0]);
  // ROS_INFO_STREAM(rpy[1]);
  // ROS_INFO_STREAM(rpy[2]);
  // // q_rot.setRPY(r, p, y);
}


int main(int argc, char** argv)
{
  // start commander node
  ros::init(argc, argv, "commander_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // set target pose
  geometry_msgs::Pose target_pose;
  target_pose.position.x = -0.2;
  target_pose.position.y = -0.5;
  target_pose.position.z = 0.4;

  // set obstacle pose
  // geometry_msgs::Pose obstacle_pose;
  // obstacle_pose.position.x = 0;
  // obstacle_pose.position.y = -0.5;
  // obstacle_pose.position.z = 0.4;

  // // set obstacle pose
  // geometry_msgs::Pose obstacle_pose2;
  // obstacle_pose2.position.x = 0.1;
  // obstacle_pose2.position.y = -0.4;
  // obstacle_pose2.position.z = 0.3;

  // // set obstacle pose
  // geometry_msgs::Pose obstacle_pose3;
  // obstacle_pose3.position.x = -0.1;
  // obstacle_pose3.position.y = -0.4;
  // obstacle_pose3.position.z = 0.3;

  // // define obstacle dimensions
  // shape_msgs::SolidPrimitive obstacle_primitive;
  // obstacle_primitive.type = obstacle_primitive.BOX;
  // obstacle_primitive.dimensions.resize(3);
  // obstacle_primitive.dimensions[obstacle_primitive.BOX_X] = 0.05;
  // obstacle_primitive.dimensions[obstacle_primitive.BOX_Y] = 0.05;
  // obstacle_primitive.dimensions[obstacle_primitive.BOX_Z] = 0.07;

  // // add obstacles to scene
  // addObstacle(obstacle_pose, obstacle_primitive, "Pepper1");
  // addObstacle(obstacle_pose2, obstacle_primitive, "Pepper2");
  // addObstacle(obstacle_pose3, obstacle_primitive, "Pepper3");

  // move to target pose
  // moveToPose(target_pose);
  multiframe();

  // // remove obstacles from scene
  // std::vector<std::string> object_ids;
  // object_ids.push_back("Pepper1");
  // object_ids.push_back("Pepper2");
  // object_ids.push_back("Pepper3");
  // removeObstacles(object_ids);
}








// void followCartesianPath()
// {
//   // Cartesian Paths
//   // ^^^^^^^^^^^^^^^
//   // You can plan a Cartesian path directly by specifying a list of waypoints
//   // for the end-effector to go through. Note that we are starting
//   // from the new start state above.  The initial pose (start state) does not
//   // need to be added to the waypoint list but adding it can help with visualizations
//   moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//   const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//   moveit_visual_tools::MoveItVisualTools visual_tools("base");
//   visual_tools.deleteAllMarkers();

//   // Remote control is an introspection tool that allows users to step through a high level script
//   // via buttons and keyboard shortcuts in RViz
//   visual_tools.loadRemoteControl();

//   // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
//   Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//   text_pose.translation().z() = 1.0;
//   visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

//   // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
//   visual_tools.trigger();

//   // Getting Basic Information
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^
//   // We can print the name of the reference frame for this robot
//   ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

//   // We can also print the name of the end-effector link for this group.
//   ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

//   // We can get a list of all the groups in the robot:
//   ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
//   std::copy(move_group_interface.getJointModelGroupNames().begin(),
//             move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  
  // std::vector<geometry_msgs::Pose> waypoints;
  // waypoints.push_back(pre_grasp);

  // geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
  // geometry_msgs::Pose new_pose = current_pose.pose;
  //   // target_pose3.position.z -= 0.2;
  // // waypoints.push_back(target_pose3);

  // // target_pose3.position.y -= 0.2;
  // // waypoints.push_back(target_pose3);

  // // new_pose.position.z += 0.05;
  // // target_pose3.position.y += 0.2;
  // new_pose.position.y -= 0.1;
  // waypoints.push_back(new_pose);  // move out by 10cm

  // // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // // which is why we will specify 0.01 as the max step in Cartesian
  // // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // // Warning - disabling the jump threshold while operating real hardware can cause
  // // large unpredictable motions of redundant joints and could be a safety issue
  // moveit_msgs::RobotTrajectory trajectory;
  // const double jump_threshold = 0.01;
  // const double eef_step = 0.00001;
  // double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  // // double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, trajectory);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // // Visualize the plan in RViz
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  // for (std::size_t i = 0; i < waypoints.size(); ++i)
  //   visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  // visual_tools.trigger();
  // visual_tools.prompt("Cartesian Move?");

  // // Cartesian motions should often be slow, e.g. when approaching objects. The speed of cartesian
  // // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
  // // the trajectory manually, as described `here <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
  // // Pull requests are welcome.
  // //
  // // You can execute a trajectory like this.
  // move_group_interface.execute(trajectory);
// }


// // move to pose function
// void moveToPoseWithObstacle(geometry_msgs::Pose target_pose)
// {
//   moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//   const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//   moveit_visual_tools::MoveItVisualTools visual_tools("base");
//   visual_tools.deleteAllMarkers();

//   // Remote control is an introspection tool that allows users to step through a high level script
//   // via buttons and keyboard shortcuts in RViz
//   visual_tools.loadRemoteControl();

//   // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
//   Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//   text_pose.translation().z() = 1.0;
//   visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

//   // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
//   visual_tools.trigger();

//   // Getting Basic Information
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^
//   // We can print the name of the reference frame for this robot
//   ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

//   // We can also print the name of the end-effector link for this group.
//   ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

//   // We can get a list of all the groups in the robot:
//   ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
//   std::copy(move_group_interface.getJointModelGroupNames().begin(),
//             move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

//   // we call the planner to compute the plan and visualize it.
//   // Note that we are just planning, not asking move_group_interface
//   // to actually move the robot.
//   tf2::Quaternion q;
//   // q.setRPY(-M_PI/2,-M_PI/4,-M_PI/2); //franka
//   // q.setRPY(M_PI/2,0,M_PI/2); //kinova
//   q.setRPY(M_PI/2,0,0); //kinova
//   geometry_msgs::Quaternion quat;
//   quat = tf2::toMsg(q);
//   geometry_msgs::Pose constrained_pose;
//   constrained_pose.orientation = quat;
//   constrained_pose.position = target_pose.position;

//   move_group_interface.setPoseTarget(constrained_pose);
//   // move_group_interface.setPoseTarget(target_pose);
//   move_group_interface.setPlanningTime(15.0);
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

//   // // visualize plan
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
//   visual_tools.publishAxisLabeled(target_pose, "pose1");
//   // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("add collision object?");

//   moveit_msgs::CollisionObject collision_object1;
//   collision_object1.header.frame_id = move_group_interface.getPlanningFrame();

//   // The id of the object is used to identify it.
//   collision_object1.id = "box1";


//   moveit_msgs::CollisionObject collision_object2;
//   collision_object2.header.frame_id = move_group_interface.getPlanningFrame();

//   // The id of the object is used to identify it.
//   collision_object2.id = "box2";


//   moveit_msgs::CollisionObject collision_object3;
//   collision_object3.header.frame_id = move_group_interface.getPlanningFrame();

//   // The id of the object is used to identify it.
//   // collision_object3.id = "box3";

//   // Define a box to add to the world.
//   shape_msgs::SolidPrimitive primitive1;
//   primitive1.type = primitive1.BOX;
//   primitive1.dimensions.resize(3);
//   primitive1.dimensions[primitive1.BOX_X] = 0.05;
//   primitive1.dimensions[primitive1.BOX_Y] = 0.05;
//   primitive1.dimensions[primitive1.BOX_Z] = 0.07;

//     // Define a box to add to the world.
//   shape_msgs::SolidPrimitive primitive2;
//   primitive2.type = primitive2.BOX;
//   primitive2.dimensions.resize(3);
//   primitive2.dimensions[primitive2.BOX_X] = 0.05;
//   primitive2.dimensions[primitive2.BOX_Y] = 0.05;
//   primitive2.dimensions[primitive2.BOX_Z] = 0.07;

//     // Define a box to add to the world.
//   // shape_msgs::SolidPrimitive primitive3;
//   // primitive3.type = primitive3.BOX;
//   // primitive3.dimensions.resize(3);
//   // primitive3.dimensions[primitive3.BOX_X] = 0.05;
//   // primitive3.dimensions[primitive3.BOX_Y] = 0.05;
//   // primitive3.dimensions[primitive3.BOX_Z] = 0.07;

//   // Define a pose for the box (specified relative to frame_id)
//   geometry_msgs::Pose box_pose1;
//   box_pose1.orientation.w = 1.0;
//   box_pose1.position.x = 0.1;
//   box_pose1.position.y = -0.3;
//   box_pose1.position.z = 0.4;

//   // Define a pose for the box (specified relative to frame_id)
//   geometry_msgs::Pose box_pose2;
//   box_pose2.orientation.w = 1.0;
//   box_pose2.position.x = 0.6;
//   box_pose2.position.y = 0.3;
//   box_pose2.position.z = 0.3;

//     // Define a pose for the box (specified relative to frame_id)
//   // geometry_msgs::Pose box_pose3;
//   // box_pose3.orientation.w = 1.0;
//   // box_pose3.position.x = 0.6;
//   // box_pose3.position.y = -0.4;
//   // box_pose3.position.z = 0.8;

//   collision_object1.primitives.push_back(primitive1);
//   collision_object1.primitive_poses.push_back(box_pose1);
//   collision_object1.operation = collision_object1.ADD;

//   // collision_object2.primitives.push_back(primitive2);
//   // collision_object2.primitive_poses.push_back(box_pose2);
//   // collision_object2.operation = collision_object2.ADD;

//   // collision_object3.primitives.push_back(primitive3);
//   // collision_object3.primitive_poses.push_back(box_pose3);
//   // collision_object3.operation = collision_object3.ADD;

//   std::vector<moveit_msgs::CollisionObject> collision_objects;
//   collision_objects.push_back(collision_object1);
//   // collision_objects.push_back(collision_object2);
//   // collision_objects.push_back(collision_object3);

//   // Now, let's add the collision object into the world
//   // (using a vector that could contain additional objects)
//   ROS_INFO_NAMED("tutorial", "Add an object into the world");
//   planning_scene_interface.addCollisionObjects(collision_objects);

//   // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
//   visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();
//   visual_tools.prompt("plan around object?");

//   // Now when we plan a trajectory it will avoid the obstacle
//   success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan (pose goal move around obstacle(s)) %s", success ? "" : "FAILED");
//   visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("execute?");
//   move_group_interface.move();

//   std::vector<std::string> object_ids;
//   object_ids.push_back(collision_object1.id);
//   // object_ids.push_back(collision_object2.id);
//   // object_ids.push_back(collision_object3.id);
//   planning_scene_interface.removeCollisionObjects(object_ids);

// //   // Show text in RViz of status
//   visual_tools.publishText(text_pose, "Objects removed", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();
// }



// // move to pose with constraint
// void moveToPoseWithConstraint(geometry_msgs::Pose target_pose)
// {
//   moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//   const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//   moveit_visual_tools::MoveItVisualTools visual_tools("base");
//   visual_tools.deleteAllMarkers();

//   // Remote control is an introspection tool that allows users to step through a high level script
//   // via buttons and keyboard shortcuts in RViz
//   visual_tools.loadRemoteControl();

//   // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
//   Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//   text_pose.translation().z() = 1.0;
//   visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

//   // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
//   visual_tools.trigger();

//   // Getting Basic Information
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^
//   // We can print the name of the reference frame for this robot
//   ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

//   // We can also print the name of the end-effector link for this group.
//   ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

//   // We can get a list of all the groups in the robot:
//   ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
//   std::copy(move_group_interface.getJointModelGroupNames().begin(),
//             move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

//   // we call the planner to compute the plan and visualize it.
//   // Note that we are just planning, not asking move_group_interface
//   // to actually move the robot.
//   move_group_interface.setPoseTarget(target_pose);
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

//   // visualize plan
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
//   visual_tools.publishAxisLabeled(target_pose, "pose1");
//   // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();


//   tf2::Quaternion q;
//   q.setRPY(M_PI/2,0,0); //kinova
//   geometry_msgs::Quaternion quat;
//   quat = tf2::toMsg(q);
//   // geometry_msgs::Pose constrained_pose;
//   // constrained_pose.orientation = quat;
//   // constrained_pose.position.x = target_pose.position.x;
//   // constrained_pose.position.y = target_pose.position.y + 0.1;
//   // constrained_pose.position.z = target_pose.position.z;

//   moveit_msgs::OrientationConstraint ocm;
//   ocm.link_name = "end_effector_link";
//   ocm.header.frame_id = "base_link";
//   ocm.orientation = quat;
//   ocm.absolute_x_axis_tolerance = 0.1;
//   ocm.absolute_y_axis_tolerance = 0.1;
//   ocm.absolute_z_axis_tolerance = 0.1;
//   ocm.weight = 1.0;

// //   // Now, set it as the path constraint for the group.
//   moveit_msgs::Constraints test_constraints;
//   test_constraints.orientation_constraints.push_back(ocm);
//   move_group_interface.setPathConstraints(test_constraints);

//   move_group_interface.setPoseTarget(target_pose);

//   // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
//   // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
//   move_group_interface.setPlanningTime(15.0);

//   success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

//   // Visualize the plan in RViz
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("execute?");

//   move_group_interface.move();

//   // When done with the path constraint be sure to clear it.
//   move_group_interface.clearPathConstraints();
// }










// int main(int argc, char*ss* argv)
// {
//   ros::init(argc, argv, "commander_node");
//   ros::NodeHandle node_handle;

//   // ROS spinning must be running for the MoveGroupInterface to get information
//   // about the robot's state. One way to do this is to start an AsyncSpinner
//   // beforehand.
//   ros::AsyncSpinner spinner(1);
//   spinner.start();


//   // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
//   // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
//   // are used interchangeably.
//   static const std::string PLANNING_GROUP = "right_arm";

//   // The :planning_interface:`MoveGroupInterface` class can be easily
//   // setup using just the name of the planning group you would like to control and plan for.
//   moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

//   // We will use the :planning_interface:`PlanningSceneInterface`
//   // class to add and remove collision objects in our "virtual world" scene
//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

//   // Raw pointers are frequently used to refer to the planning group for improved performance.
//   const moveit::core::JointModelGroup* joint_model_group =
//       move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//   // Visualization
//   // ^^^^^^^^^^^^^
//   // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
//   // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
//   namespace rvt = rviz_visual_tools;
//   moveit_visual_tools::MoveItVisualTools visual_tools("base");
//   visual_tools.deleteAllMarkers();

//   // Remote control is an introspection tool that allows users to step through a high level script
//   // via buttons and keyboard shortcuts in RViz
//   visual_tools.loadRemoteControl();

//   // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
//   Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//   text_pose.translation().z() = 1.0;
//   visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

//   // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
//   visual_tools.trigger();


//   // Getting Basic Information
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^
//   // We can print the name of the reference frame for this robot
//   ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

//   // We can also print the name of the end-effector link for this group.
//   ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

//   // We can get a list of all the groups in the robot:
//   ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
//   std::copy(move_group_interface.getJointModelGroupNames().begin(),
//             move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

//   // Start the demo
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

//   // .. _move_group_interface-planning-to-pose-goal:
//   //
//   // Planning to a Pose goal
//   // ^^^^^^^^^^^^^^^^^^^^^^^
//   // We can plan a motion for this group to a desired pose for the
//   // end-effector.
//   geometry_msgs::Pose target_pose1;
//   target_pose1.orientation.w = 1.0;
//   target_pose1.position.x = 0.28;
//   target_pose1.position.y = -0.2;
//   target_pose1.position.z = 0.5;
//   move_group_interface.setPoseTarget(target_pose1);

//   // Now, we call the planner to compute the plan and visualize it.
//   // Note that we are just planning, not asking move_group_interface
//   // to actually move the robot.
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//   bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//   ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

//   // Visualizing plans
//   // ^^^^^^^^^^^^^^^^^
//   // We can also visualize the plan as a line with markers in RViz.
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
//   visual_tools.publishAxisLabeled(target_pose1, "pose1");
//   visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
//   // Note that this can lead to problems if the robot moved in the meanwhile.
//   // move_group_interface.execute(my_plan);

//   // Moving to a pose goal
//   // ^^^^^^^^^^^^^^^^^^^^^
//   //
//   // If you do not want to inspect the planned trajectory,
//   // the following is a more robust combination of the two-step plan+execute pattern shown above
//   // and should be preferred. Note that the pose goal we had set earlier is still active,
//   // so the robot will try to move to that goal.

//   move_group_interface.move();

//   // Planning to a joint-space goal
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // Let's set a joint space goal and move towards it.  This will replace the
//   // pose target we set above.
//   //
//   // To start, we'll create an pointer that references the current robot's state.
//   // RobotState is the object that contains all the current position/velocity/acceleration data.
//   moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
//   //
//   // Next get the current set of joint values for the group.
//   std::vector<double> joint_group_positions;
//   current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

//   // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
//   joint_group_positions[0] = -tau / 6;  // -1/6 turn in radians
//   move_group_interface.setJointValueTarget(joint_group_positions);

//   // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
//   // The default values are 10% (0.1).
//   // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
//   // or set explicit factors in your code if you need your robot to move faster.
//   move_group_interface.setMaxVelocityScalingFactor(0.05);
//   move_group_interface.setMaxAccelerationScalingFactor(0.05);

//   success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

//   // Visualize the plan in RViz
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // Planning with Path Constraints
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   // 
//   // Path constraints can easily be specified for a link on the robot.
//   // Let's specify a path constraint and a pose goal for our group.
//   // First define the path constraint.
//   moveit_msgs::OrientationConstraint ocm;
//   ocm.link_name = "right_hand";
//   ocm.header.frame_id = "base";
//   ocm.orientation.w = 1.0;
//   ocm.absolute_x_axis_tolerance = 0.1;
//   ocm.absolute_y_axis_tolerance = 0.1;
//   ocm.absolute_z_axis_tolerance = 0.1;
//   ocm.weight = 1.0;

//   // Now, set it as the path constraint for the group.
//   moveit_msgs::Constraints test_constraints;
//   test_constraints.orientation_constraints.push_back(ocm);
//   move_group_interface.setPathConstraints(test_constraints);


//   // Enforce Planning in Joint Space
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // Depending on the planning problem MoveIt chooses between
//   // ``joint space`` and ``cartesian space`` for problem representation.
//   // Setting the group parameter ``enforce_joint_model_state_space:true`` in
//   // the ompl_planning.yaml file enforces the use of ``joint space`` for all plans.
//   //
//   // By default planning requests with orientation path constraints
//   // are sampled in ``cartesian space`` so that invoking IK serves as a
//   // generative sampler.
//   //
//   // By enforcing ``joint space`` the planning process will use rejection
//   // sampling to find valid requests. Please note that this might
//   // increase planning time considerably.
//   //
//   // We will reuse the old goal that we had and plan to it.
//   // Note that this will only work if the current state already
//   // satisfies the path constraints. So we need to set the start
//   // state to a new pose.
//   moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
//   geometry_msgs::Pose start_pose2;
//   start_pose2.orientation.w = 1.0;
//   start_pose2.position.x = 0.55;
//   start_pose2.position.y = -0.05;
//   start_pose2.position.z = 0.8;
//   start_state.setFromIK(joint_model_group, start_pose2);
//   move_group_interface.setStartState(start_state);

//   // Now we will plan to the earlier pose target from the new
//   // start state that we have just created.
//   move_group_interface.setPoseTarget(target_pose1);

//   // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
//   // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
//   move_group_interface.setPlanningTime(10.0);

//   success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

//   // Visualize the plan in RViz
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishAxisLabeled(start_pose2, "start");
//   visual_tools.publishAxisLabeled(target_pose1, "goal");
//   visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("next step");

//   // When done with the path constraint be sure to clear it.
//   move_group_interface.clearPathConstraints();

//   // Cartesian Paths
//   // ^^^^^^^^^^^^^^^
//   // You can plan a Cartesian path directly by specifying a list of waypoints
//   // for the end-effector to go through. Note that we are starting
//   // from the new start state above.  The initial pose (start state) does not
//   // need to be added to the waypoint list but adding it can help with visualizations
//   std::vector<geometry_msgs::Pose> waypoints;
//   waypoints.push_back(start_pose2);

//   geometry_msgs::Pose target_pose3 = start_pose2;

//   target_pose3.position.z -= 0.2;
//   waypoints.push_back(target_pose3);  // down

//   target_pose3.position.y -= 0.2;
//   waypoints.push_back(target_pose3);  // right

//   target_pose3.position.z += 0.2;
//   target_pose3.position.y += 0.2;
//   target_pose3.position.x -= 0.2;
//   waypoints.push_back(target_pose3);  // up and left

//   // We want the Cartesian path to be interpolated at a resolution of 1 cm
//   // which is why we will specify 0.01 as the max step in Cartesian
//   // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
//   // Warning - disabling the jump threshold while operating real hardware can cause
//   // large unpredictable motions of redundant joints and could be a safety issue
//   moveit_msgs::RobotTrajectory trajectory;
//   const double jump_threshold = 0.0;
//   const double eef_step = 0.01;
//   double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

//   // Visualize the plan in RViz
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
//   for (std::size_t i = 0; i < waypoints.size(); ++i)
//     visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // Cartesian motions should often be slow, e.g. when approaching objects. The speed of cartesian
//   // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
//   // the trajectory manually, as described `here <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
//   // Pull requests are welcome.
//   //
//   // You can execute a trajectory like this.
//   move_group_interface.execute(trajectory);

//   // Adding objects to the environment
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // First let's plan to another simple goal with no objects in the way.
//   move_group_interface.setStartState(*move_group_interface.getCurrentState());
//   geometry_msgs::Pose another_pose;
//   another_pose.orientation.x = 1.0;
//   another_pose.position.x = 0.7;
//   another_pose.position.y = 0.0;
//   another_pose.position.z = 0.59;
//   move_group_interface.setPoseTarget(another_pose);

//   success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");

//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Clear Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("next step");

//   // The result may look like this:
//   //
//   // .. image:: ./move_group_interface_tutorial_clear_path.gif
//   //    :alt: animation showing the arm moving relatively straight toward the goal
//   //
//   // Now let's define a collision object ROS message for the robot to avoid.
//   moveit_msgs::CollisionObject collision_object;
//   collision_object.header.frame_id = move_group_interface.getPlanningFrame();

//   // The id of the object is used to identify it.
//   collision_object.id = "box1";

//   // Define a box to add to the world.
//   shape_msgs::SolidPrimitive primitive;
//   primitive.type = primitive.BOX;
//   primitive.dimensions.resize(3);
//   primitive.dimensions[primitive.BOX_X] = 0.1;
//   primitive.dimensions[primitive.BOX_Y] = 1.5;
//   primitive.dimensions[primitive.BOX_Z] = 0.5;

//   // Define a pose for the box (specified relative to frame_id)
//   geometry_msgs::Pose box_pose;
//   box_pose.orientation.w = 1.0;
//   box_pose.position.x = 0.5;
//   box_pose.position.y = 0.0;
//   box_pose.position.z = 0.25;

//   collision_object.primitives.push_back(primitive);
//   collision_object.primitive_poses.push_back(box_pose);
//   collision_object.operation = collision_object.ADD;

//   std::vector<moveit_msgs::CollisionObject> collision_objects;
//   collision_objects.push_back(collision_object);

//   // Now, let's add the collision object into the world
//   // (using a vector that could contain additional objects)
//   ROS_INFO_NAMED("tutorial", "Add an object into the world");
//   planning_scene_interface.addCollisionObjects(collision_objects);

//   // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
//   visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

//   // Now when we plan a trajectory it will avoid the obstacle
//   success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
//   visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

//   // The result may look like this:
//   //
//   // .. image:: ./move_group_interface_tutorial_avoid_path.gif
//   //    :alt: animation showing the arm moving avoiding the new obstacle
//   //
//   // Attaching objects to the robot
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // You can attach objects to the robot, so that it moves with the robot geometry.
//   // This simulates picking up the object for the purpose of manipulating it.
//   // The motion planning should avoid collisions between the two objects as well.
//   moveit_msgs::CollisionObject object_to_attach;
//   object_to_attach.id = "cylinder1";

//   shape_msgs::SolidPrimitive cylinder_primitive;
//   cylinder_primitive.type = primitive.CYLINDER;
//   cylinder_primitive.dimensions.resize(2);
//   cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
//   cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

//   // We define the frame/pose for this cylinder so that it appears in the gripper
//   object_to_attach.header.frame_id = move_group_interface.getEndEffectorLink();
//   geometry_msgs::Pose grab_pose;
//   grab_pose.orientation.w = 1.0;
//   grab_pose.position.z = 0.1; //sri: changed to 0

//   // First, we add the object to the world (without using a vector)
//   object_to_attach.primitives.push_back(cylinder_primitive);
//   object_to_attach.primitive_poses.push_back(grab_pose);
//   object_to_attach.operation = object_to_attach.ADD;
//   planning_scene_interface.applyCollisionObject(object_to_attach);

//   // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
//   // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
//   ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
//   move_group_interface.attachObject(object_to_attach.id, "right_hand");

//   visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

//   // Replan, but now with the object in hand.
//   move_group_interface.setStartStateToCurrentState();
//   success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

//   // The result may look something like this:
//   //
//   // .. image:: ./move_group_interface_tutorial_attached_object.gif
//   //    :alt: animation showing the arm moving differently once the object is attached
//   //
//   // Detaching and Removing Objects
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // Now, let's detach the cylinder from the robot's gripper.
//   ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
//   move_group_interface.detachObject(object_to_attach.id);

//   // Show text in RViz of status
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Object detached from robot", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

//   // Now, let's remove the objects from the world.
//   ROS_INFO_NAMED("tutorial", "Remove the objects from the world");
//   std::vector<std::string> object_ids;
//   object_ids.push_back(collision_object.id);
//   object_ids.push_back(object_to_attach.id);
//   planning_scene_interface.removeCollisionObjects(object_ids);

//   // Show text in RViz of status
//   visual_tools.publishText(text_pose, "Objects removed", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");

//   // END_TUTORIAL

//   ros::shutdown();
//   return 0;
// }
