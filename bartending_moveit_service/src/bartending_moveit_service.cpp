#include "bartending_moveit_service.hpp"

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_msgs/Constraints.h>

// debug
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <unistd.h>

MoveitPlannerPostProcessor::MoveitPlannerPostProcessor()
    : nh_(""), priv_nh_("~") {
  // Init parameter
  planning_group_ =
      priv_nh_.param<std::string>("planning_group", "manipulator");
  move_group_ =
      new moveit::planning_interface::MoveGroupInterface(planning_group_);
  Initialise();
}

MoveitPlannerPostProcessor::~MoveitPlannerPostProcessor() {
  ros::shutdown();
  return;
}

void MoveitPlannerPostProcessor::Initialise() {
  // move_group_->setPlannerId("RRTConnectkConfigDefault");
  // ROS_INFO_STREAM("getPoseReferenceFrame: " <<
  // move_group_->getPoseReferenceFrame()
  //           << std::endl);
  // std::cout << "Name: " << move_group_->getEndEffector() << std::endl;
  // std::cout << "Link: " << move_group_->getEndEffectorLink() << std::endl;
  move_group_->setNumPlanningAttempts(10);
  move_group_->setPlanningTime(10.0);

  // Create base collision object
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_->getPlanningFrame();
  collision_object.id = "table_boundary";
  shape_msgs::SolidPrimitive platform;
  platform.type = platform.BOX;
  platform.dimensions.resize(3);
  platform.dimensions[0] = 1.5;
  platform.dimensions[1] = 1.5;
  platform.dimensions[2] = 0.05;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.0;

  collision_object.primitives.push_back(platform);
  collision_object.primitive_poses.push_back(box_pose);

  // shape_msgs::SolidPrimitive bottle;
  // bottle.type = bottle.CYLINDER;
  // bottle.dimensions.resize(2);
  // bottle.dimensions[0] = 0.09;
  // bottle.dimensions[1] = 0.12;

  // geometry_msgs::Pose bottle_pose;
  // bottle_pose.orientation.w = 1.0;
  // bottle_pose.position.x = 0.05;
  // bottle_pose.position.y = 0.0;
  // bottle_pose.position.z = 0.05;

  // collision_object.primitives.push_back(bottle);
  // collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  // collision_objects.push_back(collision_object);

  ROS_INFO_NAMED("bartender_robot", "Added table base into the world");
  planning_scene_interface_.addCollisionObjects(collision_objects);

  // pubs and subs
  dynamixel_workbench_pub_ =
      priv_nh_.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory",
                                                           100);
  display_planned_path_sub_ = nh_.subscribe(
      "/move_group/display_planned_path", 100,
      &MoveitPlannerPostProcessor::DisplayPlannedPathMsgCallback, this);

  get_joint_position_server_ = priv_nh_.advertiseService(
      "moveit/get_joint_position",
      &MoveitPlannerPostProcessor::getJointPositionMsgCallback, this);
  // get_kinematics_pose_server_ = priv_nh_.advertiseService(
  //     "moveit/get_kinematics_pose",
  //     &MoveitPlannerPostProcessor::getKinematicsPoseMsgCallback, this);
  set_joint_position_server_ = priv_nh_.advertiseService(
      "moveit/set_joint_position",
      &MoveitPlannerPostProcessor::setJointPositionMsgCallback, this);
  // set_kinematics_pose_server_ = priv_nh_.advertiseService(
  //     "moveit/set_kinematics_pose",
  //     &MoveitPlannerPostProcessor::setKinematicsPoseMsgCallback, this);
}

bool MoveitPlannerPostProcessor::getJointPositionMsgCallback(
    open_manipulator_msgs::GetJointPosition::Request &req,
    open_manipulator_msgs::GetJointPosition::Response &res) {
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const std::vector<std::string> &joint_names = move_group_->getJointNames();
  std::vector<double> joint_values = move_group_->getCurrentJointValues();

  for (std::size_t i = 0; i < joint_names.size(); i++) {
    res.joint_position.joint_name.push_back(joint_names[i]);
    res.joint_position.position.push_back(joint_values[i]);
  }

  spinner.stop();
  return true;
}

// bool MoveitPlannerPostProcessor::getKinematicsPoseMsgCallback(
//     open_manipulator_msgs::GetKinematicsPose::Request &req,
//     open_manipulator_msgs::GetKinematicsPose::Response &res) {
//   ros::AsyncSpinner spinner(1);
//   spinner.start();

//   geometry_msgs::PoseStamped current_pose = move_group_->getCurrentPose();

//   res.header = current_pose.header;
//   res.kinematics_pose.pose = current_pose.pose;

//   spinner.stop();
//   return true;
// }

bool MoveitPlannerPostProcessor::setJointPositionMsgCallback(
    open_manipulator_msgs::SetJointPosition::Request &req,
    open_manipulator_msgs::SetJointPosition::Response &res) {
  open_manipulator_msgs::JointPosition msg = req.joint_position;
  for (auto position : msg.position) {
    std::cout << position / M_PI * 180.0 << "--";
  }
  std::cout << std::endl;
  std::cout << "Planning group: " << req.planning_group << std::endl;
  res.is_planned = PlanPath(req.planning_group, msg);

  return true;
}

// bool MoveitPlannerPostProcessor::setKinematicsPoseMsgCallback(
//     open_manipulator_msgs::SetKinematicsPose::Request &req,
//     open_manipulator_msgs::SetKinematicsPose::Response &res) {
//   // ROS_INFO_STREAM("Goal's frame id: " << ps.header.frame_id << std::endl;
//   rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
//   visual_tools_.reset(
//       new rviz_visual_tools::RvizVisualTools("base_platform", "/debug"));
//   visual_tools_->publishAxis(req.kinematics_pose.pose);
//   visual_tools_->trigger();
//   open_manipulator_msgs::KinematicsPose msg = req.kinematics_pose;
//   res.is_planned = PlanPath(req.planning_group, msg, false);

//   return true;
// }

// bool MoveitPlannerPostProcessor::PlanPath(
//     const std::string planning_group, open_manipulator_msgs::KinematicsPose msg,
//     bool grasping) {
//   ros::AsyncSpinner spinner(1);
//   spinner.start();
//   bool is_planned = false;
//   geometry_msgs::Pose target_pose = msg.pose;

//   // rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
//   //   visual_tools_.reset(
//   //       new rviz_visual_tools::RvizVisualTools("base_platform", "/debug"));

//   //   visual_tools_->publishAxis(target_pose);
//   //   visual_tools_->trigger();

//   // Apply constraints if needed
//   if (grasping) {
//     moveit_msgs::Constraints constraints;
//     constraints.name = "Level gripper";
//     moveit_msgs::OrientationConstraint orient_constraint;
//     orient_constraint.header.frame_id = "base_platform";
//     orient_constraint.link_name = "gripper_link";
//     orient_constraint.orientation.w = 1.0;
//     orient_constraint.absolute_x_axis_tolerance = 3.14;
//     orient_constraint.absolute_y_axis_tolerance = 3.14;
//     orient_constraint.absolute_z_axis_tolerance = 3.14;
//     orient_constraint.weight = 1.0;
//     constraints.orientation_constraints.push_back(orient_constraint);
//     move_group_->setPathConstraints(constraints);
//   }

//   if (move_group_->setApproximateJointValueTarget(target_pose,
//                                                   "gripper_link")) {
//     ROS_INFO("IK Success.\n");
//   }
//   // move_group_->setPoseTarget(target_pose);
//   // move_group_->setPositionTarget(0.15, 0.04, 0.1);
//   move_group_->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
//   move_group_->setMaxAccelerationScalingFactor(
//       msg.max_accelerations_scaling_factor);
//   move_group_->setGoalTolerance(msg.tolerance);

//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   bool success = (move_group_->plan(my_plan) ==
//                   moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   if (success) {
//     is_planned = true;
//   } else {
//     ROS_WARN("Planning (task space goal) FAILED");
//     is_planned = false;
//   }
//   spinner.stop();
//   return is_planned;
// }

bool MoveitPlannerPostProcessor::PlanPath(
    const std::string planning_group,
    open_manipulator_msgs::JointPosition msg) {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  bool is_planned = false;
  const robot_state::JointModelGroup *joint_model_group =
      move_group_->getCurrentState()->getJointModelGroup(planning_group);

  moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group,
                                         joint_group_positions);
  uint8_t joint_num = msg.position.size();
  for (uint8_t index = 0; index < joint_num; index++) {
    joint_group_positions[index] = msg.position[index];
  }

  // Apply constraints if needed
  if (msg.joint_name.size() == 1) {
    ROS_INFO("Planning with end-effector constraint.\n");
    moveit_msgs::Constraints constraints;
    constraints.name = "Level gripper";
    moveit_msgs::OrientationConstraint orient_constraint;
    orient_constraint.header.frame_id = "base_platform";
    orient_constraint.link_name = "gripper_link";
    orient_constraint.orientation.x = 0.0;
    orient_constraint.orientation.y = 0.7071068;
    orient_constraint.orientation.z = 0.0;
    orient_constraint.orientation.w = 0.7071068;
    orient_constraint.absolute_x_axis_tolerance = 0.1;
    orient_constraint.absolute_y_axis_tolerance = 0.1;
    orient_constraint.absolute_z_axis_tolerance = 3.14;
    orient_constraint.weight = 1.0;
    constraints.orientation_constraints.push_back(orient_constraint);
    move_group_->setPathConstraints(constraints);
  } else {
    ROS_INFO("Planning without any end-effector constraint.\n");
  }

  move_group_->setJointValueTarget(joint_group_positions);
  move_group_->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
  move_group_->setMaxAccelerationScalingFactor(
      msg.max_accelerations_scaling_factor);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_->plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    is_planned = true;
  } else {
    ROS_WARN("Planning (joint space goal) is FAILED");
    is_planned = false;
  }
  spinner.stop();
  return is_planned;
}

void MoveitPlannerPostProcessor::DisplayPlannedPathMsgCallback(
    const moveit_msgs::DisplayTrajectory::ConstPtr &msg) {
  ROS_INFO("Obtained Planned Path");
  trajectory_msgs::JointTrajectory jnt_tra =
      msg->trajectory[0].joint_trajectory;

  // Add offsets before publishing trajectories
  for (trajectory_msgs::JointTrajectoryPoint &point : jnt_tra.points) {
    for (int i = 0; i < 5; ++i) {
      point.positions[i] += angle_offset_[i];
    }
  }
  dynamixel_workbench_pub_.publish(jnt_tra);
}

void MoveitPlannerPostProcessor::TestPlanPath() {
  ROS_INFO("Start of Demo.\n");
  // move_group_->setStartStateToCurrentState();
  open_manipulator_msgs::KinematicsPose goal_pose;
  // goal_pose.pose.position.x = 0.111111;
  // goal_pose.pose.position.y = 0.111111;
  // goal_pose.pose.position.z = 0.233111;
  // goal_pose.pose.orientation.x = 0.076;
  // goal_pose.pose.orientation.y = 0.419;
  // goal_pose.pose.orientation.z = 0.795;
  // goal_pose.pose.orientation.w = 0.431;
  // goal_pose.max_accelerations_scaling_factor = 1.0;
  // goal_pose.max_velocity_scaling_factor = 1.0;
  // goal_pose.tolerance = 0.5;

  // open_manipulator_msgs::JointPosition goal_pos;
  // goal_pos.position = {0.0, 0.0, 0.0, 0.0};
  // goal_pos.max_accelerations_scaling_factor = 1.0;
  // goal_pos.max_velocity_scaling_factor = 1.0;
  // for (int i = 0; i < 5; ++i) {
  //   sleep(3);
  //   goal_pos.position = {-0.4, 0.4, 1.6, -0.6};
  //   PlanPath(planning_group_, goal_pos);
  //   sleep(3);
  //   goal_pos.position = {-0.4, 0.9, 1.1, -1.0};
  //   PlanPath(planning_group_, goal_pos);
  //   sleep(3);
  //   goal_pos.position = {-0.4, 0.4, 1.6, -0.6};
  //   PlanPath(planning_group_, goal_pos);
  //   sleep(3);
  //   goal_pos.position = {-0.6, -0.7791, 0.9736, 1.3542};
  //   PlanPath(planning_group_, goal_pos);
  //   sleep(3);
  //   goal_pos.position = {-1.2, 0.4, 1.6, -0.6};
  //   PlanPath(planning_group_, goal_pos);
  //   sleep(3);
  //   goal_pos.position = {-1.2, 0.9, 1.1, -1.0};
  //   PlanPath(planning_group_, goal_pos);
  //   sleep(3);
  //   goal_pos.position = {-1.2, 0.4, 1.6, -0.6};
  //   PlanPath(planning_group_, goal_pos);
  //   sleep(3);
  //   goal_pos.position = {-0.6, -0.7791, 0.9736, 1.3542};
  //   PlanPath(planning_group_, goal_pos);
  //   // move_group_->setNamedTarget("home");
  // }
  // sleep(3);
  // goal_pos.position = {-0.6, -0.7791, 0.9736, 1.5542};
  // PlanPath(planning_group_, goal_pos);

  // sleep(1);
  // for (int i = 0; i < 10; ++i) {
  //   goal_pose.pose = move_group_->getRandomPose().pose;
  //   sleep(5);
  //   PlanPath(planning_group_, goal_pose, false);
  // }
  // sleep(1);
  ROS_INFO("Demo complete!\n");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "moveit_bridge");

  MoveitPlannerPostProcessor bridge;
  // bridge.TestPlanPath();

  ros::Rate loop_rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
