#include "moveit_planner_post_processor.hpp"

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

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
  // Create base collision object
  // moveit_msgs::CollisionObject collision_object;
  // collision_object.header.frame_id = move_group_->getPlanningFrame();
  // collision_object.id = "table_boundary";
  // shape_msgs::SolidPrimitive primitive;
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[0] = 0.4;
  // primitive.dimensions[1] = 0.4;
  // primitive.dimensions[2] = 0.1;

  // geometry_msgs::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  // box_pose.position.x = 0.0;
  // box_pose.position.y = 0.0;
  // box_pose.position.z = -0.05;

  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;

  // std::vector<moveit_msgs::CollisionObject> collision_objects;
  // collision_objects.push_back(collision_object);

  // ROS_INFO_NAMED("bartender_robot", "Added table base into the world");
  // planning_scene_interface_.addCollisionObjects(collision_objects);

  // pubs and subs
  dynamixel_workbench_pub_ =
      priv_nh_.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory",
                                                           100);
  display_planned_path_sub_ = nh_.subscribe(
      "/move_group/display_planned_path", 100,
      &MoveitPlannerPostProcessor::DisplayPlannedPathMsgCallback, this);
}

bool MoveitPlannerPostProcessor::PlanPath(
    const std::string planning_group,
    open_manipulator_msgs::KinematicsPose msg) {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  bool is_planned = false;
  geometry_msgs::Pose target_pose = msg.pose;

  move_group_->setPoseTarget(target_pose);
  move_group_->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
  move_group_->setMaxAccelerationScalingFactor(
      msg.max_accelerations_scaling_factor);
  move_group_->setGoalTolerance(msg.tolerance);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_->plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    is_planned = true;
  } else {
    ROS_WARN("Planning (task space goal) is FAILED");
    is_planned = false;
  }
  spinner.stop();
  return is_planned;
}

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

int main(int argc, char **argv) {
  ros::init(argc, argv, "moveit_bridge");

  MoveitPlannerPostProcessor bridge;

  ros::Rate loop_rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
