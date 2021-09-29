#ifndef MOVEIT_PLANNER_POST_PROCESSOR
#define MOVEIT_PLANNER_POST_PROCESSOR

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayTrajectory.h>

#include <vector>

// #include <std_msgs/String.h>

#include "open_manipulator_msgs/OpenManipulatorState.h"

#include "open_manipulator_msgs/GetJointPosition.h"
#include "open_manipulator_msgs/GetKinematicsPose.h"

// #include "open_manipulator_msgs/SetJointPosition.h"
// #include "open_manipulator_msgs/SetKinematicsPose.h"

class MoveitPlannerPostProcessor {
 public:
  MoveitPlannerPostProcessor();
  virtual ~MoveitPlannerPostProcessor();

 private:
  //   void ControlCallback(const ros::TimerEvent&);
  void Initialise();

  bool PlanPath(const std::string planning_group,
                open_manipulator_msgs::JointPosition msg);
  bool PlanPath(const std::string planning_group,
                open_manipulator_msgs::KinematicsPose msg);
  void DisplayPlannedPathMsgCallback(
      const moveit_msgs::DisplayTrajectory::ConstPtr& msg);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  std::string planning_group_;
  ros::Publisher dynamixel_workbench_pub_;
  ros::Subscriber display_planned_path_sub_;
  moveit::planning_interface::MoveGroupInterface* move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  
  // joint_names: [base_z_rot, base_y_rot, elbow_y_rot, wrist_y_rot, gripper_z_rot]
  // float angle_offset_[5] = {1.9789, 2.6435, 2.6077, 2.6334, 0.0};  // TODO
  float angle_offset_[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
};

#endif /* MOVEIT_PLANNER_POST_PROCESSOR */
