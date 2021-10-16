#ifndef BARTENDING_MOVEIT_SERVICE
#define BARTENDING_MOVEIT_SERVICE

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

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

class MoveitPlannerPostProcessor {
 public:
  MoveitPlannerPostProcessor();
  virtual ~MoveitPlannerPostProcessor();

  void TestPlanPath();

 private:
  //   void ControlCallback(const ros::TimerEvent&);
  void Initialise();

  bool PlanPath(const std::string planning_group,
                open_manipulator_msgs::JointPosition msg);
//   bool PlanPath(const std::string planning_group,
//                 open_manipulator_msgs::KinematicsPose msg, bool grasping);
  void DisplayPlannedPathMsgCallback(
      const moveit_msgs::DisplayTrajectory::ConstPtr &msg);
  bool setJointPositionMsgCallback(
      open_manipulator_msgs::SetJointPosition::Request &req,
      open_manipulator_msgs::SetJointPosition::Response &res);

//   bool setKinematicsPoseMsgCallback(
//       open_manipulator_msgs::SetKinematicsPose::Request &req,
//       open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool getJointPositionMsgCallback(
      open_manipulator_msgs::GetJointPosition::Request &req,
      open_manipulator_msgs::GetJointPosition::Response &res);

//   bool getKinematicsPoseMsgCallback(
//       open_manipulator_msgs::GetKinematicsPose::Request &req,
//       open_manipulator_msgs::GetKinematicsPose::Response &res);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  std::string planning_group_;
  ros::Publisher dynamixel_workbench_pub_;
  ros::Subscriber display_planned_path_sub_;
  ros::ServiceServer get_joint_position_server_;
//   ros::ServiceServer get_kinematics_pose_server_;
  ros::ServiceServer set_joint_position_server_;
//   ros::ServiceServer set_kinematics_pose_server_;
  moveit::planning_interface::MoveGroupInterface *move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // joint_names: [base_z_rot, base_y_rot, elbow_y_rot, wrist_y_rot,
  // gripper_z_rot] float angle_offset_[5] = {1.9789, 2.6435, 2.6077, 2.6334,
  // 0.0};  // TODO
  float angle_offset_[5] = {-0.7, -0.25, 0.0, 0.0, 0.0};
};

#endif /* BARTENDING_MOVEIT_SERVICE */
