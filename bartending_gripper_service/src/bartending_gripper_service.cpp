#include "bartending_gripper_service.hpp"

BartendingGripper::BartendingGripper() { Initialise(); }

void BartendingGripper::Initialise() {
  motor_cmd_.request.id = 6;
  motor_cmd_.request.addr_name = "Goal_Position";
  client_motor_control_ =
      nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>(
          "/bartender/dynamixel_command");
  service_open_ = nh_.advertiseService("bartender/open_gripper",
                                       &BartendingGripper::Open, this);
  service_close_ = nh_.advertiseService("bartender/close_gripper",
                                        &BartendingGripper::Close, this);
}

bool BartendingGripper::Open(std_srvs::Trigger::Request& req,
                             std_srvs::Trigger::Response& res) {
  ROS_INFO("Opening gripper...\n");
  motor_cmd_.request.value = 500;
  if (client_motor_control_.call(motor_cmd_)) {
    if (motor_cmd_.response.comm_result) {
      ROS_INFO("gripper open success.\n");
      res.success = true;
      return true;
    } else {
      ROS_INFO("gripper open failed.\n");
    }
  } else {
    ROS_INFO("Dynamixel command failed.\n");
  }
  res.success = false;
  return false;
}

bool BartendingGripper::Close(bartending_gripper_service::CloseGripper::Request& req,
                              bartending_gripper_service::CloseGripper::Response& res) {
  ROS_INFO("Closing gripper...\n");
  if (req.motor_position < 232) {
    ROS_INFO("Setting motor command to 232.\n");
    motor_cmd_.request.value = 232;
  } else if (req.motor_position > 650) {
    ROS_INFO("Setting motor command to 650.\n");
    motor_cmd_.request.value = 650;
  } else {
    motor_cmd_.request.value = req.motor_position;
  }
  if (client_motor_control_.call(motor_cmd_)) {
    if (motor_cmd_.response.comm_result) {
      ROS_INFO("gripper close success.\n");
      res.success = true;
      return true;
    } else {
      ROS_INFO("gripper close failed.\n");
    }
  } else {
    ROS_INFO("Dynamixel command failed.\n");
  }
  res.success = false;
  return false;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "bartending_gripper_node");
  BartendingGripper gripper;
  ros::spin();
  return 0;
}