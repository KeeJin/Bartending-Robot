#include "bartending_server.hpp"
#include "open_manipulator_msgs/SetJointPosition.h"
#include "bartending_gripper_service/CloseGripper.h"
#include <std_srvs/Trigger.h>

void BartendingServer::Initialise() {
  service_main_ =
      nh_.advertiseService("bartender/drink_request",
                           &BartendingServer::HandleCustomerRequest, this);
  client_move_arm_ = nh_.serviceClient<open_manipulator_msgs::SetJointPosition>(
      "moveit/set_joint_position");
      client_gripper_open_ = nh_.serviceClient<std_srvs::Trigger>(
      "bartender/open_gripper");
  client_gripper_close_ = nh_.serviceClient<bartending_gripper_service::CloseGripper>(
      "bartender/close_gripper");
}

bool BartendingServer::HandleCustomerRequest(
    bartending_server::BartenderCocktailRequest::Request& req,
    bartending_server::BartenderCocktailRequest::Response& res) {
  res.success = true;
  return true;
}

int main(int argc, char* argv[]) {
  ROS_INFO("Starting Bartending server...\n");
  ros::init(argc, argv, "bartending_server");
  BartendingServer server;
  ros::spin();
  return 0;
}