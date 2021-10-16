#include "bartending_server.hpp"
#include "open_manipulator_msgs/SetJointPosition.h"
#include "bartending_gripper_service/CloseGripper.h"
#include <std_srvs/Trigger.h>

void BartendingServer::Initialise() {
  // Set up ROS communication
  service_main_ =
      nh_.advertiseService("bartender/drink_request",
                           &BartendingServer::HandleCustomerRequest, this);
  client_move_arm_ = nh_.serviceClient<open_manipulator_msgs::SetJointPosition>(
      "moveit/set_joint_position");
  client_gripper_open_ =
      nh_.serviceClient<std_srvs::Trigger>("bartender/open_gripper");
  client_gripper_close_ =
      nh_.serviceClient<bartending_gripper_service::CloseGripper>(
          "bartender/close_gripper");

  // Calculate IK for all objects once
  GenerateIKSolution("shaker", shaker_position_);
  GenerateIKSolution("shaker_cap", shaker_cap_position_);
  GenerateIKSolution("jager", jager_position_);
  GenerateIKSolution("vodka", vodka_position_);
  GenerateIKSolution("redbull", redbull_position_);
  GenerateIKSolution("sprite", sprite_position_);
  GenerateIKSolution("water", water_position_);
}

void BartendingServer::GenerateIKSolution(std::string object, Position position) {
  std::vector<float> joint_states;
  const float l2 = 0.0;
  const float l3 = 0.0;
  // float p3_x, p3_y, p3_z;
  float p3_floor_projection;
  float a1, a2, a3, a4;
  float b1, b2;

  // Solve for a1
  a1 = atan2(position.y, position.x);
  joint_states.push_back(a1);

  // Solve for a3
  p3_floor_projection = sqrt((position.x * position.x + position.y * position.y));
  float squared_sum = p3_floor_projection * p3_floor_projection + position.z * position.z;
  a3 = acos((squared_sum - l2*l2 - l3*l3)/(2 * l2 * l3));

  // Solve for a2
  b1 = M_PI - atan2(position.z, p3_floor_projection);
  b2 = atan2(l3 * sin(a3), l2 + l3 * cos(a3));
  a2 = b1 - b2;

  joint_states.push_back(a2);
  joint_states.push_back(a3);

  // Solve for a4
  a4 = (a2 + a3) - M_PI_2;
  joint_states.push_back(a4);

  // Insert into unordered_map
  joint_states_.insert(std::make_pair(object, joint_states));
}

bool BartendingServer::HandleCustomerRequest(
    bartending_server::BartenderCocktailRequest::Request& req,
    bartending_server::BartenderCocktailRequest::Response& res) {
  if (!InterpretRequest(req.alcohol, req.mixer)) {
    res.success = false;
    res.message = "We do not serve that here!";
    return false;
  }
  if (!PrepareCocktail()) {
    res.success = false;
    res.message = "Sorry, failed to prepare your order!";
    return false;
  }
  res.success = true;
  return true;
}

bool BartendingServer::InterpretRequest(int8_t alcohol, int8_t mixer) {
  // TODO
  current_request_.alcohol = Alcohol::Jager;
  current_request_.mixer = Mixer::RedBull;
}

bool BartendingServer::PrepareCocktail() {
  if (PourAlcohol()) {
    if (PourMixer()) {
      if (CoverShaker()) {
        if (Shake()) {
          if (ServeCocktail()) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

bool BartendingServer::PourAlcohol() {
  // TODO
  return true;
}
bool BartendingServer::PourMixer() {
  // TODO
  return true;
}
bool BartendingServer::CoverShaker() {
  // TODO
  return true;
}
bool BartendingServer::Shake() {
  // TODO
  return true;
}
bool BartendingServer::ServeCocktail() {
  // TODO
  return true;
}

int main(int argc, char* argv[]) {
  ROS_INFO("Starting Bartending server...\n");
  ros::init(argc, argv, "bartending_server");
  BartendingServer server;
  ros::spin();
  return 0;
}