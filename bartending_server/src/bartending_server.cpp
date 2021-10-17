#include "bartending_server.hpp"
#include "open_manipulator_msgs/SetJointPosition.h"
#include "bartending_gripper_service/CloseGripper.h"
#include <std_srvs/Trigger.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <unistd.h>

void BartendingServer::Initialise() {
  // Set up ROS communication
  service_main_ =
      nh_.advertiseService("bartender/drink_request",
                           &BartendingServer::HandleCustomerRequest, this);
  client_move_arm_ = nh_.serviceClient<open_manipulator_msgs::SetJointPosition>(
      "/bartender_arm/moveit/set_joint_position");
  client_gripper_open_ =
      nh_.serviceClient<std_srvs::Trigger>("bartender/open_gripper");
  client_gripper_close_ =
      nh_.serviceClient<bartending_gripper_service::CloseGripper>(
          "/bartender/close_gripper");
  client_motor_control_ =
      nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>(
          "/bartender/dynamixel_command");

  // Calculate IK for all objects once
  GenerateIKSolution("shaker", shaker_position_);
  GenerateIKSolution("pour", pouring_position_);
  GenerateIKSolution("shaker_cap", shaker_cap_position_);
  GenerateIKSolution("jager", jager_position_);
  GenerateIKSolution("vodka", vodka_position_);
  GenerateIKSolution("redbull", redbull_position_);
  GenerateIKSolution("sprite", sprite_position_);
  GenerateIKSolution("water", water_position_);
  GenerateIKSolution("shaker_offset", shaker_position_, offset_);
  GenerateIKSolution("shaker_cap_offset", shaker_cap_position_, offset_);
  GenerateIKSolution("jager_offset", jager_position_, offset_);
  GenerateIKSolution("vodka_offset", vodka_position_, offset_);
  GenerateIKSolution("redbull_offset", redbull_position_, offset_);
  GenerateIKSolution("sprite_offset", sprite_position_, offset_);
  GenerateIKSolution("water_offset", water_position_, offset_);
}

void BartendingServer::GenerateIKSolution(std::string object, Position position,
                                          double offset) {
  ROS_INFO_STREAM("Generating IK solutions for: " << object);

  std::vector<double> joint_states;
  double l2 = 0.034;
  double l3 = 0.034;
  double a1, a2, a3, a4, a5;
  double b1, b2;
  double distance = position.distance - offset;

  ROS_INFO_STREAM("Section: " << position.section);
  ROS_INFO_STREAM("Distance: " << distance);
  ROS_INFO_STREAM("Height: " << position.height);

  // Solve for a1
  a1 = (double)position.section / 180 * M_PI;
  ROS_INFO_STREAM("Calculated angle 1: " << a1);
  joint_states.push_back(a1);

  // Solve for a3
  double squared_sum = distance * distance + position.height * position.height;
  a3 = acos((squared_sum - l2 * l2 - l3 * l3) / (2 * l2 * l3));
  printf("%.3f\n", (squared_sum - l2 * l2 - l3 * l3) / (2 * l2 * l3));
  printf("%.3f\n", acos((squared_sum - l2 * l2 - l3 * l3) / (2 * l2 * l3)));

  // Solve for a2
  b1 = M_PI_2 - atan2(position.height, distance);
  b2 = atan2(l3 * sin(a3), l2 + l3 * cos(a3));
  a2 = b1 - b2;

  ROS_INFO_STREAM("Calculated angle 2: " << a2);
  ROS_INFO_STREAM("Calculated angle 3: " << a3);
  joint_states.push_back(a2);
  joint_states.push_back(a3);

  // Solve for a4
  a4 = (a2 + a3) - M_PI_2;
  joint_states.push_back(a4);
  ROS_INFO_STREAM("Calculated angle 4: " << a4);

  a5 = 0.0;
  joint_states.push_back(a5);
  ROS_INFO_STREAM("Calculated angle 5: " << a5);

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
  return true;
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
  std::string search_offset;
  std::string search_approach;
  switch (current_request_.alcohol) {
    case Alcohol::Jager:
      search_offset = "jager_offset";
      search_approach = "jager";
      break;
    case Alcohol::Vodka:
      search_offset = "vodka_offset";
      search_approach = "vodka";
      break;
    default:
      ROS_INFO("Entry not recognised!");
      return false;
  }

  // Move arm to offset position
  ROS_INFO("Moving to offset");
  if (!MoveTo(search_offset)) {
    return false;
  }

  // Move arm to alcohol position poised to grip
  ROS_INFO("Moving to alcohol position");
  if (!MoveTo(search_approach)) {
    return false;
  }

  // Grip
  ROS_INFO("Grasping");
  if (!Grasp(300)) {
    return false;
  }

  // Move arm to pouring position
  if (!MoveTo("pour")) {
    return false;
  }

  // Pour alcohol
  Pour(500);

  // Move arm to alcohol position poised to grip
  if (!MoveTo(search_approach, true)) {
    return false;
  }

  // Release
  OpenGripper();

  // Move arm to offset position
  if (!MoveTo(search_offset)) {
    return false;
  }
  return true;
}
bool BartendingServer::PourMixer() {
  std::string search_offset;
  std::string search_approach;
  switch (current_request_.mixer) {
    case Mixer::Water:
      search_offset = "water_offset";
      search_approach = "water";
      break;
    case Mixer::RedBull:
      search_offset = "redbull_offset";
      search_approach = "redbull";
      break;
    case Mixer::Sprite:
      search_offset = "sprite_offset";
      search_approach = "sprite";
      break;
    default:
      ROS_INFO("Entry not recognised!");
      return false;
  }

  // Move arm to offset position
  if (!MoveTo(search_offset)) {
    return false;
  }

  // Move arm to mixer position poised to grip
  if (!MoveTo(search_approach, true)) {
    return false;
  }

  // Grip
  if (!Grasp(300)) {
    return false;
  }

  // Move arm to pouring position
  if (!MoveTo("pour", true)) {
    return false;
  }

  // Pour mixer
  Pour(500);

  // Move arm to mixer position poised to grip
  if (!MoveTo(search_approach, true)) {
    return false;
  }

  // Release
  OpenGripper();

  // Move arm to offset position
  if (!MoveTo(search_offset)) {
    return false;
  }
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

bool BartendingServer::Grasp(int motor_position) {
  // bartending_gripper_service::CloseGripper msg;
  // msg.request.motor_position = 300;
  // if (client_gripper_close_.call(msg)) {
  //   if (msg.response.success) {
  //     ROS_INFO("Gripper closed successfully.");
  //     return true;
  //   }
  // }
  // ROS_INFO("Gripper failed to close.");
  // return false;
  return true;
}

bool BartendingServer::OpenGripper() {
  // std_srvs::Trigger msg;
  // if (client_gripper_open_.call(msg)) {
  //   if (msg.response.success) {
  //     ROS_INFO("Gripper opened successfully.");
  //     return true;
  //   }
  // }
  // ROS_INFO("Gripper failed to open.");
  // return false;
  return true;
}

bool BartendingServer::MoveTo(std::string goal, bool keep_level) {
  open_manipulator_msgs::SetJointPosition msg;
  msg.request.planning_group = "bartender_arm";
  auto it = joint_states_.find(goal);
  if (it == joint_states_.end()) {
    ROS_ERROR("IK entry not found!\n");
    return false;
  } else {
    ROS_INFO("Joint angles: ");
    for (auto angle : it->second) {
      ROS_INFO_STREAM("--" << angle);
    }
    ROS_INFO("--\n");
    msg.request.joint_position.position = it->second;
    if (keep_level) {
      // std::vector<std::string> grasp = {"grasp"}
      msg.request.joint_position.joint_name = {"grasp"};
    }
    if (!client_move_arm_.call(msg)) {
      ROS_ERROR("Error calling service.");
      return false;
    }
    if (msg.response.is_planned) {
      ROS_INFO_STREAM("Path plan success for " << goal);
      sleep(3);
    } else {
      ROS_INFO_STREAM("Planning failed for " << goal);
      return false;
    }
  }
  return true;
}
bool BartendingServer::Pour(int microseconds) {
  // dynamixel_workbench_msgs::DynamixelCommand motor_cmd;
  // motor_cmd.request.value = 300;
  // client_motor_control_.call(motor_cmd);
  // usleep(microseconds);
  // motor_cmd.request.value = 750;
  // client_motor_control_.call(motor_cmd);
  // sleep(3);
  return true;
}

int main(int argc, char* argv[]) {
  ROS_INFO("Starting Bartending server...\n");
  ros::init(argc, argv, "bartending_server");
  BartendingServer server;
  ros::spin();
  return 0;
}