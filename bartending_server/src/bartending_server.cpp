#include "bartending_server.hpp"
#include "open_manipulator_msgs/SetJointPosition.h"
#include "bartending_gripper_service/CloseGripper.h"
#include <std_srvs/Trigger.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <unistd.h>

// #define DEBUG

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
  GenerateIKSolution("shaker", shaker_position_, 0.0, 0.0, 0.9);
  GenerateIKSolution("pour", pouring_position_, 0.0, 0.0, 0.8);
  GenerateIKSolution("serve", serving_position_, 0.0, 0.0, 0.8);
  GenerateIKSolution("serve_offset", serving_position_, 0.06, 0.0);
  GenerateIKSolution("shake", shaking_position_, 0.0, 0.0, 0.85);
  GenerateIKSolution("home", home_position_);
  GenerateIKSolution("shaker_cap_position_put_on", shaker_cap_position_put_on_,
                     0.0, 0.0, 0.8);
  GenerateIKSolution("shaker_cap_position_take_off",
                     shaker_cap_position_take_off_, 0.0, 0.0, 0.8);
  GenerateIKSolution("shaker_cap_position_off", shaker_cap_position_off_, 0.01,
                     0.02, 0.8);
  // GenerateIKSolution("jager", jager_position_, 0.0, 0.0, 0.85);
  // GenerateIKSolution("redbull", redbull_position_, 0.0, 0.0, 0.85);
  GenerateIKSolution("shaker_offset", shaker_position_, 0.03, 0.03);
  GenerateIKSolution("shaker_cap_position_off_offset", shaker_cap_position_off_,
                     0.025, 0.01, 0.8);
  // GenerateIKSolution("jager_offset_up", jager_position_, 0.01, 0.03, 0.9);
  // GenerateIKSolution("jager_offset_back", jager_position_, 0.02, 0.0, 0.9);
  // GenerateIKSolution("redbull_offset_up", redbull_position_, 0.01, 0.03,
  // 0.9); GenerateIKSolution("redbull_offset_back", redbull_position_, 0.02,
  // 0.00, 0.9);

  GoHome();
}

void BartendingServer::GenerateIKSolution(std::string object, Position position,
                                          double offset_dist,
                                          double offset_height,
                                          float mass_discount) {
  ROS_INFO_STREAM("Generating IK solutions for: " << object);

  std::vector<double> joint_states;
  double l2 = 0.0675;
  double l3 = 0.0675;
  double a1, a2, a3, a4, a5;
  double b1, b2;
  double distance = position.distance - offset_dist;
  double height = position.height + offset_height;

  ROS_INFO_STREAM("Section: " << position.section);
  ROS_INFO_STREAM("Distance: " << distance);
  ROS_INFO_STREAM("Height: " << height);

  // Solve for a1
  a1 = (double)position.section / 180 * M_PI;
  ROS_INFO_STREAM("Calculated angle 1: " << a1 / M_PI * 180);
  joint_states.push_back(a1);

  // Solve for a3
  double squared_sum = distance * distance + height * height;
  a3 = acos((squared_sum - l2 * l2 - l3 * l3) / (2 * l2 * l3));

  // Solve for a2
  b1 = atan2(height, distance);
  b2 = atan2(l3 * sin(a3), l2 + (l3 * cos(a3)));
  // ROS_INFO_STREAM("b1: " << b1 / M_PI * 180 << ", b2: " << b2 / M_PI *
  // 180);
  a2 = M_PI_2 - (b1 + b2);

  ROS_INFO_STREAM("Calculated angle 2: " << a2 / M_PI * 180);
  ROS_INFO_STREAM("Calculated angle 3: " << a3 / M_PI * 180);
  joint_states.push_back(a2);
  joint_states.push_back(a3);

  // Solve for a4
  a4 = M_PI_2 * mass_discount - (a2 + a3);
  joint_states.push_back(a4);
  ROS_INFO_STREAM("Calculated angle 4: " << a4 / M_PI * 180);

  a5 = 0.0;
  joint_states.push_back(a5);
  ROS_INFO_STREAM("Calculated angle 5: " << a5 / M_PI * 180
                                         << "\n---------------");

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
  OpenGripper();
  if (PourAlcohol() && PourMixer() && CoverShaker() && Shake() &&
      ServeCocktail()) {
    GoHome();
    ROS_INFO("Cocktail is ready.");
    return true;
  }
  GoHome();
  return false;
}

bool BartendingServer::GoHome() { return (MoveTo("home", 5)); }

bool BartendingServer::PourAlcohol() {
  std::string search_offset_up;
  std::string search_offset_back;
  std::string search_approach;
  int gripper_close_pos = 300;
  switch (current_request_.alcohol) {
    case Alcohol::Jager:
      search_offset_up = "jager_offset_up";
      search_offset_back = "jager_offset_back";
      search_approach = "jager";
      break;
    case Alcohol::Vodka:
      // search_offset = "vodka_offset";
      // search_approach = "vodka";
      break;
    default:
      ROS_INFO("Entry not recognised!");
      return false;
  }

  // Move arm to offset position
  ROS_INFO("Moving to offset");
  if (!MoveTo(search_offset_back, 6)) {
    return false;
  }

  // Move arm to alcohol position poised to grip
  ROS_INFO("Moving to alcohol position");
  if (!MoveTo(search_approach, 4, true)) {
    return false;
  }

  // Grip
  ROS_INFO("Grasping");
  if (!Grasp(alcohol_close_)) {
    return false;
  }

  // Move arm to offset position
  ROS_INFO("Moving to offset");
  if (!MoveTo(search_offset_up, 4)) {
    return false;
  }
  ROS_INFO("Moving to offset");
  if (!MoveTo(search_offset_back, 4)) {
    return false;
  }
  GoHome();

  // Move arm to pouring position
  if (!MoveTo("pour", 6)) {
    return false;
  }

  // Pour alcohol
  Pour(Direction::Left, 1000);

  GoHome();
  // Move arm to offset position
  if (!MoveTo(search_offset_back, 5)) {
    return false;
  }

  // Move arm to alcohol position poised to release
  if (!MoveTo(search_approach, 6)) {
    return false;
  }

  // Release
  OpenGripper();

  // Move arm to offset position
  if (!MoveTo(search_offset_up, 5)) {
    return false;
  }

  if (!MoveTo(search_offset_back, 5)) {
    return false;
  }
  // GoHome();
  return true;
}
bool BartendingServer::PourMixer() {
  std::string search_offset_up;
  std::string search_offset_back;
  std::string search_approach;
  int gripper_close_pos = 300;
  switch (current_request_.mixer) {
    case Mixer::Water:
      // search_offset = "water_offset";
      // search_approach = "water";
      break;
    case Mixer::RedBull:
      search_offset_up = "redbull_offset_up";
      search_offset_back = "redbull_offset_back";
      search_approach = "redbull";
      break;
    case Mixer::Sprite:
      // search_offset = "sprite_offset";
      // search_approach = "sprite";
      break;
    default:
      ROS_INFO("Entry not recognised!");
      return false;
  }

  // Move arm to offset position
  if (!MoveTo(search_offset_back, 8)) {
    return false;
  }

  // Move arm to mixer position poised to grip
  if (!MoveTo(search_approach, 5, true)) {
    return false;
  }

  // Grip
  if (!Grasp(mixer_close_)) {
    return false;
  }

  // Move arm to offset position
  ROS_INFO("Moving to offset");
  if (!MoveTo(search_offset_up, 5)) {
    return false;
  }
  if (!MoveTo(search_offset_back, 5)) {
    return false;
  }

  GoHome();

  // Move arm to pouring position
  if (!MoveTo("pour", 6, true)) {
    return false;
  }

  // Pour mixer
  Pour(Direction::Left, 3000);

  GoHome();
  // Move arm to offset position
  if (!MoveTo(search_offset_back, 5)) {
    return false;
  }
  if (!MoveTo(search_offset_up, 5)) {
    return false;
  }

  // Move arm to mixer position poised to release
  if (!MoveTo(search_approach, 6, true)) {
    return false;
  }

  // Release
  OpenGripper();

  // Move arm to offset position
  if (!MoveTo(search_offset_up, 5)) {
    return false;
  }
  if (!MoveTo(search_offset_back, 5)) {
    return false;
  }
  GoHome();
  return true;
}

bool BartendingServer::CoverShaker() {
  // GoHome();
  // Move arm to offset position
  if (!MoveTo("shaker_cap_position_off_offset", 5)) {
    return false;
  }

  // Move arm to cap position poised to grip
  if (!MoveTo("shaker_cap_position_off", 5)) {
    return false;
  }

  // Grip
  if (!Grasp(shake_cap_close_)) {
    return false;
  }

  // Move arm to offset position
  if (!MoveTo("shaker_cap_position_off_offset", 6)) {
    return false;
  }

  // GoHome();

  // Move arm to capping position
  if (!MoveTo("shaker_cap_position_put_on", 5)) {
    return false;
  }

  // Release
  OpenGripper();

  // GoHome();

  return true;
}
bool BartendingServer::Shake() {
  OpenGripper();

  // Move arm to offset position
  if (!MoveTo("shaker_offset", 5)) {
    return false;
  }

  // Move arm to shaker position poised to grip
  if (!MoveTo("shaker", 5)) {
    return false;
  }

  // Grip
  if (!Grasp(shaker_close_)) {
    return false;
  }

  // Move arm to offset position
  if (!MoveTo("shaker_cap_position_put_on", 5)) {
    return false;
  }
  // if (!MoveTo("shaker_offset", 6)) {
  //   return false;
  // }

  // Move arm to shaking position
  if (!MoveTo("shake", 5)) {
    return false;
  }
#ifndef DEBUG
  ROS_INFO("Start shaking");
  dynamixel_workbench_msgs::DynamixelCommand motor_cmd;
  motor_cmd.request.id = 1;
  motor_cmd.request.addr_name = "Goal_Position";

  for (int i = 0; i < 3; ++i) {
    motor_cmd.request.value = 550;
    while (!client_motor_control_.call(motor_cmd)) {
    }
    sleep(2);
    motor_cmd.request.value = 950;
    while (!client_motor_control_.call(motor_cmd)) {
    }
    sleep(2);
  }
  sleep(3);
  motor_cmd.request.value = 750;
  while (!client_motor_control_.call(motor_cmd))
    ;
  sleep(3);
#endif
  // GoHome();

  // Move arm to offset position
  if (!MoveTo("shaker_cap_position_put_on", 6)) {
    return false;
  }

  // Move arm to shaker position poised to release
  if (!MoveTo("shaker", 5)) {
    return false;
  }

  // Release
  if (!OpenGripper()) {
    return false;
  }

  // Move arm to offset position
  if (!MoveTo("shaker_cap_position_put_on", 6)) {
    return false;
  }
  return true;
}

bool BartendingServer::ServeCocktail() {
  // GoHome();

  // Move to Cap
  // if (!MoveTo("shaker_cap_position_put_on", 3)) {
  //   return false;
  // }
  // if (!MoveTo("shaker_cap_position_take_off", 3, true)) {
  //   return false;
  // }

  // // Grip
  // if (!Grasp(shake_cap_close_)) {
  //   return false;
  // }

  // // Move Cap
  // if (!MoveTo("shaker_cap_position_put_on", 3, true)) {
  //   return false;
  // }

  // GoHome();

  // // Move arm to offset position
  // if (!MoveTo("shaker_cap_position_off_offset", 5)) {
  //   return false;
  // }

  // // Move Cap
  // if (!MoveTo("shaker_cap_position_off", 4)) {
  //   return false;
  // }

  // // Release
  // if (!OpenGripper()) {
  //   return false;
  // }

  // // Move arm to offset position
  // if (!MoveTo("shaker_cap_position_off_offset", 5)) {
  //   return false;
  // }

  // GoHome();

  // // Move arm to offset position
  // if (!MoveTo("shaker_offset", 5)) {
  //   return false;
  // }

  // // Move arm to shaker position poised to grip
  // if (!MoveTo("shaker", 5)) {
  //   return false;
  // }

  // // Grip
  // if (!Grasp(shaker_close_)) {
  //   return false;
  // }

  // // Move arm to offset position
  // if (!MoveTo("shaker_cap_position_put_on", 8)) {
  //   return false;
  // }

  // GoHome();
  if (!MoveTo("serve_offset", 6)) {
    return false;
  }

  // Move arm to serving position
  // if (!MoveTo("serve", 5)) {
  //   return false;
  // }

  // if (!Pour(Direction::Right, 3000)) {
  //   return false;
  // }

  // if (!MoveTo("serve_offset", 6)) {
  //   return false;
  // }
  // GoHome();

  // // Move arm to offset position
  // if (!MoveTo("shaker_offset", 8)) {
  //   return false;
  // }

  // // Move arm to shaker position poised to release
  // if (!MoveTo("shaker", 5)) {
  //   return false;
  // }

  // // Release
  // if (!OpenGripper()) {
  //   return false;
  // }

  // // Move arm to home position to rest
  // GoHome();

  return true;
}

bool BartendingServer::Grasp(int motor_position) {
#ifndef DEBUG
  bartending_gripper_service::CloseGripper msg;
  msg.request.motor_position = motor_position;
  if (client_gripper_close_.call(msg)) {
    if (msg.response.success) {
      ROS_INFO("Gripper closed successfully.");
      sleep(5);
      return true;
    }
  }
  ROS_INFO("Gripper failed to close.");
  return false;
#else
  return true;
#endif
}

bool BartendingServer::OpenGripper() {
#ifndef DEBUG
  std_srvs::Trigger msg;
  if (client_gripper_open_.call(msg)) {
    if (msg.response.success) {
      ROS_INFO("Gripper opened successfully.");
      sleep(5);
      return true;
    }
  }
  ROS_INFO("Gripper failed to open.");
  return false;
#else
  return true;
#endif
}

bool BartendingServer::MoveTo(std::string goal, int delay, bool keep_level) {
  open_manipulator_msgs::SetJointPosition msg;
  msg.request.planning_group = "bartender_arm";
  // ROS_INFO_STREAM("Moving to: " << goal);
  auto it = joint_states_.find(goal);
  if (it == joint_states_.end()) {
    ROS_ERROR_STREAM("IK entry not found for: " << goal);
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
      // msg.request.joint_position.joint_name = {"grasp"};
    }
    if (!client_move_arm_.call(msg)) {
      ROS_INFO("Error calling service.");
      return false;
    }
    if (msg.response.is_planned) {
      ROS_INFO_STREAM("Path plan success for " << goal);
      sleep(delay);
    } else {
      ROS_INFO_STREAM("Planning failed for " << goal);
      return false;
    }
  }
  return true;
}
bool BartendingServer::Pour(Direction dir, int microseconds) {
  int motor_pour;
  switch (dir) {
    case Direction::Left:
      motor_pour = 300;
      break;
    case Direction::Right:
      motor_pour = 1023;
      break;
  }
#ifndef DEBUG
  dynamixel_workbench_msgs::DynamixelCommand motor_cmd;
  motor_cmd.request.id = 1;
  motor_cmd.request.addr_name = "Goal_Position";
  motor_cmd.request.value = motor_pour;
  while (!client_motor_control_.call(motor_cmd))
    ;
  // usleep(microseconds);
  sleep(1);
  motor_cmd.request.value = 750;
  while (!client_motor_control_.call(motor_cmd))
    ;
  sleep(5);
#endif
  return true;
}

int main(int argc, char* argv[]) {
  ROS_INFO("Starting Bartending server...\n");
  ros::init(argc, argv, "bartending_server");
  BartendingServer server;
  ros::spin();
  return 0;
}