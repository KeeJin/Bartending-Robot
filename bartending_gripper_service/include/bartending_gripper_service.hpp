#ifndef BARTENDING_GRIPPER_SERVICE
#define BARTENDING_GRIPPER_SERVICE

#include <ros/ros.h>
#include "bartending_gripper_service/CloseGripper.h"
#include <std_srvs/Trigger.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

class BartendingGripper {
 public:
  BartendingGripper();
  ~BartendingGripper() = default;

 private:
  void Initialise();
  bool Open(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool Close(bartending_gripper_service::CloseGripper::Request& req,
             bartending_gripper_service::CloseGripper::Response& res);

 private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_open_;
  ros::ServiceServer service_close_;
  ros::ServiceClient client_motor_control_;
  dynamixel_workbench_msgs::DynamixelCommand motor_cmd_;
};

#endif /* BARTENDING_GRIPPER_SERVICE */
