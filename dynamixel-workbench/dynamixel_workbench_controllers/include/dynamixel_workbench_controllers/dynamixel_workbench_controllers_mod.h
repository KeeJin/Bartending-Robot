#ifndef DYNAMIXEL_WORKBENCH_CONTROLLERS_20MOD
#define DYNAMIXEL_WORKBENCH_CONTROLLERS_20MOD

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

#include <dynamixel_workbench_controllers/trajectory_generator.h>

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

// Protocol 2.0
#define ADDR_PRESENT_CURRENT_2 126
#define ADDR_PRESENT_VELOCITY_2 128
#define ADDR_PRESENT_POSITION_2 132

#define LENGTH_PRESENT_CURRENT_2 2
#define LENGTH_PRESENT_VELOCITY_2 4
#define LENGTH_PRESENT_POSITION_2 4

// Protocol 2.0 (XL-320)
#define ADDR_PRESENT_LOAD_XL_320  41
#define ADDR_PRESENT_VELOCITY_XL_320 39
#define ADDR_PRESENT_POSITION_XL_320 37

#define LENGTH_PRESENT_LOAD_XL_320 2
#define LENGTH_PRESENT_VELOCITY_XL_320 2
#define LENGTH_PRESENT_POSITION_XL_320 2

// Protocol 1.0
#define ADDR_PRESENT_LOAD_1  40
#define ADDR_PRESENT_VELOCITY_1 38
#define ADDR_PRESENT_POSITION_1 36

#define LENGTH_PRESENT_LOAD_1 2
#define LENGTH_PRESENT_VELOCITY_1 2
#define LENGTH_PRESENT_POSITION_1 2

//#define DEBUG

typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

class DynamixelController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher dynamixel_state_list_pub_;
  ros::Publisher joint_states_pub_;

  // ROS Topic Subscriber
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber trajectory_sub_;

  // ROS Service Server
  ros::ServiceServer dynamixel_command_server_;

  // ROS Service Client

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;
  std::map<std::string, uint32_t> dynamixel_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list_;
  // float joint_state_offset_[5] = {-1.2067, 0.0, 0.0, 0.0, 0.6443};
  float joint_state_offset_[5] = {-1.2067, 0.0, 0.0, 0.0, 0.0};
  sensor_msgs::JointState joint_state_msg_;
  std::vector<WayPoint> pre_goal_;

  bool is_joint_state_topic_;
  bool is_cmd_vel_topic_;
  bool use_moveit_;

  double wheel_separation_;
  double wheel_radius_;

  JointTrajectory *jnt_tra_;
  trajectory_msgs::JointTrajectory *jnt_tra_msg_;

  double read_period_;
  double write_period_;
  double pub_period_;

  bool is_moving_;

 public:
  DynamixelController();
  ~DynamixelController();

  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool getDynamixelsInfo(const std::string yaml_file);
  bool loadDynamixels(void);
  bool initDynamixels(void);
  bool initSDKHandlers(void);
  bool getPresentPosition(std::vector<std::string> dxl_name);

  double getReadPeriod(){return read_period_;}
  double getWritePeriod(){return write_period_;}
  double getPublishPeriod(){return pub_period_;}

  void initPublisher(void);
  void initSubscriber(void);

  void initServer();

  void readCallback(const ros::TimerEvent&);
  void writeCallback(const ros::TimerEvent&);
  void publishCallback(const ros::TimerEvent&);

  void commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void trajectoryMsgCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
  bool dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                   dynamixel_workbench_msgs::DynamixelCommand::Response &res);
};

#endif /* DYNAMIXEL_WORKBENCH_CONTROLLERS_20MOD */
