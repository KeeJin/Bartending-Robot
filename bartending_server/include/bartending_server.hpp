#ifndef BARTENDING_SERVER
#define BARTENDING_SERVER
#include "ros/ros.h"
#include "bartending_server/BartenderCocktailRequest.h"


class BartendingServer {
  enum class Alcohol { Jager, Vodka };
  enum class Mixer { Water, RedBull, Sprite };
  struct CustomerRequest {
    Alcohol alcohol;
    Mixer mixer;
  };

 public:
  BartendingServer() { Initialise(); }
  ~BartendingServer() = default;

 private:
  void Initialise();
  bool HandleCustomerRequest(
      bartending_server::BartenderCocktailRequest::Request& req,
      bartending_server::BartenderCocktailRequest::Response& res);

 private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_main_;
  ros::ServiceClient client_move_arm_;
  ros::ServiceClient client_gripper_open_;
  ros::ServiceClient client_gripper_close_;
};

#endif /* BARTENDING_SERVER */
