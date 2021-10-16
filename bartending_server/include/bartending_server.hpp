#ifndef BARTENDING_SERVER
#define BARTENDING_SERVER
#include "ros/ros.h"
#include "bartending_server/BartenderCocktailRequest.h"
#include <unordered_map>


class BartendingServer {
  enum class Alcohol { Jager, Vodka };
  enum class Mixer { Water, RedBull, Sprite };
  struct CustomerRequest {
    Alcohol alcohol;
    Mixer mixer;
  };
  struct Position {
    float x, y, z;
  };

 public:
  BartendingServer() { Initialise(); }
  ~BartendingServer() = default;

 private:
  void Initialise();
  void GenerateIKSolution(std::string object, Position position);
  bool HandleCustomerRequest(
      bartending_server::BartenderCocktailRequest::Request& req,
      bartending_server::BartenderCocktailRequest::Response& res);
  bool InterpretRequest(int8_t alcohol, int8_t mixer);
  bool PrepareCocktail();

  // Microtasks
  bool PourAlcohol();
  bool PourMixer();
  bool CoverShaker();
  bool Shake();
  bool ServeCocktail();

 private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_main_;
  ros::ServiceClient client_move_arm_;
  ros::ServiceClient client_gripper_open_;
  ros::ServiceClient client_gripper_close_;
  CustomerRequest current_request_;
  std::unordered_map<std::string, std::vector<float>> joint_states_;
  const Position shaker_position_ = {0.1, 0.0, 0.04};
  const Position shaker_cap_position_ = {0.1, 0.3, 0.02};
  const Position jager_position_= {0.13, 0.05, 0.04};
  const Position vodka_position_= {0.11, 0.08, 0.04};
  const Position redbull_position_ = {0.13, -0.05, 0.04};
  const Position sprite_position_ = {0.11, -0.08, 0.04};
  const Position water_position_ = {0.09, -0.11, 0.04};

};

#endif /* BARTENDING_SERVER */
