#ifndef BARTENDING_SERVER
#define BARTENDING_SERVER
#include "ros/ros.h"
#include "bartending_server/BartenderCocktailRequest.h"
#include <unordered_map>

class BartendingServer {
  enum Section {
    MIN = -90,
    MAX = 90,
    SHAKER = -10,
    CAP = 30,
    POUR = -30,
    ALCOHOL1 = -80,
    // ALCOHOL2 = 80,
    MIXER1 = -45,
    // MIXER2 = -60,
    // MIXER3 = -40,
    SERVE = 65
  };
  enum class Alcohol { Jager, Vodka };
  enum class Mixer { Water, RedBull, Sprite };
  struct CustomerRequest {
    Alcohol alcohol;
    Mixer mixer;
  };
  struct Position {
    Section section;
    float distance;
    float height;
  };

 public:
  BartendingServer() { Initialise(); }
  ~BartendingServer() = default;

 private:
  void Initialise();
  void GenerateIKSolution(std::string object, Position position,
                          double offset = 0.0);
  bool HandleCustomerRequest(
      bartending_server::BartenderCocktailRequest::Request& req,
      bartending_server::BartenderCocktailRequest::Response& res);
  bool InterpretRequest(int8_t alcohol, int8_t mixer);
  bool PrepareCocktail();

  // Microtasks
  bool GoHome();
  bool PourAlcohol();
  bool PourMixer();
  bool CoverShaker();
  bool Shake();
  bool ServeCocktail();
  bool Grasp(int motor_position);
  bool OpenGripper();
  bool MoveTo(std::string goal, int delay, bool keep_level = false);
  bool Pour(int microseconds);

 private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_main_;
  ros::ServiceClient client_move_arm_;
  ros::ServiceClient client_gripper_open_;
  ros::ServiceClient client_gripper_close_;
  ros::ServiceClient client_motor_control_;
  CustomerRequest current_request_;
  const int shaker_close_ = 436;
  const int shake_cap_close_ = 410;
  const int alcohol_close_ = 232;
  const int mixer_close_ = 232;
  std::unordered_map<std::string, std::vector<double>> joint_states_;
  const Position shaker_position_ = {Section::SHAKER, -0.01, 0.085};
  const Position shaker_cap_position_off_ = {Section::CAP, -0.01, 0.104};
  const Position shaker_cap_position_put_on_ = {Section::SHAKER, -0.01, 0.128};
  const Position shaker_cap_position_take_off_ = {Section::SHAKER, -0.01, 0.098};

  const Position pouring_position_ = {Section::POUR, -0.01, 0.1};
  const Position shaking_position_ = {Section::SHAKER, -0.03, 0.1};
  const Position serving_position_ = {Section::SERVE, 0.03, 0.094};

  const Position jager_position_ = {Section::ALCOHOL1, 0.03, 0.098};
  // const Position vodka_position_ = {Section::ALCOHOL1, 0.11, 0.02};

  const Position redbull_position_ = {Section::MIXER1, 0.053, 0.098};
  // const Position sprite_position_ = {Section::MIXER2, 0.11, 0.02};
  // const Position water_position_ = {Section::MIXER3, 0.11, 0.02};

  const float offset_ = 0.03;
};

#endif /* BARTENDING_SERVER */
