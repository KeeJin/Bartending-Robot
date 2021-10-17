#ifndef BARTENDING_SERVER
#define BARTENDING_SERVER
#include "ros/ros.h"
#include "bartending_server/BartenderCocktailRequest.h"
#include <unordered_map>


class BartendingServer {
  enum Section {
    MIN = -90, 
    MAX = 90, 
    MIDDLE = 0, 
    ALCOHOL1 = 30,
    ALCOHOL2 = 60,
    MIXER1 = -80, 
    MIXER2 = -60, 
    MIXER3 = -40, 
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
  void GenerateIKSolution(std::string object, Position position, double offset = 0.0);
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
  bool Grasp(int motor_position);
  bool OpenGripper();
  bool MoveTo(std::string goal, bool keep_level = false);
  bool Pour(int microseconds);


 private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_main_;
  ros::ServiceClient client_move_arm_;
  ros::ServiceClient client_gripper_open_;
  ros::ServiceClient client_gripper_close_;
  ros::ServiceClient client_motor_control_;
  CustomerRequest current_request_;
  std::unordered_map<std::string, std::vector<double>> joint_states_;
  const Position shaker_position_ = {Section::MIDDLE, 0.03, 0.08};
  const Position pouring_position_ = {Section::ALCOHOL1, 0.03, 0.06};
  const Position shaker_cap_position_ = {Section::MIDDLE, 0.02, 0.08};
  const Position jager_position_ = {Section::ALCOHOL1, 0.06, 0.02};
  const Position vodka_position_ = {Section::ALCOHOL2, 0.03, 0.08};
  const Position redbull_position_ = {Section::MIXER1, 0.03, 0.08};
  const Position sprite_position_ = {Section::MIXER2, 0.03, 0.08};
  const Position water_position_ = {Section::MIXER3, 0.06, 0.02};
  const float offset_ = 0.01;
};

#endif /* BARTENDING_SERVER */
