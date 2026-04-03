#ifndef MBF_RL_WTW_HPP
#define MBF_RL_WTW_HPP

#include <cmath>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "../library/inference_runtime/inference_runtime.hpp"
#include "../library/loop/loop.hpp"
#include "../library/observation_buffer/observation_buffer.hpp"
#include "../library/rl_sdk/rl_sdk.hpp"
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"

struct WTWState {
  float gait_time = 0.0f;
  float phi = 0.0f;
  int gait_choice = 0;
  float gait_period = 0.5f;
  float base_height = 0.25f;
  float foot_clearance = 0.08f;
  float pitch = 0.0f;
  int gait_switch_cooldown_counter = 0;
  int gait_switch_cooldown_max = 100;
  float adjust_step = 0.01f;
};

class MBF_RL_WTW : public RL {
 public:
  MBF_RL_WTW(int argc, char **argv);
  ~MBF_RL_WTW();

  std::shared_ptr<rclcpp::Node> node;

  // Keyboard action query — checks current_keyboard against key_mapping
  bool IsActionActive(const std::string &action) const;

  WTWState wtw_state;

  void InitWTWState();
  void PreProcessGait();
  void ProcessWTWControls();

 private:
  std::vector<float> Forward() override;
  void GetState(RobotState<float> *state) override;
  void SetCommand(const RobotCommand<float> *command) override;
  void RunModel();
  void RobotControl();

  void LoadKeyMapping();
  static Input::Keyboard KeyFromString(const std::string &s);

  // Configurable keyboard mapping: action name → Keyboard enum
  std::map<std::string, Input::Keyboard> key_mapping_;
  float vel_step_ = 0.1f;

  std::shared_ptr<LoopFunc> loop_keyboard;
  std::shared_ptr<LoopFunc> loop_control;
  std::shared_ptr<LoopFunc> loop_rl;

  sensor_msgs::msg::Imu imu_;
  geometry_msgs::msg::Twist cmd_vel_;
  robot_msgs::msg::RobotCommand robot_command_pub_msg_;
  robot_msgs::msg::RobotState robot_state_sub_msg_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr
      robot_state_sub_;
  rclcpp::Publisher<robot_msgs::msg::RobotCommand>::SharedPtr
      robot_command_pub_;
};

#endif  // MBF_RL_WTW_HPP
