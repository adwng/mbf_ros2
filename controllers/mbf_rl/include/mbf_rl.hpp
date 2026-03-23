#ifndef MBF_RL_HPP
#define MBF_RL_HPP

#include <sys/wait.h>
#include <unistd.h>

#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "../library/inference_runtime/inference_runtime.hpp"
#include "../library/loop/loop.hpp"
#include "../library/observation_buffer/observation_buffer.hpp"
#include "../library/rl_sdk/rl_sdk.hpp"
#include "fsm_mbf.hpp"
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"

class MBF_RL : public RL {
 public:
  MBF_RL(int argc, char **argv);
  ~MBF_RL();

  std::shared_ptr<rclcpp::Node> node;

 private:
  // rl functions
  std::vector<float> Forward() override;
  void GetState(RobotState<float> *state) override;
  void SetCommand(const RobotCommand<float> *command) override;
  void RunModel();
  void RobotControl();

  // loop
  std::shared_ptr<LoopFunc> loop_keyboard;
  std::shared_ptr<LoopFunc> loop_control;
  std::shared_ptr<LoopFunc> loop_rl;

  sensor_msgs::msg::Imu imu;
  geometry_msgs::msg::Twist cmd_vel;
  sensor_msgs::msg::Joy joy_msg;
  robot_msgs::msg::RobotCommand robot_command_publisher_msg;
  robot_msgs::msg::RobotState robot_state_subscriber_msg;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
  rclcpp::Publisher<robot_msgs::msg::RobotCommand>::SharedPtr
      robot_command_publisher;
  rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr
      robot_state_subscriber;
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void RobotStateCallback(const robot_msgs::msg::RobotState::SharedPtr msg);
  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

  std::map<std::string, float> joint_positions;
  std::map<std::string, float> joint_velocities;
  std::map<std::string, float> joint_efforts;
};

#endif