#ifndef MBF_RL_WTW_HPP
#define MBF_RL_WTW_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "../library/loop/loop.hpp"
#include "../library/rl_sdk/rl_sdk.hpp"
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"

class MBF_RL_WTW : public RL {
 public:
  MBF_RL_WTW(int argc, char **argv);
  ~MBF_RL_WTW();

  std::shared_ptr<rclcpp::Node> node;

 private:
  std::vector<float> Forward() override;
  void GetState(RobotState<float> *state) override;
  void SetCommand(const RobotCommand<float> *command) override;
  void RunModel();
  void RobotControl();

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
