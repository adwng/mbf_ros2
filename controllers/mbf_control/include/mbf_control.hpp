#ifndef MBF_CONTROL_HPP
#define MBF_CONTROL_HPP

#include <champ/body_controller/body_controller.h>
#include <champ/kinematics/kinematics.h>
#include <champ/leg_controller/leg_controller.h>
#include <champ/utils/urdf_loader.h>

#include "TUI.hpp"
#include "mbf_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#define TUI_INFO(msg) TUI::Dashboard::log(msg, TUI::Dashboard::LogLevel::INFO)
#define TUI_WARN(msg) TUI::Dashboard::log(msg, TUI::Dashboard::LogLevel::WARN)
#define TUI_ERR(msg) TUI::Dashboard::log(msg, TUI::Dashboard::LogLevel::ERROR)

static constexpr int NUM_JOINTS = 12;
static constexpr int NUM_LEGS = 4;
static constexpr int JOINTS = 3; // JOINTS PER LEG

enum class FSMState { PASSIVE, STANDUP, LOCOMOTION, GETDOWN };

struct JointState {
  std::array<double, NUM_JOINTS> q{};       // position (rad)
  std::array<double, NUM_JOINTS> dq{};      // velocity (rad/s)
  std::array<double, NUM_JOINTS> tau_est{}; // estimated torque (Nm)
};

struct ImuData {
  tf2::Quaternion orientation_;
  double roll, pitch, yaw;
  double wx, wy, wz;
  double ax, ay, az;

  double roll_filt_ = 0.0;
  double pitch_filt_ = 0.0;
  double wx_filt_ = 0.0;
  double wy_filt_ = 0.0;

  bool imu_initialized_ = false;
};

struct JointCommand {
  std::array<double, NUM_JOINTS> q{};   // desired position
  std::array<double, NUM_JOINTS> dq{};  // desired velocity
  std::array<double, NUM_JOINTS> kp{};  // proportional gain
  std::array<double, NUM_JOINTS> kd{};  // derivative gain
  std::array<double, NUM_JOINTS> tau{}; // feedforward torque
};

struct Poses {
  std::array<double, NUM_JOINTS> passive;
  std::array<double, NUM_JOINTS> standing;
};

struct GamePad_Data_t {
  bool passive = false;
  bool standup = false;
  bool locomotion = false;
  bool getdown = false;
  double x, y, z;
  double lin_x, lin_y, ang_z;
  double roll, pitch, yaw;
};

class MBFControl : public rclcpp::Node {
  Parameter_t params;
  void loop();

  rclcpp::Clock clock_;

  // champ stuffs
  champ::Velocities req_vel_;
  champ::Pose req_pose_;

  champ::GaitConfig gait_config_;

  champ::QuadrupedBase base_;
  champ::BodyController body_controller_;
  champ::LegController leg_controller_;
  champ::Kinematics kinematics_;

  // subscribers and publishers' callbacks
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
  rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr robotStateSub_;
  rclcpp::Publisher<robot_msgs::msg::RobotCommand>::SharedPtr robotCmdPub;

  void robotStateCallback(robot_msgs::msg::RobotState::SharedPtr msg);
  void joyCallback(sensor_msgs::msg::Joy::SharedPtr msg);
  void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg);

  //  ros2 node stuff
  rclcpp::TimerBase::SharedPtr loop_timer_;

  TUI::Dashboard dash;
  int update_time = 0;
  FSMState fsm_state = FSMState::PASSIVE;
  std::string state_string_ = "PASSIVE";

  JointState jointStateData;
  JointCommand jointCommandData;
  GamePad_Data_t gamepadData;
  ImuData imuData;
  Poses poses_;

  // Transition interpolation
  std::array<double, NUM_JOINTS> interp_start_q_{};
  std::array<double, NUM_JOINTS> interp_target_q_{};
  double interp_duration_ = 0.0;
  double interp_elapsed_ = 0.0;
  bool in_transition_ = false;
  sensor_msgs::msg::Joy last_joy_msg_;
  bool first_joy_received_ = false;

  void publishCommands();

  // Methods
  void begin_transition(const std::array<double, NUM_JOINTS> &target,
                        double duration);
  void run_interpolation(double dt, int dir);
  double smooth_ratio(double t, double T);
  void set_passive_commands();
  void compute_locomotion();
  void compute_stab();
  void update_dashboard();

  double lowpass(double x, double y_prev, double alpha) {
    return alpha * x + (1.0 - alpha) * y_prev;
  }

  double rad2deg(double value) { return value * (180.0 / M_PI); }

public:
  MBFControl();
};

#endif