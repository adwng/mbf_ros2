/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_SDK_HPP
#define RL_SDK_HPP

#include <tbb/concurrent_queue.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "../fsm/fsm.hpp"
#include "../inference_runtime/inference_runtime.hpp"
#include "../logger/logger.hpp"
#include "../observation_buffer/observation_buffer.hpp"
#include "../vector_math/vector_math.hpp"

template <typename T>
struct RobotCommand {
  struct MotorCommand {
    std::vector<int> mode;
    std::vector<T> q;
    std::vector<T> dq;
    std::vector<T> tau;
    std::vector<T> kp;
    std::vector<T> kd;

    void resize(size_t num_joints) {
      mode.resize(num_joints, 0);
      q.resize(num_joints, 0.0f);
      dq.resize(num_joints, 0.0f);
      tau.resize(num_joints, 0.0f);
      kp.resize(num_joints, 0.0f);
      kd.resize(num_joints, 0.0f);
    }
  } motor_command;
};

template <typename T>
struct RobotState {
  struct IMU {
    std::vector<T> quaternion = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
    std::vector<T> gyroscope = {0.0f, 0.0f, 0.0f};
    std::vector<T> accelerometer = {0.0f, 0.0f, 0.0f};
  } imu;

  struct MotorState {
    std::vector<T> q;
    std::vector<T> dq;
    std::vector<T> ddq;
    std::vector<T> tau_est;
    std::vector<T> cur;

    void resize(size_t num_joints) {
      q.resize(num_joints, 0.0f);
      dq.resize(num_joints, 0.0f);
      ddq.resize(num_joints, 0.0f);
      tau_est.resize(num_joints, 0.0f);
      cur.resize(num_joints, 0.0f);
    }
  } motor_state;
};

namespace Input {
// Recommend: Num0-GetUp Num9-GetDown N-ToggleNavMode
//            R-SimReset Enter-SimToggle
//            M-MotorEnable K-MotorDisable P-MotorPassive
//            Num1-BaseLocomotion Num2-Num8-Skills(7)
//            WS-AxisX AD-AxisY QE-AxisYaw Space-AxisClear
enum class Keyboard {
  None = 0,
  A,
  B,
  C,
  D,
  E,
  F,
  G,
  H,
  I,
  J,
  K,
  L,
  M,
  N,
  O,
  P,
  Q,
  R,
  S,
  T,
  U,
  V,
  W,
  X,
  Y,
  Z,
  Num0,
  Num1,
  Num2,
  Num3,
  Num4,
  Num5,
  Num6,
  Num7,
  Num8,
  Num9,
  Space,
  Enter,
  Escape,
  Up,
  Down,
  Left,
  Right
};

// Recommend: A-GetUp B-GetDown X-ToggleNavMode Y-None
//            RB_Y-SimReset RB_X-SimToggle
//            LB_A-MotorEnable LB_B-MotorDisable LB_X-MotorPassive
//            RB_DPadUp-BaseLocomotion RB_DPadOthers/LB_DPadOthers-Skills(7)
//            LY-AxisX LX-AxisY RX-AxisYaw
enum class Gamepad {
  None = 0,
  A,
  B,
  X,
  Y,
  LB,
  RB,
  LStick,
  RStick,
  DPadUp,
  DPadDown,
  DPadLeft,
  DPadRight,
  LB_A,
  LB_B,
  LB_X,
  LB_Y,
  LB_LStick,
  LB_RStick,
  LB_DPadUp,
  LB_DPadDown,
  LB_DPadLeft,
  LB_DPadRight,
  RB_A,
  RB_B,
  RB_X,
  RB_Y,
  RB_LStick,
  RB_RStick,
  RB_DPadUp,
  RB_DPadDown,
  RB_DPadLeft,
  RB_DPadRight,
  LB_RB
};
inline Gamepad GamepadFromString(const std::string &name) {
  static const std::map<std::string, Gamepad> table = {
      {"None", Gamepad::None},
      {"A", Gamepad::A},
      {"B", Gamepad::B},
      {"X", Gamepad::X},
      {"Y", Gamepad::Y},
      {"LB", Gamepad::LB},
      {"RB", Gamepad::RB},
      {"LStick", Gamepad::LStick},
      {"RStick", Gamepad::RStick},
      {"DPadUp", Gamepad::DPadUp},
      {"DPadDown", Gamepad::DPadDown},
      {"DPadLeft", Gamepad::DPadLeft},
      {"DPadRight", Gamepad::DPadRight},
      {"LB_A", Gamepad::LB_A},
      {"LB_B", Gamepad::LB_B},
      {"LB_X", Gamepad::LB_X},
      {"LB_Y", Gamepad::LB_Y},
      {"LB_LStick", Gamepad::LB_LStick},
      {"LB_RStick", Gamepad::LB_RStick},
      {"LB_DPadUp", Gamepad::LB_DPadUp},
      {"LB_DPadDown", Gamepad::LB_DPadDown},
      {"LB_DPadLeft", Gamepad::LB_DPadLeft},
      {"LB_DPadRight", Gamepad::LB_DPadRight},
      {"RB_A", Gamepad::RB_A},
      {"RB_B", Gamepad::RB_B},
      {"RB_X", Gamepad::RB_X},
      {"RB_Y", Gamepad::RB_Y},
      {"RB_LStick", Gamepad::RB_LStick},
      {"RB_RStick", Gamepad::RB_RStick},
      {"RB_DPadUp", Gamepad::RB_DPadUp},
      {"RB_DPadDown", Gamepad::RB_DPadDown},
      {"RB_DPadLeft", Gamepad::RB_DPadLeft},
      {"RB_DPadRight", Gamepad::RB_DPadRight},
      {"LB_RB", Gamepad::LB_RB},
  };
  auto it = table.find(name);
  return (it != table.end()) ? it->second : Gamepad::None;
}
}  // namespace Input

struct Control {
  Input::Keyboard current_keyboard = Input::Keyboard::None,
                  last_keyboard = Input::Keyboard::None;
  Input::Gamepad current_gamepad = Input::Gamepad::None,
                 last_gamepad = Input::Gamepad::None;

  float x = 0.0f;
  float y = 0.0f;
  float yaw = 0.0f;
  bool navigation_mode = false;

  void SetKeyboard(Input::Keyboard keyboad) {
    if (current_keyboard != keyboad) {
      last_keyboard = current_keyboard;
      current_keyboard = keyboad;
    }
  }

  void SetGamepad(Input::Gamepad gamepad) {
    if (current_gamepad != gamepad) {
      last_gamepad = current_gamepad;
      current_gamepad = gamepad;
    }
  }

  void ClearInput() {
    current_keyboard = last_keyboard;
    current_gamepad = Input::Gamepad::None;
  }
};

struct YamlParams {
  YAML::Node config_node;

  // Get config value by key
  // WARNING: For vectors/containers, store result in a variable before using
  // iterators/references:
  //   ✓ auto vec = params.Get<std::vector<int>>("key"); vec.begin()
  //   ✗ params.Get<std::vector<int>>("key").begin()  // dangling reference!
  template <typename T>
  T Get(const std::string &key, const T &default_value = T()) const {
    if (config_node[key]) {
      return config_node[key].as<T>();
    }
    return default_value;
  }

  bool Has(const std::string &key) const {
    return config_node[key].IsDefined();
  }
};

template <typename T>
struct Observations {
  std::vector<T> lin_vel;
  std::vector<T> ang_vel;
  std::vector<T> gravity_vec;
  std::vector<T> commands;
  std::vector<T> base_quat;
  std::vector<T> dof_pos;
  std::vector<T> dof_vel;
  std::vector<T> actions;
  // WTW (Walk These Ways) observation components
  std::vector<T> clock_sin;
  std::vector<T> clock_cos;
  std::vector<T> gait_period_obs;
  std::vector<T> base_height_obs;
  std::vector<T> foot_clearance_obs;
  std::vector<T> pitch_obs;
  std::vector<T> gait_theta;
  // UniLab per-foot gait phase observation (FL, FR, RL, RR), values in [0, 1).
  std::vector<T> feet_phase;
  std::vector<T> gait_phase_obs;
};

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

class RL {
 public:
  RL(){};
  ~RL(){};

  YamlParams params;
  Observations<float> obs;
  std::vector<int> obs_dims;

  RobotState<float> robot_state;
  RobotCommand<float> robot_command;
  tbb::concurrent_queue<std::vector<float>> output_dof_pos_queue;
  tbb::concurrent_queue<std::vector<float>> output_dof_vel_queue;
  tbb::concurrent_queue<std::vector<float>> output_dof_tau_queue;

  FSM fsm;
  RobotState<float> start_state;
  RobotState<float> now_state;
  bool rl_init_done = false;

  // init
  void InitObservations();
  void InitOutputs();
  void InitControl();
  void InitRL(std::string robot_config_path);
  void InitJointNum(size_t num_joints);

  // rl functions
  virtual std::vector<float> Forward() = 0;
  std::vector<float> ComputeObservation();
  virtual void GetState(RobotState<float> *state) = 0;
  virtual void SetCommand(const RobotCommand<float> *command) = 0;
  void StateController(const RobotState<float> *state,
                       RobotCommand<float> *command);
  void ComputeOutput(const std::vector<float> &actions,
                     std::vector<float> &output_dof_pos,
                     std::vector<float> &output_dof_vel,
                     std::vector<float> &output_dof_tau);
  void ComputeGaitPhase();
  // yaml params
  void ReadYaml(const std::string &file_path, const std::string &file_name);

  // control
  Control control;
  void KeyboardInterface();

  // configurable keyboard mapping (used by WTW controller)
  std::map<std::string, Input::Keyboard> key_mapping;
  float vel_step = 0.1f;
  void LoadKeyMapping();
  bool IsActionActive(const std::string &action) const;
  static Input::Keyboard KeyFromString(const std::string &s);

  // WTW gait / style
  WTWState wtw_state;
  void InitWTWState();
  void PreProcessGait();
  void ProcessWTWControls();

  // history buffer
  ObservationBuffer history_obs_buf;
  std::vector<float> history_obs;

  // others
  float global_phase = 0.0f;
  int motiontime = 0;
  std::string robot_name, config_name;
  bool simulation_running = true;
  std::string ang_vel_axis = "body";  // "world" or "body"

  // UniLab gait clock (drives the "feet_phase" observation). Advances once per
  // policy step inside ComputeObservation when "feet_phase" is requested.
  float unilab_gait_phase = 0.0f;
  unsigned long long episode_length_buf = 0;
  float motion_length = 0.0;
  int InverseJointMapping(int idx) const;

  // protect func
  void TorqueProtect(const std::vector<float> &origin_output_dof_tau);
  void AttitudeProtect(const std::vector<float> &quaternion,
                       float pitch_threshold, float roll_threshold);

  // rl module
  std::unique_ptr<InferenceRuntime::Model> model;
  // output buffer
  std::vector<float> output_dof_tau;
  std::vector<float> output_dof_pos;
  std::vector<float> output_dof_vel;

  // thread safety
  std::mutex model_mutex;
};

class RLFSMState : public FSMState {
 public:
  RLFSMState(RL &rl, const std::string &name)
      : FSMState(name), rl(rl), fsm_state(nullptr), fsm_command(nullptr) {}

  RL &rl;
  const RobotState<float> *fsm_state;
  RobotCommand<float> *fsm_command;

  bool Interpolate(float &percent, const std::vector<float> &start_pos,
                   const std::vector<float> &target_pos, float duration_seconds,
                   const std::string &description = "",
                   bool use_fixed_gains = true);

  void RLControl();
};

#endif  // RL_SDK_HPP
