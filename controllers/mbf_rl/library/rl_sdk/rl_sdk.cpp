/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sdk.hpp"

void RL::StateController(const RobotState<float>* state,
                         RobotCommand<float>* command) {
  auto updateState = [&](std::shared_ptr<FSMState> statePtr) {
    if (auto rl_fsm_state = std::dynamic_pointer_cast<RLFSMState>(statePtr)) {
      rl_fsm_state->fsm_state = state;
      rl_fsm_state->fsm_command = command;
    }
  };
  for (auto& pair : fsm.states_) {
    updateState(pair.second);
  }

  fsm.Run();

  this->motiontime++;

  if (this->control.current_keyboard == Input::Keyboard::W) {
    this->control.x += 0.1f;
  }
  if (this->control.current_keyboard == Input::Keyboard::S) {
    this->control.x -= 0.1f;
  }
  if (this->control.current_keyboard == Input::Keyboard::A) {
    this->control.y += 0.1f;
  }
  if (this->control.current_keyboard == Input::Keyboard::D) {
    this->control.y -= 0.1f;
  }
  if (this->control.current_keyboard == Input::Keyboard::Q) {
    this->control.yaw += 0.1f;
  }
  if (this->control.current_keyboard == Input::Keyboard::E) {
    this->control.yaw -= 0.1f;
  }
  if (this->control.current_keyboard == Input::Keyboard::Space) {
    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;
  }
  if (this->control.current_keyboard == Input::Keyboard::N ||
      this->control.current_gamepad == Input::Gamepad::X) {
    this->control.navigation_mode = !this->control.navigation_mode;
    std::cout << std::endl
              << LOGGER::INFO << "Navigation mode: "
              << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
  }
}

std::vector<float> RL::ComputeObservation() {
  std::vector<std::vector<float>> obs_list;

  for (const std::string& observation :
       this->params.Get<std::vector<std::string>>("observations")) {
    // ============= Base Observations =============
    if (observation == "lin_vel") {
      obs_list.push_back(this->obs.lin_vel *
                         this->params.Get<float>("lin_vel_scale"));
    } else if (observation == "ang_vel") {
      // In ROS1 Gazebo, the coordinate system for angular velocity is in the
      // world coordinate system. In ROS2 Gazebo, mujoco and real robot, the
      // coordinate system for angular velocity is in the body coordinate
      // system.
      if (this->ang_vel_axis == "body") {
        obs_list.push_back(this->obs.ang_vel *
                           this->params.Get<float>("ang_vel_scale"));
      } else if (this->ang_vel_axis == "world") {
        obs_list.push_back(
            QuatRotateInverse(this->obs.base_quat, this->obs.ang_vel) *
            this->params.Get<float>("ang_vel_scale"));
      }
    } else if (observation == "gravity_vec") {
      obs_list.push_back(
          QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec));
    } else if (observation == "commands") {
      obs_list.push_back(
          this->obs.commands *
          this->params.Get<std::vector<float>>("commands_scale"));
    } else if (observation == "dof_pos") {
      std::vector<float> dof_pos_rel =
          this->obs.dof_pos -
          this->params.Get<std::vector<float>>("default_dof_pos");
      for (int i : this->params.Get<std::vector<int>>("wheel_indices")) {
        dof_pos_rel[i] = 0.0f;
      }
      obs_list.push_back(dof_pos_rel *
                         this->params.Get<float>("dof_pos_scale"));
    } else if (observation == "dof_vel") {
      obs_list.push_back(this->obs.dof_vel *
                         this->params.Get<float>("dof_vel_scale"));
    } else if (observation == "actions") {
      obs_list.push_back(this->obs.actions);
    }
    // ============= WTW Observations =============
    else if (observation == "clock_sin") {
      obs_list.push_back(this->obs.clock_sin);
    } else if (observation == "clock_cos") {
      obs_list.push_back(this->obs.clock_cos);
    } else if (observation == "gait_period") {
      obs_list.push_back(this->obs.gait_period_obs);
    } else if (observation == "base_height") {
      obs_list.push_back(this->obs.base_height_obs);
    } else if (observation == "foot_clearance") {
      obs_list.push_back(this->obs.foot_clearance_obs);
    } else if (observation == "pitch") {
      obs_list.push_back(this->obs.pitch_obs);
    } else if (observation == "gait_theta") {
      obs_list.push_back(this->obs.gait_theta);
    } else if (observation == "period") {
      obs_list.push_back(this->obs.gait_phase_obs);
    }
  }

  this->obs_dims.clear();
  for (const auto& obs : obs_list) {
    this->obs_dims.push_back(obs.size());
  }

  std::vector<float> obs;
  for (const auto& obs_vec : obs_list) {
    obs.insert(obs.end(), obs_vec.begin(), obs_vec.end());
  }
  std::vector<float> clamped_obs =
      clamp(obs, -this->params.Get<float>("clip_obs"),
            this->params.Get<float>("clip_obs"));
  return clamped_obs;
}

void RL::InitObservations() {
  this->obs.lin_vel = {0.0f, 0.0f, 0.0f};
  this->obs.ang_vel = {0.0f, 0.0f, 0.0f};
  this->obs.gravity_vec = {0.0f, 0.0f, -1.0f};
  this->obs.commands = {0.0f, 0.0f, 0.0f};
  this->obs.base_quat = {0.0f, 0.0f, 0.0f, 1.0f};
  this->obs.dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
  this->obs.dof_vel.clear();
  this->obs.dof_vel.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
  this->obs.actions.clear();
  this->obs.actions.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
  // WTW obs (safe defaults; only consumed if listed in observations YAML)
  this->obs.clock_sin = {0.0f, 0.0f, 0.0f, 0.0f};
  this->obs.clock_cos = {1.0f, 1.0f, 1.0f, 1.0f};
  this->obs.gait_period_obs = {0.5f};
  this->obs.base_height_obs = {0.2f};
  this->obs.foot_clearance_obs = {0.04f};
  this->obs.pitch_obs = {0.0f};
  this->obs.gait_theta = {0.0f, 0.5f, 0.5f, 0.0f};
  this->obs.gait_phase_obs = {0.0f, 0.0f};
  this->ComputeGaitPhase();
  this->ComputeObservation();
}

void RL::InitOutputs() {
  int num_of_dofs = this->params.Get<int>("num_of_dofs");
  this->output_dof_tau.clear();
  this->output_dof_tau.resize(num_of_dofs, 0.0f);
  this->output_dof_pos =
      this->params.Get<std::vector<float>>("default_dof_pos");
  this->output_dof_vel.clear();
  this->output_dof_vel.resize(num_of_dofs, 0.0f);
}

void RL::InitControl() {
  this->control.x = 0.0f;
  this->control.y = 0.0f;
  this->control.yaw = 0.0f;
}

void RL::InitJointNum(size_t num_joints) {
  this->robot_state.motor_state.resize(num_joints);
  this->start_state.motor_state.resize(num_joints);
  this->now_state.motor_state.resize(num_joints);
  this->robot_command.motor_command.resize(num_joints);
}

void RL::InitRL(std::string robot_config_path) {
  std::lock_guard<std::mutex> lock(this->model_mutex);

  this->ReadYaml(robot_config_path, "config.yaml");

  // init joint num first
  this->InitJointNum(this->params.Get<int>("num_of_dofs"));

  // init rl
  this->InitObservations();
  this->InitOutputs();
  this->InitControl();

  // init obs history
  const auto& observations_history = this->params.Get<std::vector<int>>(
      "observations_history");  // avoid dangling reference
  if (!observations_history.empty()) {
    int history_length = *std::max_element(observations_history.begin(),
                                           observations_history.end()) +
                         1;
    this->history_obs_buf = ObservationBuffer(
        1, this->obs_dims, history_length,
        this->params.Get<std::string>("observations_history_priority"));
  }

  // init model
  std::string model_path = std::string(POLICY_DIR) + "/" + robot_config_path +
                           "/" + this->params.Get<std::string>("model_name");
  this->model = InferenceRuntime::ModelFactory::load_model(model_path);
  if (!this->model) {
    throw std::runtime_error("Failed to load model from: " + model_path);
  }
}

void RL::ComputeOutput(const std::vector<float>& actions,
                       std::vector<float>& output_dof_pos,
                       std::vector<float>& output_dof_vel,
                       std::vector<float>& output_dof_tau) {
  std::vector<float> actions_scaled =
      actions * this->params.Get<std::vector<float>>("action_scale");
  std::vector<float> pos_actions_scaled = actions_scaled;
  std::vector<float> vel_actions_scaled(actions.size(), 0.0f);
  for (int i : this->params.Get<std::vector<int>>("wheel_indices")) {
    pos_actions_scaled[i] = 0.0f;
    vel_actions_scaled[i] = actions_scaled[i];
  }
  std::vector<float> all_actions_scaled =
      pos_actions_scaled + vel_actions_scaled;
  output_dof_pos = pos_actions_scaled +
                   this->params.Get<std::vector<float>>("default_dof_pos");
  output_dof_vel = vel_actions_scaled;
  output_dof_tau =
      this->params.Get<std::vector<float>>("rl_kp") *
          (all_actions_scaled +
           this->params.Get<std::vector<float>>("default_dof_pos") -
           this->obs.dof_pos) -
      this->params.Get<std::vector<float>>("rl_kd") * this->obs.dof_vel;
  output_dof_tau = clamp(output_dof_tau,
                         -this->params.Get<std::vector<float>>("torque_limits"),
                         this->params.Get<std::vector<float>>("torque_limits"));
}

void RL::ComputeGaitPhase() {
  float period   = this->params.Get<float>("period");
  float rl_dt    = this->params.Get<float>("dt") * this->params.Get<int>("decimation");
  float delta_phase = rl_dt * (1.0f / period);  

  this->global_phase += delta_phase;
  this->global_phase = std::fmod(this->global_phase, 1.0f);

  float angle = this->global_phase * 2.0f * static_cast<float>(M_PI);

  float cx   = this->obs.commands[0];
  float cy   = this->obs.commands[1];
  float cyaw = this->obs.commands[2];
  float cmd_norm = std::sqrt(cx*cx + cy*cy + cyaw*cyaw);

  if (cmd_norm < 0.1f) {
    this->obs.gait_phase_obs = {0.0f, 0.0f};
  } else {
    this->obs.gait_phase_obs = {std::sin(angle), std::cos(angle)};
  }
}

int RL::InverseJointMapping(int idx) const {
  auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
  for (size_t i = 0; i < joint_mapping.size(); ++i) {
    if (joint_mapping[i] == idx) return (int)i;
  }
  return -1;
}

void RL::TorqueProtect(const std::vector<float>& origin_output_dof_tau) {
  std::vector<int> out_of_range_indices;
  std::vector<float> out_of_range_values;
  for (size_t i = 0; i < origin_output_dof_tau.size(); ++i) {
    float torque_value = origin_output_dof_tau[i];
    float limit_lower =
        -this->params.Get<std::vector<float>>("torque_limits")[i];
    float limit_upper =
        this->params.Get<std::vector<float>>("torque_limits")[i];

    if (torque_value < limit_lower || torque_value > limit_upper) {
      out_of_range_indices.push_back(i);
      out_of_range_values.push_back(torque_value);
    }
  }
  if (!out_of_range_indices.empty()) {
    for (size_t i = 0; i < out_of_range_indices.size(); ++i) {
      int index = out_of_range_indices[i];
      float value = out_of_range_values[i];
      float limit_lower =
          -this->params.Get<std::vector<float>>("torque_limits")[index];
      float limit_upper =
          this->params.Get<std::vector<float>>("torque_limits")[index];

      std::cout << LOGGER::WARNING << "Torque(" << index + 1 << ")=" << value
                << " out of range(" << limit_lower << ", " << limit_upper << ")"
                << std::endl;
    }
    // Just a reminder, no protection
    // this->control.SetKeyboard(Input::Keyboard::P);
    std::cout << LOGGER::INFO << "Switching to STATE_POS_GETDOWN" << std::endl;
  }
}

void RL::AttitudeProtect(const std::vector<float>& quaternion,
                         float pitch_threshold, float roll_threshold) {
  // Use QuaternionToEuler from vector_math.hpp
  std::vector<float> euler = QuaternionToEuler(quaternion);
  float roll = euler[0] * 57.2958f;  // Convert to degrees
  float pitch = euler[1] * 57.2958f;

  if (std::fabs(roll) > roll_threshold) {
    this->control.SetKeyboard(Input::Keyboard::P);
    std::cout << LOGGER::WARNING << "Roll exceeds " << roll_threshold
              << " degrees. Current: " << roll << " degrees." << std::endl;
  }
  if (std::fabs(pitch) > pitch_threshold) {
    this->control.SetKeyboard(Input::Keyboard::P);
    std::cout << LOGGER::WARNING << "Pitch exceeds " << pitch_threshold
              << " degrees. Current: " << pitch << " degrees." << std::endl;
  }
}

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

static int kbhit() {
  static bool initialized = false;
  static termios original_term;

  // Initialize terminal to non-canonical mode on first call
  if (!initialized) {
    tcgetattr(STDIN_FILENO, &original_term);

    termios new_term = original_term;
    new_term.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echo
    new_term.c_cc[VMIN] = 0;               // Non-blocking read
    new_term.c_cc[VTIME] = 0;              // No timeout

    tcsetattr(STDIN_FILENO, TCSANOW, &new_term);

    // Register cleanup function to restore terminal on exit
    static bool cleanup_registered = false;
    if (!cleanup_registered) {
      std::atexit([]() { tcsetattr(STDIN_FILENO, TCSANOW, &original_term); });
      cleanup_registered = true;
    }

    initialized = true;
  }

  // Non-blocking read of a single character
  char c;
  int result = read(STDIN_FILENO, &c, 1);

  return (result == 1) ? (unsigned char)c : -1;
}

void RL::KeyboardInterface() {
  int c = kbhit();
  if (c > 0) {
    switch (c) {
      case '0':
        this->control.SetKeyboard(Input::Keyboard::Num0);
        break;
      case '1':
        this->control.SetKeyboard(Input::Keyboard::Num1);
        break;
      case '2':
        this->control.SetKeyboard(Input::Keyboard::Num2);
        break;
      case '3':
        this->control.SetKeyboard(Input::Keyboard::Num3);
        break;
      case '4':
        this->control.SetKeyboard(Input::Keyboard::Num4);
        break;
      case '5':
        this->control.SetKeyboard(Input::Keyboard::Num5);
        break;
      case '6':
        this->control.SetKeyboard(Input::Keyboard::Num6);
        break;
      case '7':
        this->control.SetKeyboard(Input::Keyboard::Num7);
        break;
      case '8':
        this->control.SetKeyboard(Input::Keyboard::Num8);
        break;
      case '9':
        this->control.SetKeyboard(Input::Keyboard::Num9);
        break;
      case 'a':
      case 'A':
        this->control.SetKeyboard(Input::Keyboard::A);
        break;
      case 'b':
      case 'B':
        this->control.SetKeyboard(Input::Keyboard::B);
        break;
      case 'c':
      case 'C':
        this->control.SetKeyboard(Input::Keyboard::C);
        break;
      case 'd':
      case 'D':
        this->control.SetKeyboard(Input::Keyboard::D);
        break;
      case 'e':
      case 'E':
        this->control.SetKeyboard(Input::Keyboard::E);
        break;
      case 'f':
      case 'F':
        this->control.SetKeyboard(Input::Keyboard::F);
        break;
      case 'g':
      case 'G':
        this->control.SetKeyboard(Input::Keyboard::G);
        break;
      case 'h':
      case 'H':
        this->control.SetKeyboard(Input::Keyboard::H);
        break;
      case 'i':
      case 'I':
        this->control.SetKeyboard(Input::Keyboard::I);
        break;
      case 'j':
      case 'J':
        this->control.SetKeyboard(Input::Keyboard::J);
        break;
      case 'k':
      case 'K':
        this->control.SetKeyboard(Input::Keyboard::K);
        break;
      case 'l':
      case 'L':
        this->control.SetKeyboard(Input::Keyboard::L);
        break;
      case 'm':
      case 'M':
        this->control.SetKeyboard(Input::Keyboard::M);
        break;
      case 'n':
      case 'N':
        this->control.SetKeyboard(Input::Keyboard::N);
        break;
      case 'o':
      case 'O':
        this->control.SetKeyboard(Input::Keyboard::O);
        break;
      case 'p':
      case 'P':
        this->control.SetKeyboard(Input::Keyboard::P);
        break;
      case 'q':
      case 'Q':
        this->control.SetKeyboard(Input::Keyboard::Q);
        break;
      case 'r':
      case 'R':
        this->control.SetKeyboard(Input::Keyboard::R);
        break;
      case 's':
      case 'S':
        this->control.SetKeyboard(Input::Keyboard::S);
        break;
      case 't':
      case 'T':
        this->control.SetKeyboard(Input::Keyboard::T);
        break;
      case 'u':
      case 'U':
        this->control.SetKeyboard(Input::Keyboard::U);
        break;
      case 'v':
      case 'V':
        this->control.SetKeyboard(Input::Keyboard::V);
        break;
      case 'w':
      case 'W':
        this->control.SetKeyboard(Input::Keyboard::W);
        break;
      case 'x':
      case 'X':
        this->control.SetKeyboard(Input::Keyboard::X);
        break;
      case 'y':
      case 'Y':
        this->control.SetKeyboard(Input::Keyboard::Y);
        break;
      case 'z':
      case 'Z':
        this->control.SetKeyboard(Input::Keyboard::Z);
        break;
      case ' ':
        this->control.SetKeyboard(Input::Keyboard::Space);
        break;
      case '\n':
      case '\r':
        this->control.SetKeyboard(Input::Keyboard::Enter);
        break;
      case 27:  // Escape sequence (for arrow keys on Unix/Linux/macOS)
      {
        char seq[2];
        // Try to read escape sequence non-blockingly
        if (read(STDIN_FILENO, &seq[0], 1) == 1) {
          if (seq[0] == '[') {
            if (read(STDIN_FILENO, &seq[1], 1) == 1) {
              switch (seq[1]) {
                case 'A':
                  this->control.SetKeyboard(Input::Keyboard::Up);
                  break;
                case 'B':
                  this->control.SetKeyboard(Input::Keyboard::Down);
                  break;
                case 'C':
                  this->control.SetKeyboard(Input::Keyboard::Right);
                  break;
                case 'D':
                  this->control.SetKeyboard(Input::Keyboard::Left);
                  break;
                default:
                  break;
              }
            }
          } else {
            // Plain escape key
            this->control.SetKeyboard(Input::Keyboard::Escape);
          }
        } else {
          // Plain escape key
          this->control.SetKeyboard(Input::Keyboard::Escape);
        }
      } break;
      default:
        break;
    }
  }
}

// =========================================================================
// Configurable keyboard mapping
// =========================================================================

Input::Keyboard RL::KeyFromString(const std::string& s) {
  if (s.size() == 1) {
    char c = static_cast<char>(std::tolower(static_cast<unsigned char>(s[0])));
    if (c >= 'a' && c <= 'z')
      return static_cast<Input::Keyboard>(
          static_cast<int>(Input::Keyboard::A) + (c - 'a'));
    if (c >= '0' && c <= '9')
      return static_cast<Input::Keyboard>(
          static_cast<int>(Input::Keyboard::Num0) + (c - '0'));
  }
  if (s == "space") return Input::Keyboard::Space;
  if (s == "enter") return Input::Keyboard::Enter;
  if (s == "escape") return Input::Keyboard::Escape;
  if (s == "up") return Input::Keyboard::Up;
  if (s == "down") return Input::Keyboard::Down;
  if (s == "left") return Input::Keyboard::Left;
  if (s == "right") return Input::Keyboard::Right;
  return Input::Keyboard::None;
}

void RL::LoadKeyMapping() {
  YAML::Node node = this->params.config_node["key_mapping"];
  if (!node || !node.IsMap()) {
    std::cout << LOGGER::WARNING
              << "No key_mapping in config, using defaults" << std::endl;
    key_mapping["forward"] = Input::Keyboard::W;
    key_mapping["backward"] = Input::Keyboard::S;
    key_mapping["left"] = Input::Keyboard::A;
    key_mapping["right"] = Input::Keyboard::D;
    key_mapping["yaw_left"] = Input::Keyboard::Q;
    key_mapping["yaw_right"] = Input::Keyboard::E;
    key_mapping["stop"] = Input::Keyboard::Space;
    key_mapping["getup"] = Input::Keyboard::Num0;
    key_mapping["getdown"] = Input::Keyboard::Num9;
    key_mapping["passive"] = Input::Keyboard::P;
    key_mapping["locomotion"] = Input::Keyboard::Num1;
    key_mapping["nav_mode"] = Input::Keyboard::N;
    return;
  }

  for (auto it = node.begin(); it != node.end(); ++it) {
    std::string action = it->first.as<std::string>();
    std::string key_str = it->second.as<std::string>();
    Input::Keyboard kb = KeyFromString(key_str);
    if (kb != Input::Keyboard::None) {
      key_mapping[action] = kb;
    } else {
      std::cout << LOGGER::WARNING << "Unknown key '" << key_str
                << "' for action '" << action << "'" << std::endl;
    }
  }

  vel_step = this->params.Get<float>("vel_step", 0.1f);

  std::cout << LOGGER::INFO << "Key mapping loaded — "
            << key_mapping.size() << " actions" << std::endl;
}

bool RL::IsActionActive(const std::string& action) const {
  auto it = key_mapping.find(action);
  if (it == key_mapping.end()) return false;
  return this->control.current_keyboard == it->second;
}

// =========================================================================
// WTW gait / style logic
// =========================================================================

void RL::InitWTWState() {
  auto gp = this->params.Get<std::vector<float>>("gait_period_range");
  auto bh = this->params.Get<std::vector<float>>("base_height_range");
  auto fc = this->params.Get<std::vector<float>>("foot_clearance_range");
  auto pr = this->params.Get<std::vector<float>>("pitch_range");

  wtw_state.gait_period = gp[1];
  wtw_state.base_height = bh[0];
  wtw_state.foot_clearance = fc[0];
  wtw_state.pitch = 0.0f;
  wtw_state.gait_time = 0.0f;
  wtw_state.phi = 0.0f;
  wtw_state.gait_choice = 0;
  wtw_state.adjust_step = this->params.Get<float>("wtw_adjust_step", 0.01f);
  wtw_state.gait_switch_cooldown_max =
      this->params.Get<int>("gait_switch_cooldown", 100);
  wtw_state.gait_switch_cooldown_counter = 0;

  std::cout << LOGGER::INFO << "[WTW] Initialized — gaits: "
            << this->params.Get<int>("num_gaits")
            << ", period: " << wtw_state.gait_period << std::endl;
}

void RL::PreProcessGait() {
  auto& ws = wtw_state;
  float rl_dt =
      this->params.Get<float>("dt") * this->params.Get<int>("decimation");

  ws.gait_time += rl_dt;
  if (ws.gait_time > (ws.gait_period - rl_dt / 2.0f)) ws.gait_time = 0.0f;
  ws.phi = ws.gait_time / ws.gait_period;

  auto tfl = this->params.Get<std::vector<float>>("theta_fl");
  auto tfr = this->params.Get<std::vector<float>>("theta_fr");
  auto trl = this->params.Get<std::vector<float>>("theta_rl");
  auto trr = this->params.Get<std::vector<float>>("theta_rr");

  std::vector<float> theta = {tfl[ws.gait_choice], tfr[ws.gait_choice],
                               trl[ws.gait_choice], trr[ws.gait_choice]};

  this->obs.clock_sin.resize(4);
  this->obs.clock_cos.resize(4);
  for (int i = 0; i < 4; ++i) {
    float phase = 2.0f * static_cast<float>(M_PI) * (ws.phi + theta[i]);
    this->obs.clock_sin[i] = std::sin(phase);
    this->obs.clock_cos[i] = std::cos(phase);
  }

  this->obs.gait_period_obs = {ws.gait_period};
  this->obs.base_height_obs = {ws.base_height};
  this->obs.foot_clearance_obs = {ws.foot_clearance};
  this->obs.pitch_obs = {ws.pitch};
  this->obs.gait_theta = theta;
}

void RL::ProcessWTWControls() {
  auto& ws = wtw_state;

  auto gp = this->params.Get<std::vector<float>>("gait_period_range");
  auto bh = this->params.Get<std::vector<float>>("base_height_range");
  auto fc = this->params.Get<std::vector<float>>("foot_clearance_range");
  auto pr = this->params.Get<std::vector<float>>("pitch_range");

  if (IsActionActive("gait_period_up"))
    ws.gait_period = std::min(ws.gait_period + ws.adjust_step, gp[1]);
  else if (IsActionActive("gait_period_down"))
    ws.gait_period = std::max(ws.gait_period - ws.adjust_step, gp[0]);

  if (IsActionActive("base_height_up"))
    ws.base_height = std::min(ws.base_height + ws.adjust_step, bh[1]);
  else if (IsActionActive("base_height_down"))
    ws.base_height = std::max(ws.base_height - ws.adjust_step, bh[0]);

  if (IsActionActive("foot_clearance_up"))
    ws.foot_clearance = std::min(ws.foot_clearance + ws.adjust_step, fc[1]);
  else if (IsActionActive("foot_clearance_down"))
    ws.foot_clearance = std::max(ws.foot_clearance - ws.adjust_step, fc[0]);

  if (IsActionActive("pitch_up"))
    ws.pitch = std::min(ws.pitch + ws.adjust_step, pr[1]);
  else if (IsActionActive("pitch_down"))
    ws.pitch = std::max(ws.pitch - ws.adjust_step, pr[0]);

  if (ws.gait_switch_cooldown_counter > 0) {
    --ws.gait_switch_cooldown_counter;
  } else {
    int ng = this->params.Get<int>("num_gaits");
    if (IsActionActive("gait_next")) {
      ws.gait_choice = (ws.gait_choice + 1) % ng;
      ws.gait_switch_cooldown_counter = ws.gait_switch_cooldown_max;
      std::cout << std::endl
                << LOGGER::INFO << "[WTW] Gait -> " << ws.gait_choice
                << std::endl;
    } else if (IsActionActive("gait_prev")) {
      ws.gait_choice = (ws.gait_choice - 1 + ng) % ng;
      ws.gait_switch_cooldown_counter = ws.gait_switch_cooldown_max;
      std::cout << std::endl
                << LOGGER::INFO << "[WTW] Gait -> " << ws.gait_choice
                << std::endl;
    }
  }
}

template <typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node& node) {
  std::vector<T> values;
  for (const auto& val : node) {
    values.push_back(val.as<T>());
  }
  return values;
}

void RL::ReadYaml(const std::string& file_path, const std::string& file_name) {
  std::string config_path =
      std::string(POLICY_DIR) + "/" + file_path + "/" + file_name;
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_path)[file_path];
  } catch (YAML::BadFile& e) {
    std::cout << LOGGER::ERROR << "The file '" << config_path
              << "' does not exist" << std::endl;
    return;
  }

  for (auto it = config.begin(); it != config.end(); ++it) {
    std::string key = it->first.as<std::string>();
    this->params.config_node[key] = it->second;
  }
}

bool RLFSMState::Interpolate(float& percent,
                             const std::vector<float>& start_pos,
                             const std::vector<float>& target_pos,
                             float duration_seconds,
                             const std::string& description,
                             bool use_fixed_gains) {
  if (percent >= 1.0f) {
    return false;
  }

  if (percent == 0.0f) {
    float max_diff = 0.0f;
    for (size_t i = 0; i < start_pos.size() && i < target_pos.size(); ++i) {
      max_diff = std::max(max_diff, std::abs(start_pos[i] - target_pos[i]));
    }

    if (max_diff < 0.1f) {
      percent = 1.0f;
    }
  }

  int required_frames =
      std::max(1, static_cast<int>(std::ceil(duration_seconds /
                                             rl.params.Get<float>("dt"))));
  float step = 1.0f / required_frames;

  percent += step;
  percent = std::min(percent, 1.0f);

  auto kp = use_fixed_gains ? rl.params.Get<std::vector<float>>("fixed_kp")
                            : rl.params.Get<std::vector<float>>("rl_kp");
  auto kd = use_fixed_gains ? rl.params.Get<std::vector<float>>("fixed_kd")
                            : rl.params.Get<std::vector<float>>("rl_kd");

  for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i) {
    fsm_command->motor_command.q[i] =
        (1 - percent) * start_pos[i] + percent * target_pos[i];
    fsm_command->motor_command.dq[i] = 0;
    fsm_command->motor_command.kp[i] = kp[i];
    fsm_command->motor_command.kd[i] = kd[i];
    fsm_command->motor_command.tau[i] = 0;
  }

  if (!description.empty()) {
    LOGGER::PrintProgress(percent, description);
  }

  if (percent >= 1.0f) {
    return false;
  }

  return true;
}

void RLFSMState::RLControl() {
  std::vector<float> _output_dof_pos, _output_dof_vel;
  if (rl.output_dof_pos_queue.try_pop(_output_dof_pos) &&
      rl.output_dof_vel_queue.try_pop(_output_dof_vel)) {
    for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i) {
      if (!_output_dof_pos.empty()) {
        fsm_command->motor_command.q[i] = _output_dof_pos[i];
      }
      if (!_output_dof_vel.empty()) {
        fsm_command->motor_command.dq[i] = _output_dof_vel[i];
      }
      fsm_command->motor_command.kp[i] =
          rl.params.Get<std::vector<float>>("rl_kp")[i];
      fsm_command->motor_command.kd[i] =
          rl.params.Get<std::vector<float>>("rl_kd")[i];
      fsm_command->motor_command.tau[i] = 0;
    }
  }
}
