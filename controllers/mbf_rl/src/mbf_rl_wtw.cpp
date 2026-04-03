#include "mbf_rl_wtw.hpp"

#include "fsm_wtw.hpp"

// =========================================================================
// MBF_RL_WTW — standalone Walk These Ways controller (keyboard only)
// =========================================================================

MBF_RL_WTW::MBF_RL_WTW(int argc, char **argv) {
  node = std::make_shared<rclcpp::Node>("mbf_rl_wtw_node");
  this->ang_vel_axis = "body";

  robot_name = "mbf";
  this->ReadYaml(this->robot_name, "base.yaml");
  this->ReadYaml(this->robot_name + "/wtw", "config.yaml");
  this->LoadKeyMapping();

  int num_dofs = this->params.Get<int>("num_of_dofs");
  this->robot_command_pub_msg_.motor_command.resize(num_dofs);
  this->robot_state_sub_msg_.motor_state.resize(num_dofs);
  this->InitJointNum(num_dofs);
  this->InitOutputs();
  this->InitControl();

  // FSM
  this->fsm.AddState(
      std::make_shared<wtw_fsm::StatePassive>(this, "Passive"));
  this->fsm.AddState(std::make_shared<wtw_fsm::StateGetUp>(this, "GetUp"));
  this->fsm.AddState(
      std::make_shared<wtw_fsm::StateGetDown>(this, "GetDown"));
  this->fsm.AddState(
      std::make_shared<wtw_fsm::StateWTWLocomotion>(this, "WTWLocomotion"));
  this->fsm.SetInitialState("Passive");

  // ROS2 pub/sub (no joy — keyboard only)
  this->robot_command_pub_ =
      node->create_publisher<robot_msgs::msg::RobotCommand>(
          "robot_controller/command", rclcpp::SystemDefaultsQoS());

  this->cmd_vel_sub_ =
      node->create_subscription<geometry_msgs::msg::Twist>(
          "cmd_vel", rclcpp::SystemDefaultsQoS(),
          [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            this->cmd_vel_ = *msg;
          });

  this->imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        this->imu_ = *msg;
      });

  this->robot_state_sub_ =
      node->create_subscription<robot_msgs::msg::RobotState>(
          "robot_controller/state", rclcpp::SystemDefaultsQoS(),
          [this](const robot_msgs::msg::RobotState::SharedPtr msg) {
            this->robot_state_sub_msg_ = *msg;
          });

  // Loops
  this->loop_control = std::make_shared<LoopFunc>(
      "loop_control", this->params.Get<float>("dt"),
      std::bind(&MBF_RL_WTW::RobotControl, this));
  this->loop_rl = std::make_shared<LoopFunc>(
      "loop_rl",
      this->params.Get<float>("dt") * this->params.Get<int>("decimation"),
      std::bind(&MBF_RL_WTW::RunModel, this));
  this->loop_control->start();
  this->loop_rl->start();

  this->loop_keyboard = std::make_shared<LoopFunc>(
      "loop_keyboard", 0.05,
      std::bind(&MBF_RL_WTW::KeyboardInterface, this));
  this->loop_keyboard->start();

  std::cout << LOGGER::INFO << "MBF_RL_WTW start (keyboard only)" << std::endl;
}

MBF_RL_WTW::~MBF_RL_WTW() {
  this->loop_keyboard->shutdown();
  this->loop_control->shutdown();
  this->loop_rl->shutdown();
  std::cout << LOGGER::INFO << "MBF_RL_WTW exit" << std::endl;
}

// =========================================================================
// Keyboard mapping — configurable via YAML
// =========================================================================

Input::Keyboard MBF_RL_WTW::KeyFromString(const std::string &s) {
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

void MBF_RL_WTW::LoadKeyMapping() {
  YAML::Node node = this->params.config_node["key_mapping"];
  if (!node || !node.IsMap()) {
    std::cout << LOGGER::WARNING
              << "[WTW] No key_mapping in config, using defaults" << std::endl;
    // Sensible defaults
    key_mapping_["forward"] = Input::Keyboard::W;
    key_mapping_["backward"] = Input::Keyboard::S;
    key_mapping_["left"] = Input::Keyboard::A;
    key_mapping_["right"] = Input::Keyboard::D;
    key_mapping_["yaw_left"] = Input::Keyboard::Q;
    key_mapping_["yaw_right"] = Input::Keyboard::E;
    key_mapping_["stop"] = Input::Keyboard::Space;
    key_mapping_["getup"] = Input::Keyboard::Num0;
    key_mapping_["getdown"] = Input::Keyboard::Num9;
    key_mapping_["passive"] = Input::Keyboard::P;
    key_mapping_["locomotion"] = Input::Keyboard::Num1;
    key_mapping_["nav_mode"] = Input::Keyboard::N;
    return;
  }

  for (auto it = node.begin(); it != node.end(); ++it) {
    std::string action = it->first.as<std::string>();
    std::string key_str = it->second.as<std::string>();
    Input::Keyboard kb = KeyFromString(key_str);
    if (kb != Input::Keyboard::None) {
      key_mapping_[action] = kb;
    } else {
      std::cout << LOGGER::WARNING << "[WTW] Unknown key '" << key_str
                << "' for action '" << action << "'" << std::endl;
    }
  }

  vel_step_ = this->params.Get<float>("vel_step", 0.1f);

  std::cout << LOGGER::INFO << "[WTW] Key mapping loaded — "
            << key_mapping_.size() << " actions" << std::endl;
}

bool MBF_RL_WTW::IsActionActive(const std::string &action) const {
  auto it = key_mapping_.find(action);
  if (it == key_mapping_.end()) return false;
  return this->control.current_keyboard == it->second;
}

// =========================================================================
// Robot state / command
// =========================================================================

void MBF_RL_WTW::GetState(RobotState<float> *state) {
  state->imu.quaternion[0] = imu_.orientation.w;
  state->imu.quaternion[1] = imu_.orientation.x;
  state->imu.quaternion[2] = imu_.orientation.y;
  state->imu.quaternion[3] = imu_.orientation.z;
  state->imu.gyroscope[0] = imu_.angular_velocity.x;
  state->imu.gyroscope[1] = imu_.angular_velocity.y;
  state->imu.gyroscope[2] = imu_.angular_velocity.z;

  auto jmap = this->params.Get<std::vector<int>>("joint_mapping");
  for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) {
    state->motor_state.q[i] = robot_state_sub_msg_.motor_state[jmap[i]].q;
    state->motor_state.dq[i] = robot_state_sub_msg_.motor_state[jmap[i]].dq;
    state->motor_state.tau_est[i] =
        robot_state_sub_msg_.motor_state[jmap[i]].tau_est;
  }
}

void MBF_RL_WTW::SetCommand(const RobotCommand<float> *command) {
  auto jmap = this->params.Get<std::vector<int>>("joint_mapping");
  for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) {
    robot_command_pub_msg_.motor_command[jmap[i]].q =
        command->motor_command.q[i];
    robot_command_pub_msg_.motor_command[jmap[i]].dq =
        command->motor_command.dq[i];
    robot_command_pub_msg_.motor_command[jmap[i]].kp =
        command->motor_command.kp[i];
    robot_command_pub_msg_.motor_command[jmap[i]].kd =
        command->motor_command.kd[i];
    robot_command_pub_msg_.motor_command[jmap[i]].tau =
        command->motor_command.tau[i];
  }
  robot_command_pub_->publish(robot_command_pub_msg_);
}

// =========================================================================
// Control loop
// =========================================================================

void MBF_RL_WTW::RobotControl() {
  this->GetState(&this->robot_state);

  // Velocity from configurable keyboard keys
  auto kb = this->control.current_keyboard;
  if (kb == key_mapping_["forward"])   this->control.x += vel_step_;
  if (kb == key_mapping_["backward"])  this->control.x -= vel_step_;
  if (kb == key_mapping_["left"])      this->control.y += vel_step_;
  if (kb == key_mapping_["right"])     this->control.y -= vel_step_;
  if (kb == key_mapping_["yaw_left"])  this->control.yaw += vel_step_;
  if (kb == key_mapping_["yaw_right"]) this->control.yaw -= vel_step_;
  if (kb == key_mapping_["stop"]) {
    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;
  }

  if (IsActionActive("nav_mode")) {
    this->control.navigation_mode = !this->control.navigation_mode;
    std::cout << std::endl
              << LOGGER::INFO << "Navigation mode: "
              << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
  }

  // FSM
  auto updateFSMState = [&](std::shared_ptr<FSMState> statePtr) {
    if (auto s = std::dynamic_pointer_cast<RLFSMState>(statePtr)) {
      s->fsm_state = &this->robot_state;
      s->fsm_command = &this->robot_command;
    }
  };
  for (auto &pair : fsm.states_) updateFSMState(pair.second);

  fsm.Run();
  this->motiontime++;

  this->control.ClearInput();
  this->SetCommand(&this->robot_command);
}

// =========================================================================
// RL inference loop
// =========================================================================

void MBF_RL_WTW::RunModel() {
  if (this->rl_init_done && simulation_running) {
    this->episode_length_buf += 1;

    this->PreProcessGait();

    this->obs.ang_vel = this->robot_state.imu.gyroscope;
    this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
    if (this->control.navigation_mode) {
      this->obs.commands = {static_cast<float>(cmd_vel_.linear.x),
                            static_cast<float>(cmd_vel_.linear.y),
                            static_cast<float>(cmd_vel_.angular.z)};
    }
    this->obs.base_quat = this->robot_state.imu.quaternion;
    this->obs.dof_pos = this->robot_state.motor_state.q;
    this->obs.dof_vel = this->robot_state.motor_state.dq;

    this->obs.actions = this->Forward();
    this->ComputeOutput(this->obs.actions, this->output_dof_pos,
                        this->output_dof_vel, this->output_dof_tau);

    if (!this->output_dof_pos.empty())
      output_dof_pos_queue.push(this->output_dof_pos);
    if (!this->output_dof_vel.empty())
      output_dof_vel_queue.push(this->output_dof_vel);
    if (!this->output_dof_tau.empty())
      output_dof_tau_queue.push(this->output_dof_tau);
  }
}

std::vector<float> MBF_RL_WTW::Forward() {
  std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);
  if (!lock.owns_lock()) {
    std::cout << LOGGER::WARNING
              << "Model is being reinitialized, using previous actions"
              << std::endl;
    return this->obs.actions;
  }

  std::vector<float> clamped_obs = this->ComputeObservation();

  std::vector<float> actions;
  if (this->params.Get<std::vector<int>>("observations_history").size() != 0) {
    this->history_obs_buf.insert(clamped_obs);
    this->history_obs = this->history_obs_buf.get_obs_vec(
        this->params.Get<std::vector<int>>("observations_history"));
    actions = this->model->forward({this->history_obs});
  } else {
    actions = this->model->forward({clamped_obs});
  }

  if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() &&
      !this->params.Get<std::vector<float>>("clip_actions_lower").empty()) {
    return clamp(actions,
                 this->params.Get<std::vector<float>>("clip_actions_lower"),
                 this->params.Get<std::vector<float>>("clip_actions_upper"));
  }
  return actions;
}

// =========================================================================
// WTW gait / style logic
// =========================================================================

void MBF_RL_WTW::InitWTWState() {
  auto gp = this->params.Get<std::vector<float>>("gait_period_range");
  auto bh = this->params.Get<std::vector<float>>("base_height_range");
  auto fc = this->params.Get<std::vector<float>>("foot_clearance_range");
  auto pr = this->params.Get<std::vector<float>>("pitch_range");

  wtw_state.gait_period = gp[1];
  wtw_state.base_height = bh[0];
  wtw_state.foot_clearance = fc[0];
  wtw_state.pitch = pr[1];
  wtw_state.gait_time = 0.0f;
  wtw_state.phi = 0.0f;
  wtw_state.gait_choice = 0;
  wtw_state.adjust_step =
      this->params.Get<float>("wtw_adjust_step", 0.01f);
  wtw_state.gait_switch_cooldown_max =
      this->params.Get<int>("gait_switch_cooldown", 100);
  wtw_state.gait_switch_cooldown_counter = 0;

  std::cout << LOGGER::INFO << "[WTW] Initialized — gaits: "
            << this->params.Get<int>("num_gaits")
            << ", period: " << wtw_state.gait_period << std::endl;
}

void MBF_RL_WTW::PreProcessGait() {
  auto &ws = wtw_state;
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

void MBF_RL_WTW::ProcessWTWControls() {
  auto &ws = wtw_state;

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

// =========================================================================
// WTW FSM state implementations
// =========================================================================

wtw_fsm::WTWFSMState::WTWFSMState(MBF_RL_WTW *wtw, const std::string &name)
    : RLFSMState(*wtw, name), wtw_(wtw) {}

bool wtw_fsm::WTWFSMState::IsAction(const std::string &action) const {
  return wtw_->IsActionActive(action);
}

// --- Passive ---

void wtw_fsm::StatePassive::Enter() {
  std::cout << LOGGER::NOTE
            << "WTW Passive. Press 'getup' key to stand." << std::endl;
}

void wtw_fsm::StatePassive::Run() {
  for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i) {
    fsm_command->motor_command.dq[i] = 0;
    fsm_command->motor_command.kp[i] = 0;
    fsm_command->motor_command.kd[i] = 8;
    fsm_command->motor_command.tau[i] = 0;
  }
}

void wtw_fsm::StatePassive::Exit() {}

std::string wtw_fsm::StatePassive::CheckChange() {
  if (IsAction("getup")) return "GetUp";
  return state_name_;
}

// --- GetUp ---

void wtw_fsm::StateGetUp::Enter() {
  percent_pre_getup = 0.0f;
  percent_getup = 0.0f;
  stand_from_passive =
      (rl.fsm.previous_state_ &&
       rl.fsm.previous_state_->GetStateName() == "Passive");
  rl.now_state = *fsm_state;
  rl.start_state = rl.now_state;
}

void wtw_fsm::StateGetUp::Run() {
  std::vector<float> pre_pos = {0.00, 1.36, -2.65, 0.00, 1.36, -2.65,
                                0.00, 1.36, -2.65, 0.00, 1.36, -2.65,
                                0.00, 0.00, 0.00,  0.00};
  if (stand_from_passive) {
    if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, pre_pos,
                    1.0f, "Pre Getting up", true))
      return;
    if (Interpolate(
            percent_getup, pre_pos,
            rl.params.Get<std::vector<float>>("default_dof_pos"), 2.0f,
            "Getting up", true))
      return;
  } else {
    if (Interpolate(
            percent_getup, rl.now_state.motor_state.q,
            rl.params.Get<std::vector<float>>("default_dof_pos"), 1.0f,
            "Getting up", true))
      return;
  }
}

void wtw_fsm::StateGetUp::Exit() {}

std::string wtw_fsm::StateGetUp::CheckChange() {
  if (IsAction("passive")) return "Passive";
  if (percent_getup >= 1.0f) {
    if (IsAction("locomotion")) return "WTWLocomotion";
    if (IsAction("getdown")) return "GetDown";
  }
  return state_name_;
}

// --- GetDown ---

void wtw_fsm::StateGetDown::Enter() {
  percent_getdown = 0.0f;
  rl.now_state = *fsm_state;
}

void wtw_fsm::StateGetDown::Run() {
  Interpolate(percent_getdown, rl.now_state.motor_state.q,
              rl.start_state.motor_state.q, 2.0f, "Getting down", true);
}

void wtw_fsm::StateGetDown::Exit() {}

std::string wtw_fsm::StateGetDown::CheckChange() {
  if (IsAction("passive") || percent_getdown >= 1.0f) return "Passive";
  if (IsAction("getup")) return "GetUp";
  return state_name_;
}

// --- WTW Locomotion ---

void wtw_fsm::StateWTWLocomotion::Enter() {
  rl.episode_length_buf = 0;

  rl.config_name = "wtw";
  std::string path = rl.robot_name + "/" + rl.config_name;
  try {
    rl.InitRL(path);
    rl.now_state = *fsm_state;
    wtw_->InitWTWState();
  } catch (const std::exception &e) {
    std::cout << LOGGER::ERROR << "InitRL(wtw) failed: " << e.what()
              << std::endl;
    rl.rl_init_done = false;
    rl.fsm.RequestStateChange("Passive");
  }
}

void wtw_fsm::StateWTWLocomotion::Run() {
  if (!rl.rl_init_done) rl.rl_init_done = true;

  wtw_->ProcessWTWControls();

  auto &ws = wtw_->wtw_state;
  std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "WTW x:"
            << rl.control.x << " y:" << rl.control.y
            << " yaw:" << rl.control.yaw << " | gait:" << ws.gait_choice
            << " T:" << ws.gait_period << " h:" << ws.base_height
            << std::flush;

  RLControl();
}

void wtw_fsm::StateWTWLocomotion::Exit() { rl.rl_init_done = false; }

std::string wtw_fsm::StateWTWLocomotion::CheckChange() {
  if (IsAction("passive")) return "Passive";
  if (IsAction("getdown")) return "GetDown";
  if (IsAction("getup")) return "GetUp";
  return state_name_;
}

// =========================================================================
// main
// =========================================================================

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto wtw = std::make_shared<MBF_RL_WTW>(argc, argv);
  rclcpp::spin(wtw->node);
  rclcpp::shutdown();
  return 0;
}
