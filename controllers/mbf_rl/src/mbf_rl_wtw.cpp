#include "mbf_rl_wtw.hpp"

#include "fsm_wtw.hpp"

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
  this->fsm.AddState(std::make_shared<wtw_fsm::StatePassive>(this));
  this->fsm.AddState(std::make_shared<wtw_fsm::StateGetUp>(this));
  this->fsm.AddState(std::make_shared<wtw_fsm::StateGetDown>(this));
  this->fsm.AddState(std::make_shared<wtw_fsm::StateWTWLocomotion>(this));
  this->fsm.SetInitialState("Passive");

  // ROS2 pub/sub
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

  std::cout << LOGGER::INFO << "MBF_RL_WTW start" << std::endl;
}

MBF_RL_WTW::~MBF_RL_WTW() {
  this->loop_keyboard->shutdown();
  this->loop_control->shutdown();
  this->loop_rl->shutdown();
  std::cout << LOGGER::INFO << "MBF_RL_WTW exit" << std::endl;
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
// Control loop — all input handling in one place
// =========================================================================

void MBF_RL_WTW::RobotControl() {
  this->GetState(&this->robot_state);

  // Velocity from configurable keys
  auto kb = this->control.current_keyboard;
  if (kb == key_mapping["forward"])   this->control.x += vel_step;
  if (kb == key_mapping["backward"])  this->control.x -= vel_step;
  if (kb == key_mapping["left"])      this->control.y += vel_step;
  if (kb == key_mapping["right"])     this->control.y -= vel_step;
  if (kb == key_mapping["yaw_left"])  this->control.yaw += vel_step;
  if (kb == key_mapping["yaw_right"]) this->control.yaw -= vel_step;
  if (kb == key_mapping["stop"]) {
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

  // WTW style adjustments (harmless when not in WTW locomotion)
  this->ProcessWTWControls();

  // FSM update (don't use StateController — it has hardcoded WASD velocity)
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
// main
// =========================================================================

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto wtw = std::make_shared<MBF_RL_WTW>(argc, argv);
  rclcpp::spin(wtw->node);
  rclcpp::shutdown();
  return 0;
}
