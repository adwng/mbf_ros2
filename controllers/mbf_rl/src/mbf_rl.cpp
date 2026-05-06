#include "mbf_rl.hpp"

MBF_RL::MBF_RL(int argc, char **argv) {
  node = std::make_shared<rclcpp::Node>("mbf_rl_node");
  this->ang_vel_axis = "body";

  robot_name = "mbf";
  this->ReadYaml(this->robot_name, "base.yaml");

  // auto load FSM by robot_name
  if (FSMManager::GetInstance().IsTypeSupported(this->robot_name)) {
    auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
    if (fsm_ptr) {
      this->fsm = *fsm_ptr;
    }
  } else {
    std::cout << LOGGER::ERROR
              << "[FSM] No FSM registered for robot: " << this->robot_name
              << std::endl;
  }

  this->robot_command_publisher_msg.motor_command.resize(
      this->params.Get<int>("num_of_dofs"));
  this->robot_state_subscriber_msg.motor_state.resize(
      this->params.Get<int>("num_of_dofs"));

  this->InitJointNum(this->params.Get<int>("num_of_dofs"));
  this->InitOutputs();
  this->InitControl();

  this->robot_command_publisher =
      node->create_publisher<robot_msgs::msg::RobotCommand>(
          "robot_controller/command", rclcpp::SystemDefaultsQoS());

  this->cmd_vel_subscriber =
      node->create_subscription<geometry_msgs::msg::Twist>(
          "cmd_vel", rclcpp::SystemDefaultsQoS(),
          [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            this->CmdvelCallback(msg);
          });

  this->joy_subscriber = node->create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
        this->JoyCallback(msg);
      });

  this->imu_subscriber = node->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        this->ImuCallback(msg);
      });

  this->robot_state_subscriber =
      node->create_subscription<robot_msgs::msg::RobotState>(
          "robot_controller/state", rclcpp::SystemDefaultsQoS(),
          [this](const robot_msgs::msg::RobotState::SharedPtr msg) {
            this->RobotStateCallback(msg);
          });

  // loop
  this->loop_control =
      std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"),
                                 std::bind(&MBF_RL::RobotControl, this));
  this->loop_rl = std::make_shared<LoopFunc>(
      "loop_rl",
      this->params.Get<float>("dt") * this->params.Get<int>("decimation"),
      std::bind(&MBF_RL::RunModel, this));
  this->loop_control->start();
  this->loop_rl->start();

  // keyboard
  this->loop_keyboard = std::make_shared<LoopFunc>(
      "loop_keyboard", 0.05, std::bind(&MBF_RL::KeyboardInterface, this));
  this->loop_keyboard->start();

  std::cout << LOGGER::INFO << "MBF_RL start" << std::endl;
}

MBF_RL::~MBF_RL() {
  this->loop_keyboard->shutdown();
  this->loop_control->shutdown();
  this->loop_rl->shutdown();
  std::cout << LOGGER::INFO << "MBF_RL exit" << std::endl;
}

void MBF_RL::GetState(RobotState<float> *state) {
  const auto &orientation = this->imu.orientation;
  const auto &angular_velocity = this->imu.angular_velocity;

  state->imu.quaternion[0] = orientation.w;
  state->imu.quaternion[1] = orientation.x;
  state->imu.quaternion[2] = orientation.y;
  state->imu.quaternion[3] = orientation.z;

  state->imu.gyroscope[0] = angular_velocity.x;
  state->imu.gyroscope[1] = angular_velocity.y;
  state->imu.gyroscope[2] = angular_velocity.z;

  for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) {
    state->motor_state.q[i] =
        this->robot_state_subscriber_msg
            .motor_state[this->params.Get<std::vector<int>>("joint_mapping")[i]]
            .q;
    state->motor_state.dq[i] =
        this->robot_state_subscriber_msg
            .motor_state[this->params.Get<std::vector<int>>("joint_mapping")[i]]
            .dq;
    state->motor_state.tau_est[i] =
        this->robot_state_subscriber_msg
            .motor_state[this->params.Get<std::vector<int>>("joint_mapping")[i]]
            .tau_est;
  }
}

void MBF_RL::SetCommand(const RobotCommand<float> *command) {
  for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) {
    this->robot_command_publisher_msg
        .motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]]
        .q = command->motor_command.q[i];
    this->robot_command_publisher_msg
        .motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]]
        .dq = command->motor_command.dq[i];
    this->robot_command_publisher_msg
        .motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]]
        .kp = command->motor_command.kp[i];
    this->robot_command_publisher_msg
        .motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]]
        .kd = command->motor_command.kd[i];
    this->robot_command_publisher_msg
        .motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]]
        .tau = command->motor_command.tau[i];
  }

  this->robot_command_publisher->publish(this->robot_command_publisher_msg);
}

void MBF_RL::RobotControl() {
  this->GetState(&this->robot_state);

  this->StateController(&this->robot_state, &this->robot_command);

  this->control.ClearInput();
  this->SetCommand(&this->robot_command);
}

void MBF_RL::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  this->imu = *msg;
}

void MBF_RL::CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  this->cmd_vel = *msg;
}

void MBF_RL::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  this->joy_msg = *msg;

  if (this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::A);
  if (this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::B);
  if (this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::X);
  if (this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::Y);
  if (this->joy_msg.buttons[4]) this->control.SetGamepad(Input::Gamepad::LB);
  if (this->joy_msg.buttons[5]) this->control.SetGamepad(Input::Gamepad::RB);
  if (this->joy_msg.buttons[9])
    this->control.SetGamepad(Input::Gamepad::LStick);
  if (this->joy_msg.buttons[10])
    this->control.SetGamepad(Input::Gamepad::RStick);
  if (this->joy_msg.axes[7] > 0)
    this->control.SetGamepad(Input::Gamepad::DPadUp);
  if (this->joy_msg.axes[7] < 0)
    this->control.SetGamepad(Input::Gamepad::DPadDown);
  if (this->joy_msg.axes[6] < 0)
    this->control.SetGamepad(Input::Gamepad::DPadLeft);
  if (this->joy_msg.axes[6] > 0)
    this->control.SetGamepad(Input::Gamepad::DPadRight);
  if (this->joy_msg.buttons[4] && this->joy_msg.buttons[0])
    this->control.SetGamepad(Input::Gamepad::LB_A);
  if (this->joy_msg.buttons[4] && this->joy_msg.buttons[1])
    this->control.SetGamepad(Input::Gamepad::LB_B);
  if (this->joy_msg.buttons[4] && this->joy_msg.buttons[2])
    this->control.SetGamepad(Input::Gamepad::LB_X);
  if (this->joy_msg.buttons[4] && this->joy_msg.buttons[3])
    this->control.SetGamepad(Input::Gamepad::LB_Y);
  if (this->joy_msg.buttons[4] && this->joy_msg.buttons[9])
    this->control.SetGamepad(Input::Gamepad::LB_LStick);
  if (this->joy_msg.buttons[4] && this->joy_msg.buttons[10])
    this->control.SetGamepad(Input::Gamepad::LB_RStick);
  if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] > 0)
    this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
  if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] < 0)
    this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
  if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] < 0)
    this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
  if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] > 0)
    this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
  if (this->joy_msg.buttons[5] && this->joy_msg.buttons[0])
    this->control.SetGamepad(Input::Gamepad::RB_A);
  if (this->joy_msg.buttons[5] && this->joy_msg.buttons[1])
    this->control.SetGamepad(Input::Gamepad::RB_B);
  if (this->joy_msg.buttons[5] && this->joy_msg.buttons[2])
    this->control.SetGamepad(Input::Gamepad::RB_X);
  if (this->joy_msg.buttons[5] && this->joy_msg.buttons[3])
    this->control.SetGamepad(Input::Gamepad::RB_Y);
  if (this->joy_msg.buttons[5] && this->joy_msg.buttons[9])
    this->control.SetGamepad(Input::Gamepad::RB_LStick);
  if (this->joy_msg.buttons[5] && this->joy_msg.buttons[10])
    this->control.SetGamepad(Input::Gamepad::RB_RStick);
  if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] > 0)
    this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
  if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] < 0)
    this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
  if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] < 0)
    this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
  if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] > 0)
    this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
  if (this->joy_msg.buttons[4] && this->joy_msg.buttons[5])
    this->control.SetGamepad(Input::Gamepad::LB_RB);

  this->control.x = this->joy_msg.axes[1];    // LY
  this->control.y = this->joy_msg.axes[0];    // LX
  this->control.yaw = this->joy_msg.axes[3];  // RX
}

void MBF_RL::RobotStateCallback(
    const robot_msgs::msg::RobotState::SharedPtr msg) {
  this->robot_state_subscriber_msg = *msg;
}

void MBF_RL::RunModel() {
  if (this->rl_init_done && simulation_running) {
    this->episode_length_buf += 1;
    this->obs.ang_vel = this->robot_state.imu.gyroscope;
    this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
    this->ComputeGaitPhase();
    if (this->control.navigation_mode) {
      this->obs.commands = {(float)this->cmd_vel.linear.x,
                            (float)this->cmd_vel.linear.y,
                            (float)this->cmd_vel.angular.z};
    }
    this->obs.base_quat = this->robot_state.imu.quaternion;
    this->obs.dof_pos = this->robot_state.motor_state.q;
    this->obs.dof_vel = this->robot_state.motor_state.dq;

    this->obs.actions = this->Forward();
    this->ComputeOutput(this->obs.actions, this->output_dof_pos,
                        this->output_dof_vel, this->output_dof_tau);

    if (!this->output_dof_pos.empty()) {
      output_dof_pos_queue.push(this->output_dof_pos);
    }
    if (!this->output_dof_vel.empty()) {
      output_dof_vel_queue.push(this->output_dof_vel);
    }
    if (!this->output_dof_tau.empty()) {
      output_dof_tau_queue.push(this->output_dof_tau);
    }

    // this->TorqueProtect(this->output_dof_tau);
    // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);
  }
}

std::vector<float> MBF_RL::Forward() {
  std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

  // If model is being reinitialized, return previous actions to avoid blocking
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
  } else {
    return actions;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto mbf_rl = std::make_shared<MBF_RL>(argc, argv);
  rclcpp::spin(mbf_rl->node);
  rclcpp::shutdown();

  return 0;
}
