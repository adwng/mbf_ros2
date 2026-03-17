#include "mbf_control.hpp"

champ::PhaseGenerator::Time rosTimeToChampTime(const rclcpp::Time& now) {
  static rclcpp::Time start_time = now;
  return (now - start_time).nanoseconds() / 1000ul;
}

MBFControl::MBFControl()
    : Node("mbf_control_node",
           rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true)),
      clock_(*this->get_clock()),
      body_controller_(base_),
      leg_controller_(base_, rosTimeToChampTime(clock_.now())),
      kinematics_(base_) {
  joySub = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::SensorDataQoS(),
      std::bind(&MBFControl::joyCallback, this, std::placeholders::_1));

  robotStateSub_ = this->create_subscription<robot_msgs::msg::RobotState>(
      "robot_controller/state", rclcpp::SensorDataQoS(),
      std::bind(&MBFControl::robotStateCallback, this, std::placeholders::_1));

  robotCmdPub = this->create_publisher<robot_msgs::msg::RobotCommand>(
      "robot_controller/command", rclcpp::SystemDefaultsQoS());

  std::string knee_orientation;
  std::string urdf;

  params.ctrl_freq = this->get_parameter("ctrl_freq").as_int();
  params.control.kp = this->get_parameter("control.kp").as_double();
  params.control.kd = this->get_parameter("control.kd").as_double();
  params.control.max_x = this->get_parameter("control.max_x").as_double();
  params.control.max_y = this->get_parameter("control.max_y").as_double();
  params.control.max_z = this->get_parameter("control.max_z").as_double();
  params.control.max_roll = this->get_parameter("control.max_roll").as_double();
  params.control.max_pitch =
      this->get_parameter("control.max_pitch").as_double();
  params.control.max_yaw = this->get_parameter("control.max_yaw").as_double();

  params.gamepad.passive_btn =
      this->get_parameter("gamepad.passive_btn").as_int();
  params.gamepad.stand_btn = this->get_parameter("gamepad.stand_btn").as_int();
  params.gamepad.locomotion_btn =
      this->get_parameter("gamepad.locomotion_btn").as_int();
  params.gamepad.down_btn = this->get_parameter("gamepad.down_btn").as_int();
  params.gamepad.l_btn_macro =
      this->get_parameter("gamepad.l_btn_macro").as_int();
  params.gamepad.l2_btn_macro =
      this->get_parameter("gamepad.l2_btn_macro").as_int();
  params.gamepad.r_btn_macro =
      this->get_parameter("gamepad.r_btn_macro").as_int();

  params.gamepad.left_ud_axis =
      this->get_parameter("gamepad.left_ud_axis").as_int();
  params.gamepad.left_lr_axis =
      this->get_parameter("gamepad.left_lr_axis").as_int();
  params.gamepad.right_ud_axis =
      this->get_parameter("gamepad.right_ud_axis").as_int();
  params.gamepad.right_lr_axis =
      this->get_parameter("gamepad.right_lr_axis").as_int();

  this->get_parameter("gait.pantograph_leg", gait_config_.pantograph_leg);
  this->get_parameter("gait.max_linear_velocity_x",
                      gait_config_.max_linear_velocity_x);
  this->get_parameter("gait.max_linear_velocity_y",
                      gait_config_.max_linear_velocity_y);
  this->get_parameter("gait.max_angular_velocity_z",
                      gait_config_.max_angular_velocity_z);
  this->get_parameter("gait.com_x_translation", gait_config_.com_x_translation);
  this->get_parameter("gait.swing_height", gait_config_.swing_height);
  this->get_parameter("gait.stance_depth", gait_config_.stance_depth);
  this->get_parameter("gait.stance_duration", gait_config_.stance_duration);
  this->get_parameter("gait.nominal_height", gait_config_.nominal_height);
  this->get_parameter("gait.knee_orientation", knee_orientation);
  gait_config_.knee_orientation = knee_orientation.c_str();
  this->get_parameter("urdf", urdf);

  try {
    champ::URDF::loadFromString(base_, this->get_node_parameters_interface(),
                                urdf);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load URDF: %s", e.what());
    return;
  }

  std::array<double, NUM_JOINTS> PASSIVE_POSES = {
      0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5,
  };

  std::array<double, NUM_JOINTS> STANDING_POSE = {
      0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5,
  };

  poses_.passive = PASSIVE_POSES;
  poses_.standing = STANDING_POSE;
  fsm_state = FSMState::PASSIVE;

  std::chrono::milliseconds period(static_cast<int>(1000 / params.ctrl_freq));
  loop_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&MBFControl::loop, this));
  req_pose_.position.z = gait_config_.nominal_height;
}

double MBFControl::smooth_ratio(double t, double T) {
  double s = std::clamp(t / T, 0.0, 1.0);
  // 6s^5 - 15s^4 + 10s^3: zero velocity and zero acceleration at endpoints
  return s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
}

void MBFControl::begin_transition(const std::array<double, NUM_JOINTS>& target,
                                  double duration) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    interp_start_q_[i] = jointStateData.q[i];  // snapshot actual position
  }
  interp_target_q_ = target;
  interp_duration_ = duration;
  interp_elapsed_ = 0.0;
  in_transition_ = true;
}

void MBFControl::run_interpolation(double dt, int dir) {
  interp_elapsed_ += dt;
  double ratio = smooth_ratio(interp_elapsed_, interp_duration_);
  //   double gain_scale = (dir == 1) ? ratio : (1.0 - ratio);
  double gain_scale = 1.0;
  for (int i = 0; i < NUM_JOINTS; i++) {
    double q_des =
        interp_start_q_[i] + ratio * (interp_target_q_[i] - interp_start_q_[i]);

    jointCommandData.q[i] = q_des;
    jointCommandData.dq[i] = 0.0;
    jointCommandData.kp[i] = gain_scale * params.control.kp;
    jointCommandData.kd[i] = gain_scale * params.control.kd;
    jointCommandData.tau[i] = 0.0;
  }
}

bool MBFControl::toggle_state(bool current_state, int btn_idx,
                              sensor_msgs::msg::Joy::SharedPtr msg) {
  // 1. Safety check for vector size
  if (btn_idx >= static_cast<int>(msg->buttons.size())) return current_state;

  bool pressed = msg->buttons[btn_idx];
  bool prev_pressed =
      first_joy_received_ ? last_joy_msg_.buttons[btn_idx] : false;

  // 2. Only toggle if we see a 0 -> 1 transition
  if (pressed && !prev_pressed) {
    RCLCPP_INFO(this->get_logger(), "Button %d toggled state to %s", btn_idx,
                !current_state ? "TRUE" : "FALSE");
    return !current_state;
  }
  return current_state;
}

void MBFControl::robotStateCallback(
    robot_msgs::msg::RobotState::SharedPtr msg) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    jointStateData.q[i] = msg->motor_state[i].q;
    jointStateData.dq[i] = msg->motor_state[i].dq;
    jointStateData.tau_est[i] = msg->motor_state[i].tau_est;
  }
}

void MBFControl::joyCallback(sensor_msgs::msg::Joy::SharedPtr msg) {
  auto detect_edge = [&](int btn_idx) {
    if (btn_idx < 0 || btn_idx >= (int)msg->buttons.size()) return false;
    bool pressed = msg->buttons[btn_idx];
    bool prev = first_joy_received_ ? last_joy_msg_.buttons[btn_idx] : false;
    return pressed && !prev;
  };

  // Set triggers (events)
  if (detect_edge(params.gamepad.passive_btn)) gamepadData.passive = true;
  if (detect_edge(params.gamepad.stand_btn)) gamepadData.standup = true;
  if (detect_edge(params.gamepad.locomotion_btn)) gamepadData.locomotion = true;
  if (detect_edge(params.gamepad.down_btn)) gamepadData.getdown = true;

  bool L1 = msg->buttons[params.gamepad.l_btn_macro];
  bool L2 = msg->buttons[params.gamepad.l2_btn_macro];
  bool R1 = msg->buttons[params.gamepad.r_btn_macro];

  if (L2) {
    req_pose_.position.x =
        params.control.max_x * msg->axes[params.gamepad.left_ud_axis];

    req_pose_.position.y =
        params.control.max_y * msg->axes[params.gamepad.left_lr_axis];
  } else if (L1) {
    req_vel_.linear.y = msg->axes[params.gamepad.left_lr_axis];
  } else {
    req_vel_.linear.x = msg->axes[params.gamepad.left_ud_axis];
    req_vel_.angular.z = msg->axes[params.gamepad.left_lr_axis];
  }

  if (R1) {
    req_pose_.position.z =
        (params.control.max_z * msg->axes[params.gamepad.right_ud_axis]) +
        gait_config_.nominal_height;
    req_pose_.orientation.yaw =
        params.control.max_y * msg->axes[params.gamepad.right_lr_axis];
  } else {
    req_pose_.orientation.pitch =
        params.control.max_pitch * msg->axes[params.gamepad.right_ud_axis];
    req_pose_.orientation.roll =
        params.control.max_roll * msg->axes[params.gamepad.right_lr_axis];
  }

  last_joy_msg_ = *msg;
  first_joy_received_ = true;
}

void MBFControl::publishCommands() {
  robot_msgs::msg::RobotCommand msg;
  msg.motor_command.resize(NUM_JOINTS);
  for (int i = 0; i < NUM_JOINTS; i++) {
    msg.motor_command[i].q = jointCommandData.q[i];
    msg.motor_command[i].dq = jointCommandData.dq[i];
    msg.motor_command[i].tau = jointCommandData.tau[i];
    msg.motor_command[i].kp = jointCommandData.kp[i];
    msg.motor_command[i].kd = jointCommandData.kd[i];
  }

  robotCmdPub->publish(msg);
}

void MBFControl::set_passive_commands() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    jointCommandData.q[i] = 0.0;
    jointCommandData.dq[i] = 0.0;
    jointCommandData.kp[i] = 0.0;
    jointCommandData.kd[i] = 0.0;
    jointCommandData.tau[i] = 0.0;
  }
}

void MBFControl::compute_locomotion() {
  req_vel_.linear.x = 0.5;
  RCLCPP_INFO(this->get_logger(), "stance_duration: %.3f",
              gait_config_.stance_duration);

  float target_joint_positions[12];
  geometry::Transformation target_foot_positions[4];

  body_controller_.poseCommand(target_foot_positions, req_pose_);

  float z_before = target_foot_positions[0].Z();

  auto vel_copy = req_vel_;
  leg_controller_.velocityCommand(target_foot_positions, vel_copy,
                                  rosTimeToChampTime(clock_.now()));

  float z_after = target_foot_positions[0].Z();

  RCLCPP_INFO(this->get_logger(), "Before: %.2f | After: %.2f",
              req_vel_.linear.x, vel_copy.linear.x);

  kinematics_.inverse(target_joint_positions, target_foot_positions);

  for (int i = 0; i < NUM_JOINTS; i++) {
    jointCommandData.q[i] = (double)target_joint_positions[i];
    jointCommandData.dq[i] = 0.0;
    jointCommandData.kp[i] = params.control.kp;
    jointCommandData.kd[i] = params.control.kd;
    jointCommandData.tau[i] = 0.0;
  }
}

void MBFControl::loop() {
  // 1. Safety Gate: If we haven't received a joystick message, don't log or run
  // math
  if (!first_joy_received_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Waiting for joystick input...");
    return;
  }

  // 2. Safety Gate: Ensure the axes vector is actually the size we expect
  // (Prevents crashing if a different controller is plugged in)
  if (last_joy_msg_.axes.size() < 4) {
    return;
  }

  double dt = static_cast<double>(1.0 / params.ctrl_freq);

  switch (fsm_state) {
    case FSMState::PASSIVE:
      state_string_ = "PASSIVE";
      set_passive_commands();

      if (gamepadData.standup) {
        gamepadData.standup = false;
        begin_transition(poses_.standing, params.control.standup_duration);
        fsm_state = FSMState::STANDUP;
      }
      break;

    case FSMState::STANDUP:
      state_string_ = "STANDUP";
      run_interpolation(dt, 1);

      if (interp_elapsed_ >= interp_duration_) {
        in_transition_ = false;
      }

      if (gamepadData.passive) {
        gamepadData.passive = false;
        fsm_state = FSMState::PASSIVE;
      }
      if (gamepadData.locomotion) {
        gamepadData.locomotion = false;
        fsm_state = FSMState::LOCOMOTION;
      }
      if (gamepadData.getdown) {
        gamepadData.getdown = false;
        begin_transition(poses_.passive, params.control.getdown_duration);
        fsm_state = FSMState::GETDOWN;
      }
      break;

    case FSMState::GETDOWN:
      state_string_ = "GETDOWN";
      run_interpolation(dt, -1);
      if (interp_elapsed_ >= interp_duration_) {
        in_transition_ = false;
        fsm_state = FSMState::PASSIVE;
      }
      break;

    case FSMState::LOCOMOTION:
      state_string_ = "LOCOMOTION";
      compute_locomotion();

      if (gamepadData.passive) {
        gamepadData.passive = false;
        fsm_state = FSMState::PASSIVE;
      }
      if (gamepadData.standup) {
        gamepadData.standup = false;
        begin_transition(poses_.standing, params.control.standup_duration);
        fsm_state = FSMState::STANDUP;
      }
      if (gamepadData.getdown) {
        gamepadData.getdown = false;
        begin_transition(poses_.passive, params.control.getdown_duration);
        fsm_state = FSMState::GETDOWN;
      }
      break;
  }

  publishCommands();
}