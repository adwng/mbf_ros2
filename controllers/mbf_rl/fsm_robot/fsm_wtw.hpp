#ifndef WTW_FSM_HPP
#define WTW_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"

namespace wtw_fsm {

// --- Passive ---

class StatePassive : public RLFSMState {
 public:
  StatePassive(RL *rl) : RLFSMState(*rl, "Passive") {}

  void Enter() override {
    std::cout << LOGGER::NOTE
              << "WTW Passive. Press 'getup' key to stand." << std::endl;
  }

  void Run() override {
    for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i) {
      fsm_command->motor_command.dq[i] = 0;
      fsm_command->motor_command.kp[i] = 0;
      fsm_command->motor_command.kd[i] = 8;
      fsm_command->motor_command.tau[i] = 0;
    }
  }

  void Exit() override {}

  std::string CheckChange() override {
    if (rl.IsActionActive("getup")) return "GetUp";
    return state_name_;
  }
};

// --- GetUp ---

class StateGetUp : public RLFSMState {
 public:
  StateGetUp(RL *rl) : RLFSMState(*rl, "GetUp") {}

  float percent_pre_getup = 0.0f;
  float percent_getup = 0.0f;
  bool stand_from_passive = true;

  void Enter() override {
    percent_pre_getup = 0.0f;
    percent_getup = 0.0f;
    stand_from_passive =
        (rl.fsm.previous_state_ &&
         rl.fsm.previous_state_->GetStateName() == "Passive");
    rl.now_state = *fsm_state;
    rl.start_state = rl.now_state;
  }

  void Run() override {
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

  void Exit() override {}

  std::string CheckChange() override {
    if (rl.IsActionActive("passive")) return "Passive";
    if (percent_getup >= 1.0f) {
      if (rl.IsActionActive("locomotion")) return "WTWLocomotion";
      if (rl.IsActionActive("getdown")) return "GetDown";
    }
    return state_name_;
  }
};

// --- GetDown ---

class StateGetDown : public RLFSMState {
 public:
  StateGetDown(RL *rl) : RLFSMState(*rl, "GetDown") {}

  float percent_getdown = 0.0f;

  void Enter() override {
    percent_getdown = 0.0f;
    rl.now_state = *fsm_state;
  }

  void Run() override {
    Interpolate(percent_getdown, rl.now_state.motor_state.q,
                rl.start_state.motor_state.q, 2.0f, "Getting down", true);
  }

  void Exit() override {}

  std::string CheckChange() override {
    if (rl.IsActionActive("passive") || percent_getdown >= 1.0f)
      return "Passive";
    if (rl.IsActionActive("getup")) return "GetUp";
    return state_name_;
  }
};

// --- WTW Locomotion ---

class StateWTWLocomotion : public RLFSMState {
 public:
  StateWTWLocomotion(RL *rl) : RLFSMState(*rl, "WTWLocomotion") {}

  void Enter() override {
    rl.episode_length_buf = 0;

    rl.config_name = "wtw";
    std::string path = rl.robot_name + "/" + rl.config_name;
    try {
      rl.InitRL(path);
      rl.now_state = *fsm_state;
      rl.InitWTWState();
    } catch (const std::exception &e) {
      std::cout << LOGGER::ERROR << "InitRL(wtw) failed: " << e.what()
                << std::endl;
      rl.rl_init_done = false;
      rl.fsm.RequestStateChange("Passive");
    }
  }

  void Run() override {
    if (!rl.rl_init_done) rl.rl_init_done = true;

    auto &ws = rl.wtw_state;
    std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "WTW x:"
              << rl.control.x << " y:" << rl.control.y
              << " yaw:" << rl.control.yaw << " | gait:" << ws.gait_choice
              << " T:" << ws.gait_period << " h:" << ws.base_height
              << std::flush;

    RLControl();
  }

  void Exit() override { rl.rl_init_done = false; }

  std::string CheckChange() override {
    if (rl.IsActionActive("passive")) return "Passive";
    if (rl.IsActionActive("getdown")) return "GetDown";
    if (rl.IsActionActive("getup")) return "GetUp";
    return state_name_;
  }
};

}  // namespace wtw_fsm

#endif  // WTW_FSM_HPP
