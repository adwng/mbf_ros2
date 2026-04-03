#ifndef WTW_FSM_HPP
#define WTW_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"

class MBF_RL_WTW;

namespace wtw_fsm {

class WTWFSMState : public RLFSMState {
 public:
  WTWFSMState(MBF_RL_WTW *wtw, const std::string &name);

 protected:
  MBF_RL_WTW *wtw_;
  bool IsAction(const std::string &action) const;
};

class StatePassive : public WTWFSMState {
 public:
  using WTWFSMState::WTWFSMState;
  void Enter() override;
  void Run() override;
  void Exit() override;
  std::string CheckChange() override;
};

class StateGetUp : public WTWFSMState {
 public:
  using WTWFSMState::WTWFSMState;
  float percent_pre_getup = 0.0f;
  float percent_getup = 0.0f;
  bool stand_from_passive = true;
  void Enter() override;
  void Run() override;
  void Exit() override;
  std::string CheckChange() override;
};

class StateGetDown : public WTWFSMState {
 public:
  using WTWFSMState::WTWFSMState;
  float percent_getdown = 0.0f;
  void Enter() override;
  void Run() override;
  void Exit() override;
  std::string CheckChange() override;
};

class StateWTWLocomotion : public WTWFSMState {
 public:
  using WTWFSMState::WTWFSMState;
  void Enter() override;
  void Run() override;
  void Exit() override;
  std::string CheckChange() override;
};

}  // namespace wtw_fsm

#endif  // WTW_FSM_HPP
