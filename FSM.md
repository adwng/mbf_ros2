# Extracting a Generic FSM from rl_sar

This guide documents how to extract the finite state machine (FSM) from rl_sar, decouple
it from the RL-specific components, replace the custom YAML parameter reader with native
ROS 2 parameters, and write your own FSM states for a different controller backend
(e.g. CHAMP).

---

## Table of Contents

- [Part A: FSM Extraction](#part-a-fsm-extraction)
  - [A1. Architecture Overview](#a1-architecture-overview)
  - [A2. What to Keep Unchanged](#a2-what-to-keep-unchanged)
  - [A3. What to Replace](#a3-what-to-replace)
  - [A4. New Files to Create](#a4-new-files-to-create)
  - [A5. How to Write Your Own FSM States](#a5-how-to-write-your-own-fsm-states)
  - [A6. How to Write Your Controller Node](#a6-how-to-write-your-controller-node)
  - [A7. State Transition Reference](#a7-state-transition-reference)
- [Part B: YAML to ROS 2 Parameter Migration](#part-b-yaml-to-ros-2-parameter-migration)
  - [B1. How the Current YAML System Works](#b1-how-the-current-yaml-system-works)
  - [B2. Full Parameter Inventory](#b2-full-parameter-inventory)
  - [B3. The Replacement: `RosParams`](#b3-the-replacement-rosparams)
  - [B4. Where to Make Changes](#b4-where-to-make-changes)
  - [B5. Parameter YAML File Conversion](#b5-parameter-yaml-file-conversion)
- [Implementation Checklist](#implementation-checklist)

---

# Part A: FSM Extraction

## A1. Architecture Overview

rl_sar's FSM is built in three layers:

```
┌──────────────────────────────────────────────────────┐
│  Layer 3: Robot-Specific States                      │
│  fsm_robot/fsm_go2.hpp, fsm_a1.hpp, ...             │
│  (concrete Enter/Run/Exit/CheckChange per robot)     │
├──────────────────────────────────────────────────────┤
│  Layer 2: RL-Aware State Base                        │
│  library/core/rl_sdk/rl_sdk.hpp (.cpp)               │
│  (RLFSMState, RLControl, Interpolate, StateController│
├──────────────────────────────────────────────────────┤
│  Layer 1: Generic FSM Framework                      │
│  library/core/fsm/fsm.hpp                            │
│  (FSMState, FSM, FSMFactory, FSMManager)             │
└──────────────────────────────────────────────────────┘
```

**Layer 1** is entirely generic — no RL dependencies. It can be used as-is.

**Layer 2** couples states to the `RL` class (`rl.params`, `rl.control`, `rl.fsm`,
`rl.now_state`, etc.) and provides `Interpolate()` and `RLControl()`.

**Layer 3** defines four concrete states per robot (Passive, GetUp, GetDown,
RLLocomotion) plus a factory.

The goal is to keep Layer 1, replace Layer 2 with a controller-agnostic base, and write
new Layer 3 states for your application.

---

## A2. What to Keep Unchanged

### `library/core/fsm/fsm.hpp` — The Entire Generic FSM Core

This file has **zero RL dependencies**. Keep it entirely as-is. It contains:

| Class / Macro | Lines | Purpose |
|---|---|---|
| `FSMState` | 11–26 | Abstract base with `Enter()`, `Run()`, `Exit()`, `CheckChange()` |
| `FSM` | 28–108 | State machine runner: holds states map, runs NORMAL/CHANGE loop |
| `FSMFactory` | 110–118 | Abstract factory: `CreateState()`, `GetType()`, `GetSupportedStates()` |
| `FSMManager` | 120–182 | Singleton registry: `RegisterFactory()`, `CreateFSM()` by type string |
| `REGISTER_FSM_FACTORY` | 186–192 | Macro to register a factory at static init time |

The FSM run loop (called every control tick) works as:

```
NORMAL mode:
  current_state->Run()
  next = current_state->CheckChange()
  if next != current → set mode = CHANGE

CHANGE mode (next tick):
  current_state->Exit()
  swap to next_state
  next_state->Enter()
    (if Enter() called RequestStateChange(), defer to next tick)
  next_state->Run()
  mode = NORMAL
```

### `library/core/loop/loop.hpp` — Fixed-Rate Loop Utility

Not RL-specific. Provides `LoopFunc` for running callbacks at a fixed rate. Keep as-is.

---

## A3. What to Replace

### A3.1 `RLFSMState` — The RL-Coupled State Base

**File**: `rl_sdk.hpp` lines 264–284

```cpp
class RLFSMState : public FSMState
{
public:
    RLFSMState(RL& rl, const std::string& name)
        : FSMState(name), rl(rl), fsm_state(nullptr), fsm_command(nullptr) {}

    RL& rl;                              // ← couples to entire RL class
    const RobotState<float> *fsm_state;
    RobotCommand<float> *fsm_command;

    bool Interpolate(...);               // uses rl.params
    void RLControl();                    // pulls from rl.output_dof_pos_queue
};
```

**Problem**: Every state holds `RL& rl` which drags in the RL policy, observation buffer,
model inference, etc. The states access `rl.params`, `rl.control`, `rl.fsm`,
`rl.now_state`, `rl.start_state`, `rl.episode_length_buf`, etc.

**Replace with**: A new `RobotFSMState` base that holds only what the FSM needs:
params, control input, robot state/command, and the FSM reference (for
`RequestStateChange`). See [A4](#a4-new-files-to-create).

### A3.2 `RL::StateController()` — The FSM Executor

**File**: `rl_sdk.cpp` lines 8–62

```cpp
void RL::StateController(const RobotState<float>* state, RobotCommand<float>* command)
{
    // 1. Inject state/command pointers into all RLFSMState instances
    for (auto& pair : fsm.states_)
    {
        if (auto rl_fsm_state = std::dynamic_pointer_cast<RLFSMState>(pair.second))
        {
            rl_fsm_state->fsm_state = state;
            rl_fsm_state->fsm_command = command;
        }
    }

    // 2. Run the FSM (current state's Run + CheckChange)
    fsm.Run();

    // 3. Process keyboard velocity commands (WASD/QE/Space/N)
    // ...
}
```

**Replace with**: A similar function on your controller class, but casting to your new
`RobotFSMState` instead of `RLFSMState`. See [A6](#a6-how-to-write-your-controller-node).

### A3.3 `Interpolate()` — Joint Position Interpolation

**File**: `rl_sdk.cpp` lines 531–587

This is useful to keep for any controller — it smoothly moves joints from one pose to
another. The only RL dependency is `rl.params.Get<>()` for `dt`, `fixed_kp`, `fixed_kd`,
`rl_kp`, `rl_kd`, `num_of_dofs`.

**Replace**: Port to `RobotFSMState` and change `rl.params.Get<>()` to
`params.Get<>()`. See the implementation in [A4](#a4-new-files-to-create).

### A3.4 `RLControl()` — RL Policy Output Consumer

**File**: `rl_sdk.cpp` lines 589–608

Pops RL policy outputs from `tbb::concurrent_queue` and writes motor commands.

**Delete entirely** — your states will have their own control logic.

### A3.5 `RL::ReadYaml()` / `YamlParams` — Custom YAML Reader

**Replace with**: `RosParams` wrapping `rclcpp::Node`. See [Part B](#part-b-yaml-to-ros-2-parameter-migration).

### A3.6 Robot FSM Files (`fsm_robot/fsm_go2.hpp`, etc.)

Each defines four states (`Passive`, `GetUp`, `GetDown`, `RLLocomotion`) that inherit
from `RLFSMState` and access `rl.*` members extensively.

**Replace with**: Your own state files inheriting from your new `RobotFSMState`.
See [A5](#a5-how-to-write-your-own-fsm-states).

### A3.7 Members on `RL` That FSM States Access Directly

The concrete states in `fsm_robot/*.hpp` reach into the `RL` object for these members:

| Access Pattern | What It Is | Where to Put It |
|---|---|---|
| `rl.params.Get<T>(...)` | Parameter lookup | `RobotFSMState::params` → `RosParams` |
| `rl.control.current_keyboard` | Input state | `RobotFSMState::control` → `Control&` |
| `rl.control.current_gamepad` | Input state | `RobotFSMState::control` → `Control&` |
| `rl.fsm.previous_state_` | Previous FSM state | `RobotFSMState::fsm` → `FSM&` |
| `rl.fsm.RequestStateChange(...)` | Forced transition | `RobotFSMState::fsm` → `FSM&` |
| `rl.now_state` | Captured robot state | `RobotFSMState::now_state` → `RobotState<float>&` |
| `rl.start_state` | Startup robot state | `RobotFSMState::start_state` → `RobotState<float>&` |
| `rl.episode_length_buf` | Tick counter | Only needed for RL — drop or add to your controller |
| `rl.rl_init_done` | RL init flag | Only needed for RL — drop |
| `rl.config_name` | Policy config name | Only needed for RL — drop |
| `rl.robot_name` | Robot identifier | Can pass via factory context if needed |
| `rl.InitRL(...)` | Load RL model | Only needed for RL — replace with your controller init |

---

## A4. New Files to Create

### `robot_types.hpp` — Shared Data Types

Extract these data-only structs from `rl_sdk.hpp` into a standalone header:

```cpp
#pragma once
#include <vector>
#include <string>

template <typename T>
struct RobotCommand
{
    struct MotorCommand
    {
        std::vector<int> mode;
        std::vector<T> q;
        std::vector<T> dq;
        std::vector<T> tau;
        std::vector<T> kp;
        std::vector<T> kd;

        void resize(size_t num_joints)
        {
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
struct RobotState
{
    struct IMU
    {
        std::vector<T> quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
        std::vector<T> gyroscope = {0.0f, 0.0f, 0.0f};
        std::vector<T> accelerometer = {0.0f, 0.0f, 0.0f};
    } imu;

    struct MotorState
    {
        std::vector<T> q;
        std::vector<T> dq;
        std::vector<T> ddq;
        std::vector<T> tau_est;
        std::vector<T> cur;

        void resize(size_t num_joints)
        {
            q.resize(num_joints, 0.0f);
            dq.resize(num_joints, 0.0f);
            ddq.resize(num_joints, 0.0f);
            tau_est.resize(num_joints, 0.0f);
            cur.resize(num_joints, 0.0f);
        }
    } motor_state;
};

// Copy Input namespace and Control struct from rl_sdk.hpp lines 81–146
namespace Input { /* ... */ }
struct Control { /* ... */ };
```

These are the structs from `rl_sdk.hpp` lines 28–146. They have no RL dependency.

### `ros_params.hpp` — ROS 2 Parameter Wrapper

See [Part B, Section B3](#b3-the-replacement-rosparams) for the full implementation.

### `robot_fsm_state.hpp` — Generic State Base Class

This replaces `RLFSMState`. Instead of holding `RL& rl`, it holds individual references
to only what FSM states need:

```cpp
#pragma once
#include "fsm.hpp"
#include "robot_types.hpp"
#include "ros_params.hpp"
#include "logger.hpp"

struct RobotFSMContext
{
    RosParams& params;
    Control& control;
    FSM& fsm;
    RobotState<float>& now_state;
    RobotState<float>& start_state;
};

class RobotFSMState : public FSMState
{
public:
    RobotFSMState(RobotFSMContext& ctx, const std::string& name)
        : FSMState(name), ctx(ctx) {}

    RobotFSMContext& ctx;
    const RobotState<float>* fsm_state = nullptr;
    RobotCommand<float>* fsm_command = nullptr;

    // Ported from RLFSMState::Interpolate() in rl_sdk.cpp lines 531–587
    // Changed rl.params.Get<>() → ctx.params.Get<>()
    bool Interpolate(
        float& percent,
        const std::vector<float>& start_pos,
        const std::vector<float>& target_pos,
        float duration_seconds,
        const std::string& description = "",
        bool use_fixed_gains = true)
    {
        if (percent >= 1.0f)
            return false;

        if (percent == 0.0f)
        {
            float max_diff = 0.0f;
            for (size_t i = 0; i < start_pos.size() && i < target_pos.size(); ++i)
                max_diff = std::max(max_diff, std::abs(start_pos[i] - target_pos[i]));
            if (max_diff < 0.1f)
                percent = 1.0f;
        }

        int required_frames = std::max(1, static_cast<int>(
            std::ceil(duration_seconds / ctx.params.Get<float>("dt"))));
        float step = 1.0f / required_frames;
        percent += step;
        percent = std::min(percent, 1.0f);

        auto kp = use_fixed_gains
            ? ctx.params.Get<std::vector<float>>("fixed_kp")
            : ctx.params.Get<std::vector<float>>("rl_kp");
        auto kd = use_fixed_gains
            ? ctx.params.Get<std::vector<float>>("fixed_kd")
            : ctx.params.Get<std::vector<float>>("rl_kd");

        for (int i = 0; i < ctx.params.Get<int>("num_of_dofs"); ++i)
        {
            fsm_command->motor_command.q[i] = (1 - percent) * start_pos[i] + percent * target_pos[i];
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = kp[i];
            fsm_command->motor_command.kd[i] = kd[i];
            fsm_command->motor_command.tau[i] = 0;
        }

        if (!description.empty())
            LOGGER::PrintProgress(percent, description);

        return (percent < 1.0f);
    }
};
```

Key differences from `RLFSMState`:
- Holds `RobotFSMContext&` instead of `RL&`
- No `RLControl()` method
- `Interpolate()` uses `ctx.params` instead of `rl.params`
- States access control input via `ctx.control` instead of `rl.control`
- States access the FSM via `ctx.fsm` instead of `rl.fsm`

---

## A5. How to Write Your Own FSM States

Here's a complete example replacing go2's RL FSM with a generic one. Compare this
side-by-side with `fsm_robot/fsm_go2.hpp` to see what changed.

### Example: `fsm_robot/fsm_champ.hpp`

```cpp
#pragma once
#include "fsm.hpp"
#include "robot_fsm_state.hpp"

namespace champ_fsm {

// ─── State: Passive ─────────────────────────────────────────────
// Equivalent to go2_fsm::RLFSMStatePassive (fsm_go2.hpp lines 15–47)
// Changed: RLFSMState(*rl, ...) → RobotFSMState(ctx, ...)
//          rl.params  → ctx.params
//          rl.control → ctx.control
class Passive : public RobotFSMState
{
public:
    Passive(RobotFSMContext& ctx) : RobotFSMState(ctx, "Passive") {}

    void Enter() override
    {
        std::cout << LOGGER::NOTE
            << "Passive mode. Press '0' to stand up." << std::endl;
    }

    void Run() override
    {
        for (int i = 0; i < ctx.params.Get<int>("num_of_dofs"); ++i)
        {
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = 8;
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (ctx.control.current_keyboard == Input::Keyboard::Num0
            || ctx.control.current_gamepad == Input::Gamepad::A)
            return "Stand";
        return state_name_;
    }
};

// ─── State: Stand (replaces GetUp) ─────────────────────────────
// Equivalent to go2_fsm::RLFSMStateGetUp (fsm_go2.hpp lines 49–116)
// Changed: rl.now_state → ctx.now_state
//          rl.start_state → ctx.start_state
//          rl.fsm.previous_state_ → ctx.fsm.previous_state_
class Stand : public RobotFSMState
{
public:
    Stand(RobotFSMContext& ctx) : RobotFSMState(ctx, "Stand") {}

    float percent_getup = 0.0f;

    void Enter() override
    {
        percent_getup = 0.0f;
        ctx.now_state = *fsm_state;
        ctx.start_state = ctx.now_state;
    }

    void Run() override
    {
        Interpolate(percent_getup,
            ctx.now_state.motor_state.q,
            ctx.params.Get<std::vector<float>>("default_dof_pos"),
            2.0f, "Standing up", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (ctx.control.current_keyboard == Input::Keyboard::P
            || ctx.control.current_gamepad == Input::Gamepad::LB_X)
            return "Passive";

        if (percent_getup >= 1.0f)
        {
            if (ctx.control.current_keyboard == Input::Keyboard::Num1
                || ctx.control.current_gamepad == Input::Gamepad::RB_DPadUp)
                return "Walk";    // ← your controller state instead of RLLocomotion
            if (ctx.control.current_keyboard == Input::Keyboard::Num9
                || ctx.control.current_gamepad == Input::Gamepad::B)
                return "SitDown";
        }
        return state_name_;
    }
};

// ─── State: SitDown (replaces GetDown) ──────────────────────────
// Equivalent to go2_fsm::RLFSMStateGetDown (fsm_go2.hpp lines 118–150)
class SitDown : public RobotFSMState
{
public:
    SitDown(RobotFSMContext& ctx) : RobotFSMState(ctx, "SitDown") {}

    float percent_getdown = 0.0f;

    void Enter() override
    {
        percent_getdown = 0.0f;
        ctx.now_state = *fsm_state;
    }

    void Run() override
    {
        Interpolate(percent_getdown,
            ctx.now_state.motor_state.q,
            ctx.start_state.motor_state.q,
            2.0f, "Sitting down", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (ctx.control.current_keyboard == Input::Keyboard::P
            || ctx.control.current_gamepad == Input::Gamepad::LB_X
            || percent_getdown >= 1.0f)
            return "Passive";
        if (ctx.control.current_keyboard == Input::Keyboard::Num0
            || ctx.control.current_gamepad == Input::Gamepad::A)
            return "Stand";
        return state_name_;
    }
};

// ─── State: Walk (replaces RLLocomotion) ────────────────────────
// This is where you plug in CHAMP or any other controller.
// The original called rl.InitRL() in Enter() and RLControl() in Run().
// Replace with your controller's logic.
class Walk : public RobotFSMState
{
public:
    Walk(RobotFSMContext& ctx) : RobotFSMState(ctx, "Walk") {}

    void Enter() override
    {
        // Initialize your controller here
        // e.g. champ_controller.init(ctx.params);
    }

    void Run() override
    {
        // Call your controller here instead of RLControl()
        // e.g. auto cmd = champ_controller.compute(ctx.control.x, ctx.control.y, ctx.control.yaw);
        // Then write to fsm_command->motor_command.*
        std::cout << "\r\033[K" << std::flush << LOGGER::INFO
            << "Walk x:" << ctx.control.x
            << " y:" << ctx.control.y
            << " yaw:" << ctx.control.yaw << std::flush;
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (ctx.control.current_keyboard == Input::Keyboard::P
            || ctx.control.current_gamepad == Input::Gamepad::LB_X)
            return "Passive";
        if (ctx.control.current_keyboard == Input::Keyboard::Num9
            || ctx.control.current_gamepad == Input::Gamepad::B)
            return "SitDown";
        if (ctx.control.current_keyboard == Input::Keyboard::Num0
            || ctx.control.current_gamepad == Input::Gamepad::A)
            return "Stand";
        return state_name_;
    }
};

} // namespace champ_fsm

// ─── Factory ────────────────────────────────────────────────────
// Equivalent to Go2FSMFactory (fsm_go2.hpp lines 220–250)
// Changed: RL* context → RobotFSMContext* context
class ChampFSMFactory : public FSMFactory
{
public:
    ChampFSMFactory(const std::string& initial) : initial_state_(initial) {}

    std::shared_ptr<FSMState> CreateState(void* context, const std::string& state_name) override
    {
        auto* ctx = static_cast<RobotFSMContext*>(context);
        if (state_name == "Passive")  return std::make_shared<champ_fsm::Passive>(*ctx);
        if (state_name == "Stand")    return std::make_shared<champ_fsm::Stand>(*ctx);
        if (state_name == "SitDown")  return std::make_shared<champ_fsm::SitDown>(*ctx);
        if (state_name == "Walk")     return std::make_shared<champ_fsm::Walk>(*ctx);
        return nullptr;
    }

    std::string GetType() const override { return "champ_go2"; }

    std::vector<std::string> GetSupportedStates() const override
    {
        return {"Passive", "Stand", "SitDown", "Walk"};
    }

    std::string GetInitialState() const override { return initial_state_; }

private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(ChampFSMFactory, "Passive")
```

**Key mapping from original to new**:

| Original (fsm_go2.hpp) | New (fsm_champ.hpp) |
|---|---|
| `RLFSMState(*rl, "RLFSMStatePassive")` | `RobotFSMState(ctx, "Passive")` |
| `rl.params.Get<int>("num_of_dofs")` | `ctx.params.Get<int>("num_of_dofs")` |
| `rl.control.current_keyboard` | `ctx.control.current_keyboard` |
| `rl.now_state = *fsm_state` | `ctx.now_state = *fsm_state` |
| `rl.fsm.RequestStateChange(...)` | `ctx.fsm.RequestStateChange(...)` |
| `RLControl()` | Your controller call |
| `rl.InitRL(...)` | Your controller init |
| `RL* rl = static_cast<RL*>(context)` | `auto* ctx = static_cast<RobotFSMContext*>(context)` |

---

## A6. How to Write Your Controller Node

This replaces `rl_sim.cpp`. The original node inherits from `RL` and calls
`StateController` every tick. Your version creates a `RobotFSMContext` and runs the FSM
through a similar function.

### Equivalent of `RL::StateController()` (rl_sdk.cpp lines 8–62)

```cpp
void MyController::RunFSM(const RobotState<float>* state, RobotCommand<float>* command)
{
    // Step 1: Inject state/command into all FSM states (same pattern as rl_sdk.cpp lines 10–21)
    for (auto& [name, s] : fsm_.states_)
    {
        if (auto rs = std::dynamic_pointer_cast<RobotFSMState>(s))
        {
            rs->fsm_state = state;
            rs->fsm_command = command;
        }
    }

    // Step 2: Run FSM (same as rl_sdk.cpp line 23)
    fsm_.Run();

    // Step 3: Process velocity commands (same as rl_sdk.cpp lines 27–61)
    if (control_.current_keyboard == Input::Keyboard::W) control_.x += 0.1f;
    if (control_.current_keyboard == Input::Keyboard::S) control_.x -= 0.1f;
    if (control_.current_keyboard == Input::Keyboard::A) control_.y += 0.1f;
    if (control_.current_keyboard == Input::Keyboard::D) control_.y -= 0.1f;
    if (control_.current_keyboard == Input::Keyboard::Q) control_.yaw += 0.1f;
    if (control_.current_keyboard == Input::Keyboard::E) control_.yaw -= 0.1f;
    if (control_.current_keyboard == Input::Keyboard::Space)
    {
        control_.x = 0.0f; control_.y = 0.0f; control_.yaw = 0.0f;
    }
}
```

### Equivalent of Constructor (rl_sim.cpp lines 8–70)

```cpp
MyController::MyController()
{
    // Create ROS 2 node
    node_ = std::make_shared<rclcpp::Node>("robot_controller");

    // Initialize params from ROS 2 (replaces ReadYaml on line 56)
    params_.Init(node_);

    // Build context (replaces passing `this` as RL*)
    ctx_ = RobotFSMContext{params_, control_, fsm_, now_state_, start_state_};

    // Create FSM (same pattern as rl_sim.cpp lines 58–70)
    std::string robot_type = "champ_go2";  // or from a ROS 2 param
    if (FSMManager::GetInstance().IsTypeSupported(robot_type))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(robot_type, &ctx_);
        if (fsm_ptr) fsm_ = *fsm_ptr;
    }

    // Init joint sizes
    int num_dofs = params_.Get<int>("num_of_dofs");
    robot_state_.motor_state.resize(num_dofs);
    robot_command_.motor_command.resize(num_dofs);
    now_state_.motor_state.resize(num_dofs);
    start_state_.motor_state.resize(num_dofs);

    // Start control loop (same as rl_sim.cpp line 153)
    loop_control_ = std::make_shared<LoopFunc>(
        "loop_control", params_.Get<float>("dt"),
        std::bind(&MyController::RobotControl, this));
    loop_control_->start();
}
```

### Equivalent of `RobotControl()` (rl_sim.cpp lines 316–364)

```cpp
void MyController::RobotControl()
{
    GetState(&robot_state_);                               // read from ROS topics
    RunFSM(&robot_state_, &robot_command_);                // run FSM
    control_.ClearInput();                                 // clear one-shot inputs
    SetCommand(&robot_command_);                           // publish to ROS topics
}
```

---

## A7. State Transition Reference

The original rl_sar state graph:

```
                   Num0/A                    Num1/RB
  [Passive] ────────────────► [GetUp] ──────────────────► [RLLocomotion]
      ▲          P/LB_X         │  ▲        Num0/A              │
      │◄────────────────────────┘  └────────────────────────────┘
      │                                      Num9/B
      │                         ┌────────────────────────────────┘
      │          P/LB_X         ▼
      └──────────────────── [GetDown] ──► (auto on complete) ──► [Passive]
```

Suggested generic version:

```
                   Num0/A                  Num1/RB
  [Passive] ────────────────► [Stand] ──────────────► [Walk]
      ▲          P/LB_X         │  ▲      Num0/A        │
      │◄────────────────────────┘  └─────────────────────┘
      │                                    Num9/B
      │                       ┌──────────────────────────┘
      │         P/LB_X        ▼
      └──────────────────  [SitDown] ──► (auto) ──► [Passive]
```

---

# Part B: YAML to ROS 2 Parameter Migration

## B1. How the Current YAML System Works

### The `YamlParams` Struct

Defined in `rl_sdk.hpp` lines 148–170. Wraps `yaml-cpp`'s `YAML::Node`:

```cpp
struct YamlParams
{
    YAML::Node config_node;

    template<typename T>
    T Get(const std::string& key, const T& default_value = T()) const
    {
        if (config_node[key])
            return config_node[key].as<T>();
        return default_value;
    }

    bool Has(const std::string& key) const
    {
        return config_node[key].IsDefined();
    }
};
```

The `RL` class holds an instance on line 191: `YamlParams params;`

### The `ReadYaml()` Loader

Defined in `rl_sdk.cpp` lines 469–488. Reads a YAML file keyed by `file_path`, then
flattens all key-value pairs into `params.config_node`. Subsequent calls merge into the
same map (later loads overwrite overlapping keys).

### Where `ReadYaml()` Is Called

**Load 1 — at startup** (base robot params):

| File | Line | Call |
|------|------|------|
| `src/rl_sim.cpp` | 56 | `this->ReadYaml(this->robot_name, "base.yaml")` |
| `src/rl_sim_mujoco.cpp` | 85 | `this->ReadYaml(this->robot_name, "base.yaml")` |
| `src/rl_real_go2.cpp` | 26 | `this->ReadYaml(this->robot_name, "base.yaml")` |
| `src/rl_real_a1.cpp` | 24 | `this->ReadYaml(this->robot_name, "base.yaml")` |
| `src/rl_real_lite3.cpp` | 24 | `this->ReadYaml(this->robot_name, "base.yaml")` |
| `src/rl_real_g1.cpp` | 24 | `this->ReadYaml(this->robot_name, "base.yaml")` |
| `src/rl_real_l4w4.cpp` | 24 | `this->ReadYaml(this->robot_name, "base.yaml")` |

**Load 2 — when entering RLLocomotion** (RL policy params):

Called in `RL::InitRL()` at `rl_sdk.cpp` line 229:
`this->ReadYaml(robot_config_path, "config.yaml")`

Triggered by `RLFSMStateRLLocomotion::Enter()` in each robot's FSM file.

---

## B2. Full Parameter Inventory

### Parameters from `base.yaml` (keep these)

| Key | Type | Purpose |
|-----|------|---------|
| `dt` | `float` | Control loop timestep |
| `decimation` | `int` | Policy runs every N control steps |
| `num_of_dofs` | `int` | Number of joints |
| `fixed_kp` | `vector<float>` | PD gains for Interpolate (GetUp/GetDown) |
| `fixed_kd` | `vector<float>` | PD gains for Interpolate (GetUp/GetDown) |
| `default_dof_pos` | `vector<float>` | Standing joint positions |
| `joint_names` | `vector<string>` | ROS joint topic names |
| `joint_controller_names` | `vector<string>` | ROS controller names |
| `joint_mapping` | `vector<int>` | Reorder joints between policy and hardware |
| `torque_limits` | `vector<float>` | Safety limits |
| `wheel_indices` | `vector<int>` | Which joints are wheels |

### Parameters from `config.yaml` (RL-only, drop these)

`model_name`, `num_observations`, `observations`, `observations_history`,
`observations_history_priority`, `clip_obs`, `clip_actions_lower`, `clip_actions_upper`,
`rl_kp`, `rl_kd`, `action_scale`, `lin_vel_scale`, `ang_vel_scale`, `dof_pos_scale`,
`dof_vel_scale`, `commands_scale`

---

## B3. The Replacement: `RosParams`

### The Wrapper Class

Create `ros_params.hpp` with a class that provides the **same `Get<T>("key")` interface**
but reads from the ROS 2 parameter server:

```cpp
#pragma once
#include <rclcpp/rclcpp.hpp>

class RosParams
{
public:
    explicit RosParams(rclcpp::Node::SharedPtr node) : node_(node) {}
    RosParams() : node_(nullptr) {}

    void Init(rclcpp::Node::SharedPtr node) { node_ = node; }

    template<typename T>
    T Get(const std::string& key, const T& default_value = T()) const
    {
        if (!node_->has_parameter(key))
            node_->declare_parameter<T>(key, default_value);
        T value;
        node_->get_parameter(key, value);
        return value;
    }

    bool Has(const std::string& key) const
    {
        return node_ && node_->has_parameter(key);
    }

private:
    rclcpp::Node::SharedPtr node_;
};
```

### Handle `float` vs `double`

ROS 2 parameters only support `double`, but the codebase uses `float`. Add
specializations so all existing `params.Get<float>(...)` calls work without change:

```cpp
template<>
inline float RosParams::Get<float>(const std::string& key, const float& default_value) const
{
    return static_cast<float>(Get<double>(key, static_cast<double>(default_value)));
}

template<>
inline std::vector<float> RosParams::Get<std::vector<float>>(
    const std::string& key, const std::vector<float>& default_value) const
{
    std::vector<double> dv(default_value.begin(), default_value.end());
    auto result = Get<std::vector<double>>(key, dv);
    return std::vector<float>(result.begin(), result.end());
}
```

---

## B4. Where to Make Changes

### B4.1 `rl_sdk.hpp` — Replace `YamlParams` with `RosParams`

**Lines 148–170**: Delete the `YamlParams` struct.

**Line 191**: Change member type:

```cpp
// BEFORE
YamlParams params;

// AFTER
RosParams params;
```

**Line 222**: Delete `ReadYaml` declaration.

### B4.2 `rl_sdk.cpp` — Remove YAML Functions

**Lines 469–488**: Delete `RL::ReadYaml()` function body.

**Line 229**: Delete `this->ReadYaml(robot_config_path, "config.yaml")` inside `InitRL()`.

### B4.3 Node Constructors — Replace `ReadYaml` with `params.Init()`

In every node constructor, replace the YAML load with ROS 2 parameter init:

| File | Line to Delete | Line to Add |
|------|---------------|-------------|
| `src/rl_sim.cpp` | 56: `this->ReadYaml(this->robot_name, "base.yaml")` | `this->params.Init(ros2_node);` |
| `src/rl_sim_mujoco.cpp` | 85: `this->ReadYaml(this->robot_name, "base.yaml")` | `this->params.Init(ros2_node);` |
| `src/rl_real_go2.cpp` | 26: `this->ReadYaml(this->robot_name, "base.yaml")` | `this->params.Init(ros2_node);` |
| `src/rl_real_a1.cpp` | 24: `this->ReadYaml(this->robot_name, "base.yaml")` | `this->params.Init(ros2_node);` |
| `src/rl_real_lite3.cpp` | 24: `this->ReadYaml(this->robot_name, "base.yaml")` | `this->params.Init(ros2_node);` |
| `src/rl_real_g1.cpp` | 24: `this->ReadYaml(this->robot_name, "base.yaml")` | `this->params.Init(ros2_node);` |
| `src/rl_real_l4w4.cpp` | 24: `this->ReadYaml(this->robot_name, "base.yaml")` | `this->params.Init(ros2_node);` |

### B4.4 FSM State Files — No Changes Needed

States access params via `rl.params.Get<T>(...)`. Since `RosParams` exposes the exact
same `Get<T>()` interface, **no edits are needed** in any `fsm_robot/fsm_*.hpp` file.

### B4.5 `Interpolate()` and `RLControl()` — No Changes Needed

These access params via `rl.params.Get<T>(...)` — same interface, no edits.

---

## B5. Parameter YAML File Conversion

### Convert `base.yaml` to ROS 2 Format

**Before** (`policy/go2/base.yaml`):

```yaml
go2:
  dt: 0.005
  decimation: 4
  num_of_dofs: 12
  fixed_kp: [80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0]
  fixed_kd: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
  default_dof_pos: [0.00, 0.80, -1.50, 0.00, 0.80, -1.50, 0.00, 0.80, -1.50, 0.00, 0.80, -1.50]
  joint_names: ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", ...]
  joint_mapping: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
  torque_limits: [23.5, 23.5, 23.5, ...]
  wheel_indices: []
```

**After** (`config/go2_params.yaml`):

```yaml
/**:
  ros__parameters:
    dt: 0.005
    decimation: 4
    num_of_dofs: 12
    fixed_kp: [80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0]
    fixed_kd: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
    default_dof_pos: [0.00, 0.80, -1.50, 0.00, 0.80, -1.50, 0.00, 0.80, -1.50, 0.00, 0.80, -1.50]
    joint_names: ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", ...]
    joint_mapping: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
    torque_limits: [23.5, 23.5, 23.5, ...]
    wheel_indices: [0]  # ROS 2 cannot represent empty arrays; use sentinel or omit
```

### Load in Launch File

```python
Node(
    package='your_package',
    executable='robot_controller',
    name='robot_controller',
    parameters=['config/go2_params.yaml'],
)
```

### Empty Array Gotcha

ROS 2 parameters cannot represent empty arrays. For `wheel_indices: []`, either:

- **Option A**: Use a sentinel `[-1]` and filter in code
- **Option B**: Omit the key and handle `Has()` returning false

---

# Implementation Checklist

### FSM Extraction

```
[ ] Create robot_types.hpp: extract RobotCommand, RobotState, Input, Control from rl_sdk.hpp
[ ] Create ros_params.hpp: RosParams class with float/double specializations
[ ] Create robot_fsm_state.hpp: RobotFSMState with RobotFSMContext, ported Interpolate()
[ ] Write your FSM states (e.g. fsm_champ.hpp): Passive, Stand, SitDown, Walk
[ ] Write your FSM factory + REGISTER_FSM_FACTORY macro call
[ ] Write your controller node: RobotFSMContext setup, FSM creation, control loop
```

### YAML Migration

```
[ ] In rl_sdk.hpp: delete YamlParams struct (lines 148–170)
[ ] In rl_sdk.hpp: change params member type to RosParams (line 191)
[ ] In rl_sdk.hpp: delete ReadYaml declaration (line 222)
[ ] In rl_sdk.cpp: delete ReadYaml function body (lines 469–488)
[ ] In rl_sdk.cpp: delete ReadYaml call inside InitRL (line 229)
[ ] In each node constructor: replace ReadYaml(...) with params.Init(ros2_node)
[ ] Convert base.yaml files to ROS 2 parameter YAML format (ros__parameters:)
[ ] Update launch files to load the new parameter YAML
[ ] Remove yaml-cpp dependency from CMakeLists.txt / package.xml if no longer needed
```

### Cleanup (if fully replacing RL)

```
[ ] Delete rl_sdk.hpp and rl_sdk.cpp (replaced by robot_types + robot_fsm_state + ros_params)
[ ] Delete all fsm_robot/fsm_*.hpp files (replaced by your FSM states)
[ ] Delete policy/ directory (RL models and configs)
[ ] Delete rl_real_*.cpp / rl_real_*.hpp files
[ ] Delete rl_sim.cpp / rl_sim.hpp (replaced by your controller node)
```
