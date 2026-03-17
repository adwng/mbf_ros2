# 12-DOF Quadruped Robot Dog FSM Controller — Implementation Guide

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [FSM States](#fsm-states)
4. [Joint Interpolation Theory](#joint-interpolation-theory)
5. [Data Structures](#data-structures)
6. [Interpolation Functions](#interpolation-functions)
7. [FSM Implementation](#fsm-implementation)
8. [Gain Ramping](#gain-ramping)
9. [Safety Considerations](#safety-considerations)
10. [Tuning Guide](#tuning-guide)
11. [Full Reference Implementation](#full-reference-implementation)

---

## Overview

This document describes how to implement a finite state machine (FSM) controller for a
12-DOF quadruped robot dog with four states: **PASSIVE**, **STANDUP**, **LOCOMOTION**,
and **GETDOWN**. The critical challenge addressed here is ensuring smooth, safe joint
transitions between states — particularly when moving from a limp (zero-torque) passive
state to a standing pose, or back down again.

### The Problem

Naively commanding joints to jump from their current angles to a target pose in a single
control tick produces:

- Massive instantaneous torque demands
- Mechanical shock and jerk
- Potential hardware damage (gear teeth, belts, structural components)
- Loss of stability / the robot flinging itself

### The Solution

Use **time-based trajectory interpolation** with smooth (S-curve) profiles during
transition states. The STANDUP and GETDOWN states are not instant events — they are
states that persist for a configurable duration, during which joint commands are smoothly
interpolated from a captured start pose to a target pose.

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│                   Control Loop                       │
│                  (runs at N Hz)                       │
│                                                      │
│   ┌──────────┐    ┌──────────┐    ┌──────────────┐  │
│   │  Read     │───▶│   FSM    │───▶│  Publish     │  │
│   │  Sensors  │    │  Update  │    │  Commands    │  │
│   └──────────┘    └──────────┘    └──────────────┘  │
│                        │                             │
│              ┌─────────┴──────────┐                  │
│              │                    │                   │
│        ┌─────▼─────┐     ┌───────▼───────┐          │
│        │ Transition │     │  Locomotion   │          │
│        │ Interp.    │     │  Controller   │          │
│        └───────────┘     └───────────────┘          │
└─────────────────────────────────────────────────────┘
```

### Control Loop Frequency

A typical quadruped runs its low-level control loop at **500 Hz** (2 ms period).
The interpolation runs inside this loop, so each tick advances the interpolation by
`dt = 1.0 / ctrl_freq`.

### Joint Convention (12-DOF)

A standard 12-DOF quadruped has 3 joints per leg × 4 legs:

| Index | Joint               | Typical Range (rad) |
|-------|---------------------|---------------------|
| 0     | Front-Left Hip Abd  | \[-0.8, 0.8\]       |
| 1     | Front-Left Hip Flex | \[-0.5, 3.0\]       |
| 2     | Front-Left Knee     | \[-2.7, -0.5\]      |
| 3     | Front-Right Hip Abd | \[-0.8, 0.8\]       |
| 4     | Front-Right Hip Flex| \[-0.5, 3.0\]       |
| 5     | Front-Right Knee    | \[-2.7, -0.5\]      |
| 6     | Rear-Left Hip Abd   | \[-0.8, 0.8\]       |
| 7     | Rear-Left Hip Flex  | \[-0.5, 3.0\]       |
| 8     | Rear-Left Knee      | \[-2.7, -0.5\]      |
| 9     | Rear-Right Hip Abd  | \[-0.8, 0.8\]       |
| 10    | Rear-Right Hip Flex | \[-0.5, 3.0\]       |
| 11    | Rear-Right Knee     | \[-2.7, -0.5\]      |

> **Note:** Exact ranges and sign conventions depend on your specific robot (Unitree,
> MIT Mini Cheetah, custom, etc.). Always verify against your URDF / actuator specs.

---

## FSM States

```
                    user cmd
    ┌──────────┐  ──────────▶  ┌──────────┐
    │          │                │          │
    │ PASSIVE  │                │ STANDUP  │──── interpolation complete ───┐
    │ (limp)   │◀── abort ─────│ (interp) │                               │
    │          │                │          │                               ▼
    └──────────┘               └──────────┘                       ┌──────────────┐
         ▲                                                        │              │
         │                                                        │  LOCOMOTION  │
         │ interpolation                                          │  (gait ctrl) │
         │ complete                                               │              │
         │                                                        └──────┬───────┘
    ┌────┴─────┐                                                         │
    │          │◀──────────── user cmd ──────────────────────────────────┘
    │ GETDOWN  │
    │ (interp) │
    │          │
    └──────────┘
```

### State Descriptions

| State       | Duration    | What Happens                                                           |
|-------------|-------------|------------------------------------------------------------------------|
| PASSIVE     | Indefinite  | All joints at zero torque/gains. Robot is limp. Safe to physically manipulate. |
| STANDUP     | ~1–3 sec    | Joints interpolate from current pose to standing pose. PD gains ramp up. |
| LOCOMOTION  | Indefinite  | Gait controller / RL policy / MPC is active. Full PD gains.            |
| GETDOWN     | ~1–3 sec    | Joints interpolate from current pose to passive (folded) pose. PD gains ramp down. |

### Transition Rules

| From        | To          | Trigger                        | Action                                     |
|-------------|-------------|--------------------------------|--------------------------------------------|
| PASSIVE     | STANDUP     | User command (e.g. gamepad)    | Capture current q, set target = stand pose |
| STANDUP     | LOCOMOTION  | Interpolation timer complete   | Automatic                                  |
| STANDUP     | GETDOWN     | User abort mid-standup         | Capture current q, set target = passive pose |
| LOCOMOTION  | GETDOWN     | User command                   | Capture current q, set target = passive pose |
| GETDOWN     | PASSIVE     | Interpolation timer complete   | Automatic, zero out all gains/torques      |

---

## Joint Interpolation Theory

### Why Not Linear Interpolation?

Linear interpolation (`q(t) = q_start + (t/T) * (q_end - q_start)`) moves at constant
velocity. This means:

- At `t = 0`: velocity jumps from 0 to `(q_end - q_start) / T` **instantaneously**
- At `t = T`: velocity drops from that value to 0 **instantaneously**

These velocity discontinuities cause infinite jerk (in theory) and very high jerk (in
practice), leading to vibration, torque spikes, and mechanical stress.

### Cosine Interpolation

The simplest smooth alternative. Maps `[0, 1]` to `[0, 1]` with zero velocity at both
endpoints:

```
s(t) = 0.5 × (1 - cos(π × t / T))
```

**Properties:**
- `s(0) = 0`, `s(T) = 1`
- `s'(0) = 0`, `s'(T) = 0` (zero velocity at endpoints)
- Non-zero acceleration at endpoints

This is good enough for most practical applications.

### Quintic (5th-order) Polynomial — Minimum Snap

For even smoother motion, use a quintic polynomial that enforces zero velocity **and**
zero acceleration at both endpoints:

```
s(t) = 6τ⁵ - 15τ⁴ + 10τ³     where τ = t / T
```

**Properties:**
- `s(0) = 0`, `s(T) = 1`
- `s'(0) = 0`, `s'(T) = 0` (zero velocity)
- `s''(0) = 0`, `s''(T) = 0` (zero acceleration)

This is the recommended choice for hardware safety.

### Comparison Plot (Conceptual)

```
Position (s)
1.0 ┤                          ╭──────────
    │                        ╭─╯
    │                      ╭─╯
    │                   ╭──╯       ← quintic (smoothest)
    │                 ╭─╯
    │              ╭──╯
    │           ╭──╯
    │        ╭──╯
    │     ╭──╯
0.0 ┤─────╯
    └─────────────────────────────────────
    0                  T/2                T

Velocity (ds/dt)
    ┤         ╭────╮
    │        ╱      ╲
    │       ╱        ╲         ← quintic: bell curve
    │      ╱          ╲
    │     ╱            ╲
    │    ╱              ╲
    │   ╱                ╲
    │  ╱                  ╲
0.0 ┤─╯                    ╰──
    └─────────────────────────────────────
    0                  T/2                T
```

---

## Data Structures

### Header File (`quadruped_control.hpp`)

```cpp
#pragma once

#include <array>
#include <string>
#include <cmath>

static constexpr int NUM_JOINTS = 12;
static constexpr int NUM_LEGS = 4;
static constexpr int JOINTS_PER_LEG = 3;

enum class FSMState {
    PASSIVE,
    STANDUP,
    LOCOMOTION,
    GETDOWN
};

struct JointState {
    std::array<double, NUM_JOINTS> q{};       // position (rad)
    std::array<double, NUM_JOINTS> dq{};      // velocity (rad/s)
    std::array<double, NUM_JOINTS> tau_est{};  // estimated torque (Nm)
};

struct JointCommand {
    std::array<double, NUM_JOINTS> q{};    // desired position
    std::array<double, NUM_JOINTS> dq{};   // desired velocity
    std::array<double, NUM_JOINTS> kp{};   // proportional gain
    std::array<double, NUM_JOINTS> kd{};   // derivative gain
    std::array<double, NUM_JOINTS> tau{};  // feedforward torque
};

// Each actuator executes: tau = kp * (q_des - q) + kd * (dq_des - dq) + tau_ff

struct TransitionConfig {
    double standup_duration = 2.0;   // seconds
    double getdown_duration = 2.0;   // seconds
    double standup_kp = 60.0;        // target PD gains at full stand
    double standup_kd = 5.0;
    double locomotion_kp = 60.0;
    double locomotion_kd = 5.0;
};

struct Poses {
    std::array<double, NUM_JOINTS> passive;   // folded / rest pose
    std::array<double, NUM_JOINTS> standing;  // standing pose
};

class QuadrupedControl {
public:
    void loop();

private:
    // FSM
    FSMState fsm_state_ = FSMState::PASSIVE;
    std::string state_string_ = "PASSIVE";

    // Joint data
    JointState joint_state_;
    JointCommand joint_cmd_;

    // Transition interpolation
    std::array<double, NUM_JOINTS> interp_start_q_{};
    std::array<double, NUM_JOINTS> interp_target_q_{};
    double interp_duration_ = 0.0;
    double interp_elapsed_ = 0.0;
    bool in_transition_ = false;

    // Config
    TransitionConfig config_;
    Poses poses_;
    double ctrl_freq_ = 500.0;  // Hz

    // Methods
    void begin_transition(const std::array<double, NUM_JOINTS>& target,
                          double duration);
    void run_interpolation(double dt);
    double smooth_ratio(double t, double T);
    void set_passive_commands();
    void compute_locomotion();
    bool user_requests_standup();
    bool user_requests_getdown();
    bool user_requests_abort();
    void publish_commands();
};
```

---

## Interpolation Functions

### Implementation

```cpp
// Cosine interpolation — zero velocity at endpoints
double QuadrupedControl::smooth_ratio(double t, double T) {
    double s = std::clamp(t / T, 0.0, 1.0);
    return 0.5 * (1.0 - std::cos(s * M_PI));
}
```

Or, for the quintic version:

```cpp
double QuadrupedControl::smooth_ratio(double t, double T) {
    double s = std::clamp(t / T, 0.0, 1.0);
    // 6s^5 - 15s^4 + 10s^3: zero velocity and zero acceleration at endpoints
    return s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
}
```

### `begin_transition`

Called once at the moment of state change. Captures where all joints currently are.

```cpp
void QuadrupedControl::begin_transition(
        const std::array<double, NUM_JOINTS>& target,
        double duration) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        interp_start_q_[i] = joint_state_.q[i];  // snapshot actual position
    }
    interp_target_q_ = target;
    interp_duration_ = duration;
    interp_elapsed_ = 0.0;
    in_transition_ = true;
}
```

> **Critical:** Always read from `joint_state_.q` (the measured/actual position), not
> from a previously commanded position. This ensures correctness even if the robot
> drifted, was physically moved, or a previous transition was interrupted.

### `run_interpolation`

Called every control tick while a transition is active. Advances the timer and computes
the interpolated joint commands.

```cpp
void QuadrupedControl::run_interpolation(double dt) {
    interp_elapsed_ += dt;
    double ratio = smooth_ratio(interp_elapsed_, interp_duration_);

    for (int i = 0; i < NUM_JOINTS; i++) {
        double q_des = interp_start_q_[i]
                     + ratio * (interp_target_q_[i] - interp_start_q_[i]);

        joint_cmd_.q[i]   = q_des;
        joint_cmd_.dq[i]  = 0.0;
        joint_cmd_.kp[i]  = ratio * config_.standup_kp;
        joint_cmd_.kd[i]  = ratio * config_.standup_kd;
        joint_cmd_.tau[i] = 0.0;
    }
}
```

---

## FSM Implementation

### Main Loop

```cpp
void QuadrupedControl::loop() {
    double dt = 1.0 / ctrl_freq_;

    switch (fsm_state_) {

    // ──────────────────────────────────────────
    // PASSIVE: robot is limp, no torques applied
    // ──────────────────────────────────────────
    case FSMState::PASSIVE:
        state_string_ = "PASSIVE";
        set_passive_commands();

        if (user_requests_standup()) {
            begin_transition(poses_.standing, config_.standup_duration);
            fsm_state_ = FSMState::STANDUP;
        }
        break;

    // ──────────────────────────────────────────
    // STANDUP: interpolating from current pose to standing pose
    // ──────────────────────────────────────────
    case FSMState::STANDUP:
        state_string_ = "STANDUP";
        run_interpolation(dt);

        if (interp_elapsed_ >= interp_duration_) {
            in_transition_ = false;
            fsm_state_ = FSMState::LOCOMOTION;
        }

        if (user_requests_abort()) {
            begin_transition(poses_.passive, config_.getdown_duration);
            fsm_state_ = FSMState::GETDOWN;
        }
        break;

    // ──────────────────────────────────────────
    // LOCOMOTION: gait controller is active
    // ──────────────────────────────────────────
    case FSMState::LOCOMOTION:
        state_string_ = "LOCOMOTION";
        compute_locomotion();

        if (user_requests_getdown()) {
            begin_transition(poses_.passive, config_.getdown_duration);
            fsm_state_ = FSMState::GETDOWN;
        }
        break;

    // ──────────────────────────────────────────
    // GETDOWN: interpolating from current pose to passive pose
    // ──────────────────────────────────────────
    case FSMState::GETDOWN:
        state_string_ = "GETDOWN";
        run_interpolation(dt);

        if (interp_elapsed_ >= interp_duration_) {
            in_transition_ = false;
            fsm_state_ = FSMState::PASSIVE;
        }
        break;
    }

    publish_commands();
}
```

### Passive Command Helper

```cpp
void QuadrupedControl::set_passive_commands() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        joint_cmd_.q[i]   = 0.0;
        joint_cmd_.dq[i]  = 0.0;
        joint_cmd_.kp[i]  = 0.0;
        joint_cmd_.kd[i]  = 0.0;
        joint_cmd_.tau[i] = 0.0;
    }
}
```

---

## Gain Ramping

This is a critical detail that is often overlooked. During STANDUP, the PD gains must
be ramped alongside the position trajectory.

### Why?

Consider what happens at `t = 0` of the STANDUP transition if you apply full gains
instantly:

```
tau = kp * (q_des - q_actual) + kd * (dq_des - dq_actual)
```

Even though `q_des ≈ q_actual` (the interpolation just started), there will be some
small error, and with a large `kp`, this creates a noticeable jerk. More importantly,
the robot's joints may not be exactly at the captured start position by the time the
first command executes (communication latency, etc.).

### Implementation

Gain ramping is already built into the `run_interpolation` function above:

```cpp
joint_cmd_.kp[i] = ratio * config_.standup_kp;
joint_cmd_.kd[i] = ratio * config_.standup_kd;
```

- At `t = 0`: `ratio = 0` → gains are zero (still limp)
- At `t = T/2`: `ratio ≈ 0.5` → gains at half strength
- At `t = T`: `ratio = 1.0` → gains at full strength

### GETDOWN Gain Ramping (Reversed)

For GETDOWN, you want gains to decrease as the robot settles into the passive pose.
Modify `run_interpolation` to accept a direction parameter, or handle it separately:

```cpp
void QuadrupedControl::run_interpolation_getdown(double dt) {
    interp_elapsed_ += dt;
    double ratio = smooth_ratio(interp_elapsed_, interp_duration_);

    for (int i = 0; i < NUM_JOINTS; i++) {
        double q_des = interp_start_q_[i]
                     + ratio * (interp_target_q_[i] - interp_start_q_[i]);

        joint_cmd_.q[i]   = q_des;
        joint_cmd_.dq[i]  = 0.0;
        // Gains decrease as we approach the passive pose
        joint_cmd_.kp[i]  = (1.0 - ratio) * config_.standup_kp;
        joint_cmd_.kd[i]  = (1.0 - ratio) * config_.standup_kd;
        joint_cmd_.tau[i] = 0.0;
    }
}
```

Alternatively, unify with a flag:

```cpp
enum class TransitionDirection { RISING, FALLING };

void QuadrupedControl::run_interpolation(double dt, TransitionDirection dir) {
    interp_elapsed_ += dt;
    double ratio = smooth_ratio(interp_elapsed_, interp_duration_);

    double gain_scale = (dir == TransitionDirection::RISING) ? ratio : (1.0 - ratio);

    for (int i = 0; i < NUM_JOINTS; i++) {
        double q_des = interp_start_q_[i]
                     + ratio * (interp_target_q_[i] - interp_start_q_[i]);

        joint_cmd_.q[i]   = q_des;
        joint_cmd_.dq[i]  = 0.0;
        joint_cmd_.kp[i]  = gain_scale * config_.standup_kp;
        joint_cmd_.kd[i]  = gain_scale * config_.standup_kd;
        joint_cmd_.tau[i] = 0.0;
    }
}
```

Then call with:
- STANDUP: `run_interpolation(dt, TransitionDirection::RISING)`
- GETDOWN: `run_interpolation(dt, TransitionDirection::FALLING)`

---

## Safety Considerations

### 1. Joint Limit Clamping

Always clamp commanded positions to the actuator's physical limits:

```cpp
static constexpr std::array<double, NUM_JOINTS> Q_MIN = {
    -0.8, -0.5, -2.7,  // FL
    -0.8, -0.5, -2.7,  // FR
    -0.8, -0.5, -2.7,  // RL
    -0.8, -0.5, -2.7   // RR
};

static constexpr std::array<double, NUM_JOINTS> Q_MAX = {
     0.8,  3.0, -0.5,
     0.8,  3.0, -0.5,
     0.8,  3.0, -0.5,
     0.8,  3.0, -0.5
};

// Inside run_interpolation, after computing q_des:
q_des = std::clamp(q_des, Q_MIN[i], Q_MAX[i]);
```

### 2. Torque Limits

Clamp the final computed torque before sending to actuators:

```cpp
static constexpr double MAX_TORQUE = 33.5;  // Nm, depends on your motors

// In publish_commands():
for (int i = 0; i < NUM_JOINTS; i++) {
    joint_cmd_.tau[i] = std::clamp(joint_cmd_.tau[i], -MAX_TORQUE, MAX_TORQUE);
}
```

### 3. IMU-Based Fall Detection

If the robot falls during STANDUP or LOCOMOTION, immediately transition to PASSIVE:

```cpp
bool QuadrupedControl::check_safety() {
    double roll_deg  = rad2deg(imu_.roll);
    double pitch_deg = rad2deg(imu_.pitch);

    if (std::fabs(roll_deg) > 60.0 || std::fabs(pitch_deg) > 60.0) {
        return false;  // fallen over
    }
    return true;
}
```

Integrate into the FSM:

```cpp
case FSMState::LOCOMOTION:
    if (!check_safety()) {
        set_passive_commands();         // immediately go limp
        fsm_state_ = FSMState::PASSIVE; // skip GETDOWN — robot is fallen
        break;
    }
    // ... normal locomotion ...
    break;
```

### 4. Communication Timeout

If you haven't received fresh joint state data in N ms, go to PASSIVE:

```cpp
if (time_since_last_joint_state > 0.1) {  // 100 ms timeout
    set_passive_commands();
    fsm_state_ = FSMState::PASSIVE;
}
```

---

## Tuning Guide

### Transition Duration

| Parameter          | Safe Starting Value | Notes                                     |
|--------------------|--------------------|--------------------------------------------|
| `standup_duration` | 2.0 s              | Longer = safer but slower. Shorten once validated. |
| `getdown_duration` | 2.0 s              | Can be slightly faster since gravity assists.      |

### PD Gains

| Parameter    | Starting Value | Notes                                               |
|-------------|----------------|------------------------------------------------------|
| `standup_kp`| 40–80          | Higher = stiffer tracking. Start low.                |
| `standup_kd`| 3–8            | Higher = more damping. Prevents oscillation.         |

**Tuning procedure:**

1. Start with low `kp` (e.g. 20) and moderate `kd` (e.g. 5)
2. Command STANDUP. Observe if joints reach target smoothly.
3. If joints are sluggish / don't reach target: increase `kp`
4. If joints oscillate around target: increase `kd` or decrease `kp`
5. If robot shakes / vibrates: decrease both gains
6. Once STANDUP is smooth, verify GETDOWN with same process

### Interpolation Profile

- **Cosine**: Good default. Simple. Zero velocity at endpoints.
- **Quintic**: Better for aggressive (fast) transitions where acceleration matters.
- For transitions of 1.5s+ at typical quadruped speeds, cosine is sufficient.

### Pose Definition

Finding good standing and passive poses:

1. **Standing pose**: Command the robot in simulation (Gazebo/MuJoCo/Isaac) to a
   nominal stand. Read the joint angles. These are your standing pose values.
2. **Passive pose**: Physically fold the robot's legs into a compact rest position.
   Read the joint encoders. These are your passive pose values.
3. Alternatively, use the URDF's default joint positions as a starting point and
   adjust from there.

---

## Full Reference Implementation

Below is a complete, self-contained implementation file. Adapt the ROS 2 interfaces,
message types, and topic names to match your robot's SDK.

```cpp
#include "quadruped_control/quadruped_control.hpp"
#include <cmath>
#include <algorithm>

// ─── Pose Definitions ──────────────────────────────────────────

static const std::array<double, NUM_JOINTS> PASSIVE_POSE = {
    // FL: abd, hip, knee
    0.0,  1.25, -2.5,
    // FR
    0.0,  1.25, -2.5,
    // RL
    0.0,  1.25, -2.5,
    // RR
    0.0,  1.25, -2.5
};

static const std::array<double, NUM_JOINTS> STANDING_POSE = {
    0.0,  0.67, -1.3,
    0.0,  0.67, -1.3,
    0.0,  0.67, -1.3,
    0.0,  0.67, -1.3
};

// ─── Constructor ────────────────────────────────────────────────

QuadrupedControl::QuadrupedControl() {
    poses_.passive  = PASSIVE_POSE;
    poses_.standing = STANDING_POSE;
    fsm_state_ = FSMState::PASSIVE;
}

// ─── Interpolation ─────────────────────────────────────────────

double QuadrupedControl::smooth_ratio(double t, double T) {
    double s = std::clamp(t / T, 0.0, 1.0);
    // Quintic smooth-step: zero velocity and acceleration at endpoints
    return s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
}

void QuadrupedControl::begin_transition(
        const std::array<double, NUM_JOINTS>& target,
        double duration) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        interp_start_q_[i] = joint_state_.q[i];
    }
    interp_target_q_ = target;
    interp_duration_  = duration;
    interp_elapsed_   = 0.0;
    in_transition_    = true;
}

void QuadrupedControl::run_interpolation(double dt, TransitionDirection dir) {
    interp_elapsed_ += dt;
    double ratio = smooth_ratio(interp_elapsed_, interp_duration_);
    double gain_scale = (dir == TransitionDirection::RISING)
                            ? ratio
                            : (1.0 - ratio);

    for (int i = 0; i < NUM_JOINTS; i++) {
        double q_des = interp_start_q_[i]
                     + ratio * (interp_target_q_[i] - interp_start_q_[i]);

        joint_cmd_.q[i]   = q_des;
        joint_cmd_.dq[i]  = 0.0;
        joint_cmd_.kp[i]  = gain_scale * config_.standup_kp;
        joint_cmd_.kd[i]  = gain_scale * config_.standup_kd;
        joint_cmd_.tau[i] = 0.0;
    }
}

// ─── Passive ────────────────────────────────────────────────────

void QuadrupedControl::set_passive_commands() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        joint_cmd_.q[i]   = 0.0;
        joint_cmd_.dq[i]  = 0.0;
        joint_cmd_.kp[i]  = 0.0;
        joint_cmd_.kd[i]  = 0.0;
        joint_cmd_.tau[i] = 0.0;
    }
}

// ─── Main Loop ──────────────────────────────────────────────────

void QuadrupedControl::loop() {
    double dt = 1.0 / ctrl_freq_;

    switch (fsm_state_) {

    case FSMState::PASSIVE:
        state_string_ = "PASSIVE";
        set_passive_commands();

        if (user_requests_standup()) {
            begin_transition(poses_.standing, config_.standup_duration);
            fsm_state_ = FSMState::STANDUP;
        }
        break;

    case FSMState::STANDUP:
        state_string_ = "STANDUP";
        run_interpolation(dt, TransitionDirection::RISING);

        if (interp_elapsed_ >= interp_duration_) {
            in_transition_ = false;
            fsm_state_ = FSMState::LOCOMOTION;
        }

        if (user_requests_abort()) {
            begin_transition(poses_.passive, config_.getdown_duration);
            fsm_state_ = FSMState::GETDOWN;
        }
        break;

    case FSMState::LOCOMOTION:
        state_string_ = "LOCOMOTION";

        if (!check_safety()) {
            set_passive_commands();
            fsm_state_ = FSMState::PASSIVE;
            break;
        }

        compute_locomotion();

        if (user_requests_getdown()) {
            begin_transition(poses_.passive, config_.getdown_duration);
            fsm_state_ = FSMState::GETDOWN;
        }
        break;

    case FSMState::GETDOWN:
        state_string_ = "GETDOWN";
        run_interpolation(dt, TransitionDirection::FALLING);

        if (interp_elapsed_ >= interp_duration_) {
            in_transition_ = false;
            fsm_state_ = FSMState::PASSIVE;
        }
        break;
    }

    publish_commands();
}
```

---

## Summary Checklist

- [ ] Define `PASSIVE_POSE` and `STANDING_POSE` from your robot's URDF / simulation
- [ ] Implement `smooth_ratio` (cosine or quintic)
- [ ] `begin_transition` captures actual joint positions at moment of state change
- [ ] `run_interpolation` advances timer, computes smooth `q_des`, ramps PD gains
- [ ] STANDUP uses `TransitionDirection::RISING` (gains ramp up)
- [ ] GETDOWN uses `TransitionDirection::FALLING` (gains ramp down)
- [ ] Joint limits are clamped before publishing
- [ ] Torque limits are clamped before publishing
- [ ] Fall detection triggers immediate PASSIVE (skip GETDOWN)
- [ ] Communication timeout triggers PASSIVE
- [ ] Transition can be interrupted mid-way (re-calling `begin_transition` from current position)
- [ ] Tune `standup_duration`, `kp`, `kd` on hardware starting with conservative values
