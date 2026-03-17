# Replacing rl_sar's YAML Parameter System with ROS 2 Native Parameters

This guide explains how the current custom YAML parameter reader works in rl_sar
and how to replace it with ROS 2's native `get_parameter` / `declare_parameter` system.

---

## 1. How the Current System Works

### 1.1 The `YamlParams` Struct

Defined in `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp` (lines 148–170):

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

This is a thin wrapper around `yaml-cpp`'s `YAML::Node`. Every parameter in the entire
codebase is accessed through a single call pattern: `params.Get<T>("key")`.

The `RL` class holds an instance as a member (`rl_sdk.hpp` line 191):

```cpp
class RL
{
    YamlParams params;
    // ...
};
```

### 1.2 The `ReadYaml()` Loader

Defined in `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp` (lines 469–488):

```cpp
void RL::ReadYaml(const std::string& file_path, const std::string& file_name)
{
    std::string config_path = std::string(POLICY_DIR) + "/" + file_path + "/" + file_name;
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path)[file_path];
    }
    catch (YAML::BadFile &e)
    {
        std::cout << LOGGER::ERROR << "The file '" << config_path << "' does not exist" << std::endl;
        return;
    }

    for (auto it = config.begin(); it != config.end(); ++it)
    {
        std::string key = it->first.as<std::string>();
        this->params.config_node[key] = it->second;
    }
}
```

It reads a YAML file keyed by `file_path`, then flattens all key-value pairs into the
single `params.config_node` map. Subsequent calls **merge** into the same map, so later
loads overwrite earlier ones for the same keys.

### 1.3 Where `ReadYaml()` Is Called

There are exactly **two loading points** — one at startup, one when entering RL locomotion:

#### Load 1: At startup (base robot params)

Called in every node constructor. Examples:

| File | Line | Call |
|------|------|------|
| `src/rl_sim.cpp` | 56 | `this->ReadYaml(this->robot_name, "base.yaml")` |
| `src/rl_sim_mujoco.cpp` | 85 | `this->ReadYaml(this->robot_name, "base.yaml")` |
| `src/rl_real_go2.cpp` | 26 | `this->ReadYaml(this->robot_name, "base.yaml")` |
| `src/rl_real_a1.cpp` | 24 | `this->ReadYaml(this->robot_name, "base.yaml")` |
| `src/rl_real_lite3.cpp` | 24 | `this->ReadYaml(this->robot_name, "base.yaml")` |
| `src/rl_real_g1.cpp` | 24 | `this->ReadYaml(this->robot_name, "base.yaml")` |
| `src/rl_real_l4w4.cpp` | 24 | `this->ReadYaml(this->robot_name, "base.yaml")` |

These load files like `policy/go2/base.yaml` containing hardware constants (dt, PD gains,
joint names, etc.).

#### Load 2: When entering RLLocomotion (policy-specific params)

Called inside `RL::InitRL()` at `rl_sdk.cpp` line 229:

```cpp
void RL::InitRL(std::string robot_config_path)
{
    this->ReadYaml(robot_config_path, "config.yaml");  // e.g. "go2/himloco"
    // ...
}
```

This is triggered by `RLFSMStateRLLocomotion::Enter()` in each robot's FSM file (e.g.
`fsm_go2.hpp` line 169). It loads `policy/go2/himloco/config.yaml` containing
RL-specific params (model name, observation config, action scales, etc.).

**Key behavior**: Because both loads go into the same `params.config_node`, the
`config.yaml` values overwrite `base.yaml` values for overlapping keys (like `rl_kp`,
`fixed_kp`, `num_of_dofs`, etc.).

### 1.4 Where `params.Get<T>()` Is Used

The `params.Get<T>("key")` pattern is used in three locations:

1. **`rl_sdk.cpp`** — core RL logic (`ComputeObservation`, `ComputeOutput`, `InitRL`,
   `Interpolate`, `RLControl`, `TorqueProtect`, `CSVLogger`, etc.)
2. **`fsm_robot/fsm_*.hpp`** — FSM states access params via `rl.params.Get<T>(...)` for
   things like `num_of_dofs`, `default_dof_pos`
3. **`rl_sim.cpp` / `rl_real_*.cpp`** — node constructors use params for joint count,
   loop timing, joint names, etc.

---

## 2. Full Parameter Inventory

### Parameters from `base.yaml` (robot hardware)

| Key | Type | Purpose |
|-----|------|---------|
| `dt` | `float` | Control loop timestep |
| `decimation` | `int` | Policy runs every `decimation` control steps |
| `num_of_dofs` | `int` | Number of joints |
| `fixed_kp` | `vector<float>` | PD gains for interpolation (GetUp/GetDown) |
| `fixed_kd` | `vector<float>` | PD gains for interpolation (GetUp/GetDown) |
| `default_dof_pos` | `vector<float>` | Standing joint positions |
| `joint_names` | `vector<string>` | ROS joint topic names |
| `joint_controller_names` | `vector<string>` | ROS controller names |
| `joint_mapping` | `vector<int>` | Reorder joints between policy and hardware order |
| `torque_limits` | `vector<float>` | Safety limits |
| `wheel_indices` | `vector<int>` | Which joints are wheels (empty for legged) |

### Parameters from `config.yaml` (RL policy — drop these for non-RL use)

| Key | Type |
|-----|------|
| `model_name` | `string` |
| `num_observations` | `int` |
| `observations` | `vector<string>` |
| `observations_history` | `vector<int>` |
| `observations_history_priority` | `string` |
| `clip_obs` | `float` |
| `clip_actions_lower` | `vector<float>` |
| `clip_actions_upper` | `vector<float>` |
| `rl_kp` | `vector<float>` |
| `rl_kd` | `vector<float>` |
| `action_scale` | `vector<float>` |
| `lin_vel_scale` | `float` |
| `ang_vel_scale` | `float` |
| `dof_pos_scale` | `float` |
| `dof_vel_scale` | `float` |
| `commands_scale` | `vector<float>` |

---

## 3. The Replacement: `RosParams`

### 3.1 Create the Wrapper

Create a new header (e.g. `ros_params.hpp`) with a class that provides the **same
`Get<T>("key")` interface** but reads from the ROS 2 parameter server:

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

### 3.2 Handle the `float` vs `double` Problem

ROS 2 parameters only support `double`, not `float`. The rl_sar codebase calls
`params.Get<float>(...)` and `params.Get<std::vector<float>>(...)` everywhere.
Add explicit template specializations:

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

This makes every existing `params.Get<float>("dt")` and
`params.Get<std::vector<float>>("fixed_kp")` call work without modification.

---

## 4. Where to Make Changes

### 4.1 `rl_sdk.hpp` — Replace `YamlParams` with `RosParams`

**Location**: Lines 148–170 (struct definition) and line 191 (member declaration)

**Change**: Replace the `YamlParams` struct with `RosParams` (or include your new header
and change the member type):

```cpp
// BEFORE (line 191)
YamlParams params;

// AFTER
RosParams params;
```

Then remove the `YamlParams` struct definition (lines 148–170) entirely.

### 4.2 `rl_sdk.hpp` — Remove `ReadYaml` Declaration

**Location**: Line 222

```cpp
// DELETE this line:
void ReadYaml(const std::string& file_path, const std::string& file_name);
```

### 4.3 `rl_sdk.cpp` — Remove `ReadYaml` Implementation

**Location**: Lines 469–488

Delete the entire `RL::ReadYaml()` function body.

### 4.4 `rl_sdk.cpp` — Remove `ReadYaml` Call in `InitRL()`

**Location**: Line 229

```cpp
// BEFORE
void RL::InitRL(std::string robot_config_path)
{
    std::lock_guard<std::mutex> lock(this->model_mutex);
    this->ReadYaml(robot_config_path, "config.yaml");  // DELETE this line
    // ...
}
```

If you're keeping `InitRL` for your use case, just remove the `ReadYaml` call. If you're
replacing RL with a different controller, you'll rewrite this function entirely.

### 4.5 Node Constructors — Remove `ReadYaml` Call, Initialize `RosParams`

In every node constructor, replace the YAML load with `RosParams` initialization.

**Example: `rl_sim.cpp` (lines 15–56)**

```cpp
// BEFORE
#elif defined(USE_ROS2)
    ros2_node = std::make_shared<rclcpp::Node>("rl_sim_node");
    // ... param_client setup to get robot_name ...

    // read params from yaml
    this->ReadYaml(this->robot_name, "base.yaml");  // DELETE

// AFTER
#elif defined(USE_ROS2)
    ros2_node = std::make_shared<rclcpp::Node>("rl_sim_node");
    // ... param_client setup to get robot_name ...

    // Initialize params from ROS 2 parameter server
    this->params.Init(ros2_node);
```

**Apply the same pattern to all node constructors:**

| File | Line to remove | Line to add |
|------|---------------|-------------|
| `src/rl_sim.cpp` | Line 56: `this->ReadYaml(...)` | `this->params.Init(ros2_node);` |
| `src/rl_sim_mujoco.cpp` | Line 85: `this->ReadYaml(...)` | `this->params.Init(ros2_node);` |
| `src/rl_real_go2.cpp` | Line 26: `this->ReadYaml(...)` | `this->params.Init(ros2_node);` |
| `src/rl_real_a1.cpp` | Line 24: `this->ReadYaml(...)` | `this->params.Init(ros2_node);` |
| `src/rl_real_lite3.cpp` | Line 24: `this->ReadYaml(...)` | `this->params.Init(ros2_node);` |
| `src/rl_real_g1.cpp` | Line 24: `this->ReadYaml(...)` | `this->params.Init(ros2_node);` |
| `src/rl_real_l4w4.cpp` | Line 24: `this->ReadYaml(...)` | `this->params.Init(ros2_node);` |

### 4.6 FSM State Files — No Changes Needed

The FSM states (e.g. `fsm_go2.hpp`) access params through `rl.params.Get<T>(...)`.
Since `RosParams` exposes the exact same `Get<T>()` interface, **no changes are needed
in any FSM state file**. The calls like:

```cpp
for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
```

...will work identically because `RosParams::Get<int>` has the same signature as
`YamlParams::Get<int>`.

### 4.7 `Interpolate()` and `RLControl()` — No Changes Needed

These functions in `rl_sdk.cpp` (lines 531–608) access params via `rl.params.Get<T>(...)`.
Same reasoning as above — the interface is unchanged, so no edits required.

---

## 5. Parameter YAML File Conversion

### 5.1 Convert `base.yaml` to ROS 2 Format

The file format changes slightly. ROS 2 expects parameters under a
`<node_name>.ros__parameters` key.

**Before** (`policy/go2/base.yaml`):

```yaml
go2:
  dt: 0.005
  decimation: 4
  num_of_dofs: 12
  fixed_kp: [80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0]
  fixed_kd: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
  default_dof_pos: [0.00, 0.80, -1.50, 0.00, 0.80, -1.50, 0.00, 0.80, -1.50, 0.00, 0.80, -1.50]
  joint_names: ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"]
  joint_mapping: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
  torque_limits: [23.5, 23.5, 23.5, 23.5, 23.5, 23.5, 23.5, 23.5, 23.5, 23.5, 23.5, 23.5]
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
    joint_names: ["FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                  "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                  "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                  "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"]
    joint_mapping: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
    torque_limits: [23.5, 23.5, 23.5, 23.5, 23.5, 23.5, 23.5, 23.5, 23.5, 23.5, 23.5, 23.5]
    wheel_indices: [0]  # NOTE: ROS 2 does not support empty arrays, use a dummy or handle in code
```

Using `/**:` as the node name wildcard makes it apply to any node. Alternatively, replace
`/**` with your specific node name (e.g. `robot_controller:`).

### 5.2 Load in Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='robot_controller',
            name='robot_controller',
            parameters=['config/go2_params.yaml'],
        ),
    ])
```

### 5.3 Empty Array Gotcha

ROS 2 parameters cannot represent empty arrays. For `wheel_indices: []`, you have
two options:

- **Option A**: Use a sentinel value like `[-1]` and filter in code
- **Option B**: Don't declare it at all and handle `Has()` returning false:

```cpp
std::vector<int> wheel_idx;
if (params.Has("wheel_indices"))
    wheel_idx = params.Get<std::vector<int>>("wheel_indices");
```

---

## 6. Summary Checklist

```
[ ] Create ros_params.hpp with RosParams class + float/double specializations
[ ] In rl_sdk.hpp: replace YamlParams struct with #include "ros_params.hpp"
[ ] In rl_sdk.hpp: change member type from YamlParams to RosParams (line 191)
[ ] In rl_sdk.hpp: delete ReadYaml declaration (line 222)
[ ] In rl_sdk.cpp: delete ReadYaml function body (lines 469–488)
[ ] In rl_sdk.cpp: delete ReadYaml call inside InitRL (line 229)
[ ] In each node constructor: replace ReadYaml(...) with params.Init(ros2_node)
    - rl_sim.cpp line 56
    - rl_sim_mujoco.cpp line 85
    - rl_real_go2.cpp line 26
    - rl_real_a1.cpp line 24
    - rl_real_lite3.cpp line 24
    - rl_real_g1.cpp line 24
    - rl_real_l4w4.cpp line 24
[ ] Convert base.yaml files to ROS 2 parameter YAML format
[ ] Update launch files to load the new parameter YAML
[ ] Remove yaml-cpp dependency from CMakeLists.txt / package.xml (if no longer needed)
[ ] No changes needed in fsm_robot/*.hpp or Interpolate/RLControl
```
