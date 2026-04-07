# MBF Quadruped Stack

MBF is a ROS 2 Humble workspace for a quadruped platform with:
- ros2_control-based hardware/simulation control
- CHAMP-based locomotion control
- Reinforcement learning policy integration
- ODrive motor interface for real hardware

## Repository Layout

- `mbf_description`: robot model, ros2_control configs, simulation assets
- `mbf_bringup`: launch files for simulation and real robot bringup
- `controllers/mbf_control`: joystick + locomotion + stabilization controller
- `controllers/mbf_rl`: RL runtime controller and policy integration
- `robot_joint_controller`: per-joint low-level torque/PD controller plugin

## Dependencies

- ROS 2 Humble
- Gazebo Classic (for simulation)
- `ros2_control` and `ros2_controllers`
- LibTorch (required by `controllers/mbf_rl`)

## Features

### ODrive Hardware Interface
Custom hardware interface for ODrive motor controllers.

### Joint PD/Torque Control
`robot_joint_controller` plugin performs joint-level command handling for each actuator.

### CHAMP Integration
CHAMP locomotion stack integration for command-driven gait and posture.

### RL Integration
RL inference path under `controllers/mbf_rl`.

Credit to [FanZiQi](https://github.com/fan-ziqi) for inspiration from `rl_sar`.

## Installation

```bash
cd ros2_ws
git clone https://github.com/adwng/mbf_champ.git src
colcon build
source install/setup.bash
```

## LibTorch Requirement (RL)

`mbf_rl` expects LibTorch to be installed in `$HOME`.  
If LibTorch is missing or in a different path, `controllers/mbf_rl` may fail to build.

## Running

### Simulation

```bash
source install/setup.bash
ros2 launch mbf_bringup robot.launch.py use_sim:=true rviz:=true
./src/shfiles/run_controller.sh
./src/shfiles/run_rl_controller.sh   # optional: test RL policy
./src/shfiles/gamepad.sh
```

### Real Robot

```bash
source install/setup.bash
ros2 launch mbf_bringup robot.launch.py use_sim:=false rviz:=false
```

For real robot operation, complete this checklist first.

#### 1) Configure CAN Service (One-Time)

Follow your CAN HAT vendor guide to enable CAN on Raspberry Pi and make it persistent at boot.
If you keep helper scripts in `shfiles`, use them to install systemd startup units once.

#### 2) Enable UART for IMU (If IMU Uses GPIO UART)

If your IMU is connected through GPIO UART (instead of USB-UART):
- enable serial/UART in Raspberry Pi configuration
- add your user to `dialout`:

```bash
sudo usermod -aG dialout $USER
```

- reboot, then verify UART device links:

```bash
ls -l /dev/serial*
```

- if Bluetooth occupies `ttyAMA0`, add `dtoverlay=disable-bt` in `/boot/firmware/config.txt` (with `enable_uart=1`)

#### 3) Install and Run IMU Driver

This project expects IMU data from an external IMU node.

```bash
cd ~/ros2_ws/src
git clone https://github.com/nguyen-v/10_axis_imu_ros2.git
```

Build it following that repository's instructions, then ensure it publishes `sensor_msgs/msg/Imu` to `imu/data` (or remap as needed).

#### 4) Calibrate ODrive Motor Drivers

This stack assumes ODrive over CAN for hardware control.
Before first real run:
- assign ODrive node IDs
- set CAN baudrate and bus termination correctly
- run motor calibration and direction validation

Helper scripts in this repo:
- `scripts/odrive_calibrate.py`
- `scripts/odrive_interactive.py`
- `scripts/calibration.md`

#### 5) Hardware Interface Direction/Offset Validation

Validate sign conventions and zero offsets before enabling full torque output.
Start with logging/debug enabled in your `ros2_control` hardware configuration, verify that commanded and measured directions are consistent for all joints, then disable verbose logging for normal use.

#### 6) Bringup Sequence

Recommended startup order on real hardware:
1. CAN interface up and verified
2. IMU node running
3. robot bringup launch (`use_sim:=false`)
4. controller process (`./src/shfiles/run_controller.sh`)
5. optional RL controller (`./src/shfiles/run_rl_controller.sh`)
6. gamepad node/client (`./src/shfiles/gamepad.sh`)

## Training a New Policy

Training is done outside this repo. One tested option:
- [LeggedGym-Ex](https://github.com/lupinjia/LeggedGym-Ex)

After training:
- export/update the policy `.pt` file in the relevant `controllers/mbf_rl/policy/...` directory
- update the matching `config.yaml`

## Bill of Materials (BOM)

1. Raspberry Pi 4 (8 GB) x1
2. Waveshare CANBUS Hat ([link](https://www.waveshare.com/product/rs485-can-hat-b.htm))
3. Waveshare 10 DOF ROS IMU ([link](https://www.waveshare.com/wiki/10_DOF_ROS_IMU_(A)))
4. GIM6010-8 motors x12

## Power Source

Current setup uses five 18650 cells in series (nominal 15 A output), assembled with spliceable battery slots.

![Battery slot reference](docs/batteryslot.png)


