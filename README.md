# MBF
My own implementation on building a quadrupedal robotic platform.

## Dependencies
- ROS2 Distribution: Humble
- Simulation Medium: Gazebo Classic
- ROS2 Control & ROS2 Controllers
- LibTorch

## Features
### Odrive based Hardware Interface
Custom Hardware Interface to interface with Odrive Controlelrs

## PD Torque Controller
ROS2 Controller that controls joints via PD loop.

## Champ Integration
Integration of Champ Controller for basic locomotion capabilities

## RL Integration
Make sure to clone Libtorch to `$HOME` direcotry, otherwise building `mbf_rl` package will fail

Credits to [FanZiQi](https://github.com/fan-ziqi) for `rl_sar` project.

## Installation
```
pixi init ros2_ws
cd ros2_ws
git clone https://github.com/adwng/mbf_champ.git src
pixi shell
colcon build
```

>[!NOTE]
> For real robot, need to clone 
> 
> [Nguyen V](git clone https://github.com/nguyen-v/10_axis_imu_ros2.git)'s repository for IMU interfacing

## Running

### Simulation
```
source install/setup.bash
ros2 launch wbr_bringup robot.launch.py rviz:=true
./src/shfiles/run_controller.sh
./src/shfiles/run_rl_controller.sh #if want to test policy
./src/shfiles/gamepad.sh 
```

## Training New Policy
I used this [repo](https://github.com/lupinjia/LeggedGym-Ex) to train my policy.

After policy is trained, modify `policy` directory to update the `.pt` file and also the `config.yaml`

## BOM
1. RPI4 8GB x1
2. Waveshare CANBUS Hat ([link](https://www.waveshare.com/product/rs485-can-hat-b.htm) here)
3. Waveshare ROS Imu ([link](https://www.waveshare.com/wiki/10_DOF_ROS_IMU_(A)) here)
4. GIM6010-8 X12

## Power Source
I used 5 18650 batteries capable of 15A nominal output in series. To create the connection, I used spliceable battery slots, attached as reference:
![alt text](docs/batteryslot.png)


