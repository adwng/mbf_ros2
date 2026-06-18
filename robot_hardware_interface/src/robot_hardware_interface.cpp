#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_enums.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "socket_can.hpp"

namespace robot_ros2_control {

class Axis;

class RobotHardwareInterface final : public hardware_interface::SystemInterface {
 public:
  using return_type = hardware_interface::return_type;
  using State = rclcpp_lifecycle::State;

  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  CallbackReturn on_configure(const State& previous_state) override;
  CallbackReturn on_cleanup(const State& previous_state) override;
  CallbackReturn on_activate(const State& previous_state) override;
  CallbackReturn on_deactivate(const State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
  return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

 private:
  void on_can_msg(const can_frame& frame);
  void set_axis_command_mode(const Axis& axis);

  bool active_;
  EpollEventLoop event_loop_;
  std::vector<Axis> axes_;
  std::string can_intf_name_;
  int logging_;
  int read_torques;
  SocketCanIntf can_intf_;
  rclcpp::Time timestamp_;
};

struct Axis {
  Axis(SocketCanIntf* can_intf, uint32_t node_id)
      : can_intf_(can_intf),
        node_id_(node_id),
        reduction_ratio_(1.0),
        auto_offset_(0.0),
        manual_offset_rad_(0.0),
        offset_(0.0),
        axis_direction_(1) {}

  void on_can_msg(const rclcpp::Time& timestamp, const can_frame& frame);

  void on_can_msg();

  SocketCanIntf* can_intf_;
  uint32_t node_id_;
  double reduction_ratio_;
  int auto_offset_;
  double manual_offset_rad_;
  double offset_;
  int axis_direction_;
  bool offset_initialized_ = false;
  bool encoder_ready_ = false;

  // Commands (ros2_control => ODrives)
  double pos_setpoint_ = 0.0f;     // [rad]
  double vel_setpoint_ = 0.0f;     // [rad/s]
  double torque_setpoint_ = 0.0f;  // [Nm]

  // State (ODrives => ros2_control)
  // rclcpp::Time encoder_estimates_timestamp_;
  // uint32_t axis_error_ = 0;
  // uint8_t axis_state_ = 0;
  // uint8_t procedure_result_ = 0;
  // uint8_t trajectory_done_flag_ = 0;
  double pos_estimate_ = NAN;  // [rad]
  double vel_estimate_ = NAN;  // [rad/s]
  // double iq_setpoint_ = NAN;
  // double iq_measured_ = NAN;
  double torque_target_ = NAN;    // [Nm]
  double torque_estimate_ = NAN;  // [Nm]
  // uint32_t active_errors_ = 0;
  // uint32_t disarm_reason_ = 0;
  // double fet_temperature_ = NAN;
  // double motor_temperature_ = NAN;
  // double bus_voltage_ = NAN;
  // double bus_current_ = NAN;

  // Indicates which controller inputs are enabled. This is configured by the
  // controller that sits on top of this hardware interface. Multiple inputs
  // can be enabled at the same time, in this case the non-primary inputs are
  // used as feedforward terms.
  // This implicitly defines the ODrive's control mode.
  bool pos_input_enabled_ = false;
  bool vel_input_enabled_ = false;
  bool torque_input_enabled_ = false;

  template <typename T>
  void send(const T& msg) const {
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | msg.cmd_id;
    frame.can_dlc = msg.msg_length;
    msg.encode_buf(frame.data);

    can_intf_->send_can_frame(frame);
  }

  // Send a Remote Transmission Request for a motor->host message, asking the
  // ODrive to reply once with the corresponding data frame. Used for messages
  // that the SteadyWin GIM6010-8 firmware does not broadcast cyclically (e.g.
  // Get_Torques, 0x01C). The reply arrives as a normal data frame and is
  // dispatched through Axis::on_can_msg by the standard receive path.
  // If a particular firmware build does not honor RTR for a given cmd_id,
  // fall back to RxSdo (0x004) reads using endpoint IDs from the firmware
  // version's endpoints JSON.
  template <typename TMsg>
  void request() const {
    struct can_frame frame {};
    frame.can_id = (node_id_ << 5) | TMsg::cmd_id;
    frame.can_id |= CAN_RTR_FLAG;
    frame.can_dlc = TMsg::msg_length;
    can_intf_->send_can_frame(frame);
  }
};

}  // namespace robot_ros2_control

using namespace robot_ros2_control;

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn RobotHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  can_intf_name_ = info_.hardware_parameters["can"];
  logging_ = std::stoi(info_.hardware_parameters.at("logging"));
  read_torques = std::stoi(info_.hardware_parameters.at("read_torques"));

  for (auto& joint : info_.joints) {
    uint32_t node_id = std::stoi(joint.parameters.at("node_id"));
    Axis axis(&can_intf_, node_id);
    // Parse the reduction_ratio parameter; if missing, you could default
    // to 1.0.
    if (joint.parameters.find("reduction_ratio") != joint.parameters.end()) {
      axis.reduction_ratio_ = std::stod(joint.parameters.at("reduction_ratio"));
    }
    if (joint.parameters.find("offset") != joint.parameters.end()) {
      // `offset` is configured in JOINT RADIANS.
      axis.manual_offset_rad_ = std::stod(joint.parameters.at("offset"));
    }
    if (joint.parameters.find("axis_direction") != joint.parameters.end()) {
      axis.axis_direction_ = std::stoi(joint.parameters.at("axis_direction"));
    }
    if (joint.parameters.find("auto_offset") != joint.parameters.end()) {
      axis.auto_offset_ = std::stoi(joint.parameters.at("auto_offset"));
    }
    // Internal storage uses motor turns.
    axis.offset_ = (axis.manual_offset_rad_ * axis.reduction_ratio_) / (2 * M_PI) /
                   axis.axis_direction_;
    axes_.push_back(axis);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotHardwareInterface::on_configure(const State&) {
  if (!can_intf_.init(can_intf_name_, &event_loop_,
                      std::bind(&RobotHardwareInterface::on_can_msg, this, _1))) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"),
                 "Failed to initialize SocketCAN on %s",
                 can_intf_name_.c_str());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"),
              "Initialized SocketCAN on %s", can_intf_name_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotHardwareInterface::on_cleanup(const State&) {
  can_intf_.deinit();
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotHardwareInterface::on_activate(const State&) {
  RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"),
              "activating ODrives...");

  active_ = true;

  // Reset encoder readiness on every activation so we always use fresh data.
  for (auto& axis : axes_) {
    axis.encoder_ready_ = false;
    axis.pos_estimate_ = NAN;
    axis.vel_estimate_ = NAN;
    axis.torque_target_ = NAN;
    axis.torque_estimate_ = NAN;
  }

  // Wait briefly to collect initial encoder values
  RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"),
              "Waiting for encoder estimates...");

  auto start = std::chrono::steady_clock::now();

  while (true) {
    can_intf_.read_nonblocking();

    bool all_ready = true;
    for (auto& axis : axes_) {
      if (axis.auto_offset_ == 1 && !axis.encoder_ready_) {
        all_ready = false;
        break;
      }
    }

    if (all_ready) break;

    if (std::chrono::steady_clock::now() - start > std::chrono::seconds(2)) {
      RCLCPP_WARN(rclcpp::get_logger("RobotHardwareInterface"),
                  "Timeout waiting for encoder estimates");
      break;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"),
              "Capturing startup encoder positions.");

  bool auto_offset_failed = false;
  for (auto& axis : axes_) {
    if (axis.auto_offset_ == 1) {
      if (!std::isnan(axis.pos_estimate_)) {
        // Convert joint radians back to motor turns.
        // pos_estimate_ = (raw_turns - offset_) * scale
        // => raw_turns = motor_turns_from_pos + offset_
        const double motor_turns_from_pos =
            (axis.pos_estimate_ * axis.reduction_ratio_) / (2 * M_PI) /
            axis.axis_direction_;
        const double startup_motor_turns = motor_turns_from_pos + axis.offset_;
        const double manual_offset_turns =
            (axis.manual_offset_rad_ * axis.reduction_ratio_) / (2 * M_PI) /
            axis.axis_direction_;

        // Deterministic rule:
        // final offset = startup measured offset + configured manual trim.
        axis.offset_ = startup_motor_turns + manual_offset_turns;
        axis.offset_initialized_ = true;

        RCLCPP_INFO(
            rclcpp::get_logger("RobotHardwareInterface"),
            "Axis %d auto+manual offset -> startup: %.6f turns, manual: %.6f rad (%.6f turns), final: %.6f turns",
            axis.node_id_, startup_motor_turns, axis.manual_offset_rad_,
            manual_offset_turns, axis.offset_);
      } else {
        auto_offset_failed = true;
        RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"),
                     "Axis %d encoder not ready. Auto offset failed.",
                     axis.node_id_);
      }
    } else {
      // Manual-only mode.
      axis.offset_ =
          (axis.manual_offset_rad_ * axis.reduction_ratio_) / (2 * M_PI) /
          axis.axis_direction_;
      axis.offset_initialized_ = true;
      RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"),
                  "Axis %d manual offset only: %.6f rad (%.6f turns)",
                  axis.node_id_, axis.manual_offset_rad_, axis.offset_);
    }
  }

  // Avoid mixed/partial offsets that cause inconsistent posture.
  if (auto_offset_failed) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"),
                 "Activation aborted due to incomplete auto_offset data.");
    active_ = false;
    return CallbackReturn::ERROR;
  }

  for (auto& axis : axes_) {
    set_axis_command_mode(axis);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotHardwareInterface::on_deactivate(const State&) {
  RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"),
              "deactivating ODrives...");

  active_ = false;
  for (auto& axis : axes_) {
    set_axis_command_mode(axis);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RobotHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    // return torque_targets if not reading torques
    if (read_torques) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &axes_[i].torque_estimate_));
    } else {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &axes_[i].torque_target_));
    }
    
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &axes_[i].vel_estimate_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &axes_[i].pos_estimate_));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RobotHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &axes_[i].torque_setpoint_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &axes_[i].vel_setpoint_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &axes_[i].pos_setpoint_));
  }

  return command_interfaces;
}

return_type RobotHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  for (size_t i = 0; i < axes_.size(); ++i) {
    Axis& axis = axes_[i];
    std::array<std::pair<std::string, bool*>, 3> interfaces = {
        {{info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION,
          &axis.pos_input_enabled_},
         {info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY,
          &axis.vel_input_enabled_},
         {info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT,
          &axis.torque_input_enabled_}}};

    bool mode_switch = false;

    for (const std::string& key : stop_interfaces) {
      for (auto& kv : interfaces) {
        if (kv.first == key) {
          *kv.second = false;
          mode_switch = true;
        }
      }
    }

    for (const std::string& key : start_interfaces) {
      for (auto& kv : interfaces) {
        if (kv.first == key) {
          *kv.second = true;
          mode_switch = true;
        }
      }
    }

    if (mode_switch) {
      set_axis_command_mode(axis);
    }
  }

  return return_type::OK;
}

return_type RobotHardwareInterface::read(const rclcpp::Time& timestamp,
                                       const rclcpp::Duration&) {
  timestamp_ = timestamp;

  while (can_intf_.read_nonblocking()) {
    // Drain inbound frames; this consumes replies to the previous cycle's
    // Get_Torques RTRs as well as any cyclic encoder broadcasts.
  }

  // The SteadyWin GIM6010-8 firmware does not broadcast Get_Torques (0x01C)
  // cyclically, so poll it via RTR every cycle. The device replies with a
  // standard data frame which Axis::on_can_msg decodes on the next read(). 
  // Only activate if needed by xacro, otherwise don't read. This is to prevent consecutive RTR polling
  if (active_ && read_torques) {
    for (auto& axis : axes_) {
      axis.request<Get_Torques_msg_t>();
    }
  }

  return return_type::OK;
}

return_type RobotHardwareInterface::write(const rclcpp::Time&,
                                        const rclcpp::Duration&) {
  for (size_t i = 0; i < axes_.size(); ++i) {
    auto& axis = axes_[i];

    if (axis.pos_input_enabled_) {
      Set_Input_Pos_msg_t msg;
      // Convert joint command [rad] to motor turns, then apply motor-turn offset.
      msg.Input_Pos = ((axis.pos_setpoint_ * axis.reduction_ratio_) /
                       (2 * M_PI)) *
                          axis.axis_direction_ +
                      axis.offset_;
      msg.Vel_FF =
          axis.vel_input_enabled_
              ? ((axis.vel_setpoint_ * axis.reduction_ratio_) / (2 * M_PI)) *
                    axis.axis_direction_
              : 0.0f;
      msg.Torque_FF = axis.torque_input_enabled_
                          ? (axis.torque_setpoint_ / axis.reduction_ratio_) *
                                axis.axis_direction_
                          : 0.0f;
      axis.send(msg);
    } else if (axis.vel_input_enabled_) {
      Set_Input_Vel_msg_t msg;
      msg.Input_Vel = (axis.vel_setpoint_ * axis.reduction_ratio_) /
                      (2 * M_PI) * axis.axis_direction_;
      msg.Input_Torque_FF =
          axis.torque_input_enabled_
              ? (axis.torque_setpoint_ / axis.reduction_ratio_) *
                    axis.axis_direction_
              : 0.0f;
      axis.send(msg);
    } else if (axis.torque_input_enabled_) {
      Set_Input_Torque_msg_t msg;
      msg.Input_Torque =
          axis.torque_setpoint_ / axis.reduction_ratio_ * axis.axis_direction_;
      axis.send(msg);
    }

    if (logging_ == 1) {
        const double logged_torque = read_torques ? axis.torque_estimate_ : axis.torque_setpoint_;
        RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"),
                    "[Joint: %s] cmd_pos: %.4f rad, cmd_vel: %.4f rad/s, "
                    "cmd_torque: %.4f Nm | "
                    "pos_est: %.4f rad, vel_est: %.4f rad/s, torque_est: %.4f Nm",
                    info_.joints[i].name.c_str(), axis.pos_setpoint_,
                    axis.vel_setpoint_, axis.torque_setpoint_, axis.pos_estimate_,
                    axis.vel_estimate_, logged_torque);
    }
  }

  return return_type::OK;
}

void RobotHardwareInterface::on_can_msg(const can_frame& frame) {
  for (auto& axis : axes_) {
    if ((frame.can_id >> 5) == axis.node_id_) {
      axis.on_can_msg(timestamp_, frame);
    }
  }
}

void RobotHardwareInterface::set_axis_command_mode(const Axis& axis) {
  if (!active_) {
    RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"),
                "Interface inactive. Setting axis to idle.");
    Set_Axis_State_msg_t idle_msg;
    idle_msg.Axis_Requested_State = AXIS_STATE_IDLE;
    axis.send(idle_msg);
    return;
  }

  Set_Controller_Mode_msg_t control_msg;
  Clear_Errors_msg_t clear_error_msg;
  Set_Axis_State_msg_t state_msg;

  clear_error_msg.Identify = 0;
  control_msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
  state_msg.Axis_Requested_State = AXIS_STATE_CLOSED_LOOP_CONTROL;

  if (axis.pos_input_enabled_) {
    RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"),
                "Setting to position control.");
    control_msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
  } else if (axis.vel_input_enabled_) {
    RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"),
                "Setting to velocity control.");
    control_msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
  } else if (axis.torque_input_enabled_) {
    RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"),
                "Setting to torque control.");
    control_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"),
                "No control mode specified. Setting to idle.");
    state_msg.Axis_Requested_State = AXIS_STATE_IDLE;
    axis.send(state_msg);
    return;
  }

  axis.send(control_msg);
  axis.send(clear_error_msg);
  axis.send(state_msg);
}

void Axis::on_can_msg(const rclcpp::Time&, const can_frame& frame) {
  uint8_t cmd = frame.can_id & 0x1f;

  auto try_decode = [&]<typename TMsg>(TMsg& msg) {
    if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
      RCLCPP_WARN(rclcpp::get_logger("RobotHardwareInterface"),
                  "message %d too short", cmd);
      return false;
    }
    msg.decode_buf(frame.data);
    return true;
  };

  switch (cmd) {
    case Get_Encoder_Estimates_msg_t::cmd_id: {
      if (Get_Encoder_Estimates_msg_t msg; try_decode(msg)) {
        pos_estimate_ =
            ((msg.Pos_Estimate - offset_) * (2 * M_PI) / reduction_ratio_) *
            axis_direction_;
        vel_estimate_ = (msg.Vel_Estimate * (2 * M_PI) / reduction_ratio_) *
                        axis_direction_;
        encoder_ready_ = true;
      }
    } break;
    case Get_Torques_msg_t::cmd_id: {
      if (Get_Torques_msg_t msg; try_decode(msg)) {
        torque_target_ =
            (msg.Torque_Target / reduction_ratio_) * axis_direction_;
        torque_estimate_ =
            (msg.Torque_Estimate * reduction_ratio_) * axis_direction_;
        ;
      }
    } break;
      // silently ignore unimplemented command IDs
  }
}

PLUGINLIB_EXPORT_CLASS(robot_ros2_control::RobotHardwareInterface,
                       hardware_interface::SystemInterface)