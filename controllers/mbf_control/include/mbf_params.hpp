#ifndef __WBR_PARAMS_H__
#define __WBR_PARAMS_H__

#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <vector>

class Parameter_t {
 public:
  int ctrl_freq;

  struct Control {
    double kp, kd;
    double standup_duration, getdown_duration;
    double max_x, max_y, max_z;
    double max_roll, max_pitch, max_yaw;
  } control;

  struct Gamepad {
    int passive_btn, stand_btn, locomotion_btn, down_btn;
    int l_btn_macro, l2_btn_macro, r_btn_macro;
    int left_ud_axis, left_lr_axis, right_ud_axis, right_lr_axis;
  } gamepad;
};

#endif