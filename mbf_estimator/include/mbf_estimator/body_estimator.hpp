
//   /imu/data ─────────────────►┐
//                               │
//   /joint_states (q,v,τ) ──────┤──► [GRF synth] → [predict] → [update]
//                                                       │            │
//                                                       │            ├──► /state_estimator/odom
//                                                       │            └──► TF: odom -> base_link
//                                                       │
//                                                       └──► /state_estimator/contact_force_*_foot
//                                                            (published for downstream / debug)
//
// One node, one /joint_states-rate cycle: synthesise per-foot ||F|| from
// τ, compute contact_prob via sigmoid, LP-filter into trust, run the KF.

#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include "mbf_estimator/linear_kf.hpp"

namespace mbf_estimator {

class BodyEstimator : public rclcpp::Node {
 public:
  BodyEstimator();

 private:
  struct LegSpec {
    std::string name;
    std::vector<std::string> joint_names;
    std::string foot_frame;
    std::string contact_topic;       // also the wrench publish topic

    pinocchio::FrameIndex frame_id{0};
    std::array<int, 3>    idx_v{};

    // Published for downstream / plotjuggler. Same topic name the bridge
    // used; if both nodes are launched simultaneously, the publishers
    // collide — pick one.
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub;

    double last_force_mag{0.0};
    double contact_prob{0.0};   // 0..1, sigmoid-shaped in ||F||
    double trust{0.0};          // 0..1, LP-filtered contact_prob
  };

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void buildIncomingIndex(const sensor_msgs::msg::JointState& msg);

  
  void computeAndPublishGrfs(const Eigen::VectorXd& q,
                             const Eigen::VectorXd& tau,
                             const builtin_interfaces::msg::Time& stamp);

  // I/O params
  std::string js_topic_, imu_topic_, odom_topic_;
  std::string odom_frame_, base_frame_;
  bool        publish_tf_{true};
  bool        publish_debug_{false};

  
  double      contact_force_center_{10.0};
  double      contact_force_scale_{3.0};

  bool        gravity_compensate_grf_{true};

  double      lp_tau_{0.05};
  double      imu_timeout_{0.1};

  // Filter
  LinearKF              kf_;
  LinearKF::Params      kf_params_;
  Eigen::Vector3d       gravity_world_{0.0, 0.0, -9.81};
  std::optional<rclcpp::Time> last_step_;

  // Pinocchio model + per-leg specs (resolved at startup).
  pinocchio::Model      model_;
  pinocchio::Data       data_;
  std::vector<LegSpec>  legs_;
  std::vector<int>      idx_in_v_;       // model.names[k+1] -> incoming JointState index
  int                   last_incoming_name_count_{-1};  // triggers idx rebuild on change

  // Latest IMU sample (predict input).
  std::mutex                              imu_mutex_;
  std::optional<sensor_msgs::msg::Imu>    latest_imu_;

  // ROS handles
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>                tf_pub_;
};

}  // namespace mbf_estimator
