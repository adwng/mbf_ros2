#include "mbf_estimator/body_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>     // computeJointJacobians, getFrameJacobian
#include <pinocchio/algorithm/rnea.hpp>         // computeGeneralizedGravity

using std::placeholders::_1;

namespace mbf_estimator {

namespace {

Eigen::Quaterniond quatFromMsg(const geometry_msgs::msg::Quaternion& q) {
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
}

}  // namespace

BodyEstimator::BodyEstimator() : rclcpp::Node("body_estimator") {
  // ── Topic / frame params ───────────────────────────────────────────
  js_topic_      = declare_parameter<std::string>("joint_states_topic", "/joint_states");
  imu_topic_     = declare_parameter<std::string>("imu_topic",          "/imu/data");
  odom_topic_    = declare_parameter<std::string>("odom_topic",         "/state_estimator/odom");
  odom_frame_    = declare_parameter<std::string>("odom_frame",         "odom");
  base_frame_    = declare_parameter<std::string>("base_frame",         "base_link");
  publish_tf_    = declare_parameter<bool>("publish_tf",     true);
  publish_debug_ = declare_parameter<bool>("publish_debug",  false);

  // Contact detection: continuous sigmoid in ||F|| per foot. Centre is
  // the 50%-probability point; scale is the width of the transition.
  // Sigmoid is preferred over a Schmitt trigger because the gravity-only
  // GRF estimator leaves small residual "force" on swing legs (~3–6 N from
  // unmodelled Coriolis/inertial torques); a hard threshold flickers in
  // that band, whereas a sigmoid yields a smooth probability that the
  // downstream LP filter can shape into a clean `trust` signal.
  contact_force_center_   = declare_parameter<double>("contact_force_center", 10.0);
  contact_force_scale_    = declare_parameter<double>("contact_force_scale",   3.0);
  gravity_compensate_grf_ = declare_parameter<bool>("gravity_compensate_grf", true);
  // LP time constant on contact_prob → trust. Should be a few control
  // periods so touchdown / liftoff transients are softened but contact
  // changes still respond in < 50 ms.
  lp_tau_               = declare_parameter<double>("lp_tau",                0.05);
  imu_timeout_          = declare_parameter<double>("imu_timeout",           0.1);

  // ── KF tuning ──────────────────────────────────────────────────────
  kf_params_.sigma_acc             = declare_parameter<double>("kf.sigma_acc",             0.5);
  kf_params_.sigma_foot_pos_stance = declare_parameter<double>("kf.sigma_foot_pos_stance", 0.002);
  kf_params_.sigma_meas_pos        = declare_parameter<double>("kf.sigma_meas_pos",        0.005);
  kf_params_.sigma_meas_vel        = declare_parameter<double>("kf.sigma_meas_vel",        0.05);
  kf_params_.sigma_meas_height     = declare_parameter<double>("kf.sigma_meas_height",     0.005);
  kf_params_.swing_inflate         = declare_parameter<double>("kf.swing_inflate",         100.0);
  kf_params_.foot_height_offset    = declare_parameter<double>("kf.foot_height_offset",    0.0);
  const double g_z                 = declare_parameter<double>("kf.gravity_z",            -9.81);
  gravity_world_ = Eigen::Vector3d(0.0, 0.0, g_z);

  // ── URDF / Pinocchio model ────────────────────────────────────────
  const auto urdf = declare_parameter<std::string>("robot_description", "");
  if (urdf.empty()) {
    RCLCPP_FATAL(get_logger(),
        "Parameter 'robot_description' is empty. Pass the URDF in via the launch file.");
    throw std::runtime_error("missing robot_description");
  }
  pinocchio::urdf::buildModelFromXML(urdf, model_);
  data_ = pinocchio::Data(model_);

  // ── Per-leg specs ─────────────────────────────────────────────────
  const auto leg_keys = declare_parameter<std::vector<std::string>>(
      "leg_names", std::vector<std::string>{"lf", "rf", "lh", "rh"});
  if (static_cast<int>(leg_keys.size()) != LinearKF::kNFeet) {
    RCLCPP_FATAL(get_logger(),
        "leg_names must have exactly %d entries (got %zu).",
        LinearKF::kNFeet, leg_keys.size());
    throw std::runtime_error("bad leg count");
  }

  legs_.reserve(leg_keys.size());
  for (size_t i = 0; i < leg_keys.size(); ++i) {
    const auto& key = leg_keys[i];
    LegSpec leg;
    leg.name          = key;
    leg.joint_names   = declare_parameter<std::vector<std::string>>(
        "leg_" + key + ".joints", std::vector<std::string>{});
    leg.foot_frame    = declare_parameter<std::string>("leg_" + key + ".foot_frame", "");
    leg.contact_topic = declare_parameter<std::string>("leg_" + key + ".contact_topic", "");

    if (leg.joint_names.size() != 3 || leg.foot_frame.empty() || leg.contact_topic.empty()) {
      RCLCPP_FATAL(get_logger(),
          "leg_%s spec incomplete (joints=%zu, frame='%s', topic='%s').",
          key.c_str(), leg.joint_names.size(),
          leg.foot_frame.c_str(), leg.contact_topic.c_str());
      throw std::runtime_error("bad leg spec");
    }

    for (size_t j = 0; j < 3; ++j) {
      const auto& jn = leg.joint_names[j];
      if (!model_.existJointName(jn)) {
        RCLCPP_FATAL(get_logger(),
            "Joint '%s' (leg %s) not found in URDF model.",
            jn.c_str(), key.c_str());
        throw std::runtime_error("missing joint in URDF");
      }
      leg.idx_v[j] = model_.idx_vs[model_.getJointId(jn)];
    }

    if (!model_.existFrame(leg.foot_frame)) {
      RCLCPP_FATAL(get_logger(),
          "Foot frame '%s' (leg %s) not found in URDF model.",
          leg.foot_frame.c_str(), key.c_str());
      throw std::runtime_error("missing frame in URDF");
    }
    leg.frame_id = model_.getFrameId(leg.foot_frame);

    // Publish the synthesised wrench on the same topic the bridge used,
    // so plotjuggler / contact_detection consumers don't notice the
    // single-node migration. Duplicate publishers if mbf_se_bridge is
    // also running — disable one.
    leg.pub = create_publisher<geometry_msgs::msg::WrenchStamped>(
        leg.contact_topic, rclcpp::QoS(100));

    legs_.push_back(std::move(leg));
  }

  // ── Filter / publishers / subscribers ─────────────────────────────
  kf_.init(kf_params_);

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 50);
  if (publish_tf_) {
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&BodyEstimator::onImu, this, _1));
  js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      js_topic_, rclcpp::QoS(10),
      std::bind(&BodyEstimator::onJointState, this, _1));

  RCLCPP_INFO(get_logger(),
      "body_estimator: nq=%d nv=%d, %zu legs. imu=%s, joint_states=%s, "
      "odom=%s (%s -> %s)",
      model_.nq, model_.nv, legs_.size(),
      imu_topic_.c_str(), js_topic_.c_str(), odom_topic_.c_str(),
      odom_frame_.c_str(), base_frame_.c_str());
}

void BodyEstimator::onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(imu_mutex_);
  latest_imu_ = *msg;
}

void BodyEstimator::computeAndPublishGrfs(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& tau,
    const builtin_interfaces::msg::Time& stamp) {
  // Optional gravity compensation: subtract g(q) so the residual torque
  // represents contact forcing only (same as mbf_se_bridge/grf_estimator).
  Eigen::VectorXd tau_eff = tau;
  if (gravity_compensate_grf_) {
    pinocchio::computeGeneralizedGravity(model_, data_, q);
    tau_eff -= data_.g;
  }

  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model_.nv);

  for (auto& leg : legs_) {
    J.setZero();
    pinocchio::getFrameJacobian(
        model_, data_, leg.frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

    Eigen::Matrix3d J_lin;
    Eigen::Vector3d tau_leg;
    for (int j = 0; j < 3; ++j) {
      J_lin.col(j) = J.block<3, 1>(0, leg.idx_v[j]);
      tau_leg(j)   = tau_eff(leg.idx_v[j]);
    }
    // fullPivLu solve so a near-singular leg pose (knee fully extended)
    // degrades gracefully instead of overflowing.
    const Eigen::Vector3d F = J_lin.transpose().fullPivLu().solve(tau_leg);
    leg.last_force_mag = F.norm();

    // contact_prob = sigmoid((||F|| - centre) / scale). Clamp the
    // argument to keep std::exp away from over/underflow on impulse
    // spikes at touchdown.
    const double zr = (leg.last_force_mag - contact_force_center_) /
                      std::max(1e-6, contact_force_scale_);
    const double zs = std::clamp(zr, -30.0, 30.0);
    leg.contact_prob = 1.0 / (1.0 + std::exp(-zs));

    geometry_msgs::msg::WrenchStamped w;
    w.header.stamp     = stamp;
    w.header.frame_id  = base_frame_;
    w.wrench.force.x   = F(0);
    w.wrench.force.y   = F(1);
    w.wrench.force.z   = F(2);
    leg.pub->publish(w);
  }
}

void BodyEstimator::buildIncomingIndex(const sensor_msgs::msg::JointState& msg) {
  
  idx_in_v_.assign(static_cast<size_t>(model_.nv), -1);
  for (int k = 0; k < model_.nv; ++k) {
    const auto& jname = model_.names[k + 1];
    auto it = std::find(msg.name.begin(), msg.name.end(), jname);
    if (it == msg.name.end()) {
      RCLCPP_WARN(get_logger(),
          "Joint '%s' is in the URDF but not in the first /joint_states; "
          "q/v will be zeroed for that joint.", jname.c_str());
    } else {
      idx_in_v_[k] = static_cast<int>(it - msg.name.begin());
    }
  }
}

void BodyEstimator::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
  
  const int n_names = static_cast<int>(msg->name.size());
  if (idx_in_v_.empty() || n_names != last_incoming_name_count_) {
    buildIncomingIndex(*msg);
    last_incoming_name_count_ = n_names;
  }

  // Snapshot the latest IMU sample under the mutex.
  sensor_msgs::msg::Imu imu;
  {
    std::lock_guard<std::mutex> lk(imu_mutex_);
    if (!latest_imu_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "No IMU received yet; skipping estimator step.");
      return;
    }
    imu = *latest_imu_;
  }

  // Compute dt from the joint-state stamp. Defend against clock jumps
  // (sim restart, paused sim, very first sample).
  const rclcpp::Time now(msg->header.stamp);
  double dt = 0.005;
  if (last_step_) {
    const double measured = (now - *last_step_).seconds();
    if (measured > 0.0 && measured < 0.1) dt = measured;
  }
  last_step_ = now;

  const rclcpp::Time imu_t(imu.header.stamp);
  const double imu_age = (now - imu_t).seconds();
  if (std::abs(imu_age) > imu_timeout_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "IMU sample is %.3fs old; estimator output may drift.", imu_age);
  }

  // ── Body orientation & angular velocity from IMU ──────────────────
  // We assume the IMU is mounted aligned with base_link If you add a non-trivial imu_link → base_link
  // rotation later, compose it here.
  const Eigen::Quaterniond q_b_w = quatFromMsg(imu.orientation);
  const Eigen::Matrix3d    R     = q_b_w.toRotationMatrix();   // body → world

  const Eigen::Vector3d omega_b(imu.angular_velocity.x,
                                imu.angular_velocity.y,
                                imu.angular_velocity.z);
  const Eigen::Vector3d omega_w = R * omega_b;

  // Specific force from the accelerometer → acceleration in world frame:
  //   a_imu = R⁻¹ (a_world - g_world)   ⇒   a_world = R a_imu + g_world.
  // With g_world = (0, 0, -9.81), a stationary level robot reads
  // a_imu = (0, 0, +9.81) — the usual accelerometer convention.
  const Eigen::Vector3d a_imu(imu.linear_acceleration.x,
                              imu.linear_acceleration.y,
                              imu.linear_acceleration.z);
  const Eigen::Vector3d a_world = R * a_imu + gravity_world_;

  // ── Foot kinematics + GRF synthesis ────────────────────────────────
  Eigen::VectorXd q   = Eigen::VectorXd::Zero(model_.nq);
  Eigen::VectorXd v   = Eigen::VectorXd::Zero(model_.nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(model_.nv);
  for (int k = 0; k < model_.nv; ++k) {
    const int idx = idx_in_v_[k];
    if (idx < 0) continue;
    if (static_cast<size_t>(idx) < msg->position.size()) q(k)   = msg->position[idx];
    if (static_cast<size_t>(idx) < msg->velocity.size()) v(k)   = msg->velocity[idx];
    if (static_cast<size_t>(idx) < msg->effort.size())   tau(k) = msg->effort[idx];
  }

  // Compute joint Jacobians first (needed for GRF). This internally
  // performs forwardKinematics(q) but does NOT populate data_.v, so we
  // follow up with the (q, v) variant before reading frame velocities.
  // data_.J survives that second call because forwardKinematics doesn't
  // touch the Jacobian buffer.
  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::forwardKinematics(model_, data_, q, v);
  pinocchio::updateFramePlacements(model_, data_);

  computeAndPublishGrfs(q, tau, msg->header.stamp);

  // LP-filter contact_prob into per-foot trust. Tau is exposed via the
  // lp_tau parameter; alpha is the discrete-time pole derived from dt.
  // The filter is applied here (per estimator step) rather than in the
  // wrench callback so its time base matches the predict/update cadence.
  const double alpha = dt / (std::max(1e-6, lp_tau_) + dt);

  std::array<LinearKF::FootObs, LinearKF::kNFeet> obs;
  for (size_t i = 0; i < legs_.size(); ++i) {
    auto& leg = legs_[i];

    leg.trust = (1.0 - alpha) * leg.trust + alpha * leg.contact_prob;

    // Pinocchio model is fixed-base, so "world" inside Pinocchio is
    // base_link. r_b is the hip-to-foot vector in base_link orientation;
    // v_rel_b is the foot's linear velocity relative to base_link, also
    // expressed in base_link orientation.
    const Eigen::Vector3d r_b      = data_.oMf[leg.frame_id].translation();
    const Eigen::Vector3d v_rel_b  = pinocchio::getFrameVelocity(
        model_, data_, leg.frame_id, pinocchio::LOCAL_WORLD_ALIGNED).linear();

    // Compose body motion: rotate into the actual (moving) world frame
    // and add ω × r for the body's rotation about the world origin.
    const Eigen::Vector3d p_rel_world = R * r_b;
    const Eigen::Vector3d v_rel_world = R * v_rel_b + omega_w.cross(p_rel_world);

    obs[i].p_rel_world = p_rel_world;
    obs[i].v_rel_world = v_rel_world;
    obs[i].trust       = leg.trust;
  }

  // ── Combined predict + update ──────────────────────────────────────
  kf_.step(dt, a_world, obs);

  // ── Publish ───────────────────────────────────────────────────────
  nav_msgs::msg::Odometry odom;
  odom.header.stamp     = msg->header.stamp;
  odom.header.frame_id  = odom_frame_;
  odom.child_frame_id   = base_frame_;
  odom.pose.pose.position.x  = kf_.p().x();
  odom.pose.pose.position.y  = kf_.p().y();
  odom.pose.pose.position.z  = kf_.p().z();
  odom.pose.pose.orientation = imu.orientation;
  // nav_msgs/Odometry convention: twist is expressed in child frame.
  const Eigen::Vector3d v_body = R.transpose() * kf_.v();
  odom.twist.twist.linear.x  = v_body.x();
  odom.twist.twist.linear.y  = v_body.y();
  odom.twist.twist.linear.z  = v_body.z();
  odom.twist.twist.angular   = imu.angular_velocity;
  odom_pub_->publish(odom);

  if (publish_tf_ && tf_pub_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header             = odom.header;
    tf.child_frame_id     = base_frame_;
    tf.transform.translation.x = kf_.p().x();
    tf.transform.translation.y = kf_.p().y();
    tf.transform.translation.z = kf_.p().z();
    tf.transform.rotation       = imu.orientation;
    tf_pub_->sendTransform(tf);
  }

  if (publish_debug_) {
    
    Eigen::Vector3d v_kin_avg = Eigen::Vector3d::Zero();
    double weight_sum = 0.0;
    for (size_t i = 0; i < legs_.size(); ++i) {
      if (legs_[i].contact_prob <= 0.5) continue;
      const double w = legs_[i].contact_prob;
      v_kin_avg += w * (-obs[i].v_rel_world);
      weight_sum += w;
    }
    if (weight_sum > 1e-3) v_kin_avg /= weight_sum;
    const Eigen::Vector3d v_kin_body = R.transpose() * v_kin_avg;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[est] p=(%+.2f, %+.2f, %+.2f) v_body=(%+.2f, %+.2f, %+.2f) "
        "v_kin_body=(%+.2f, %+.2f, %+.2f) "
        "omega_b=(%+.2f, %+.2f, %+.2f) "
        "trust=[%.2f %.2f %.2f %.2f] (raw=[%.2f %.2f %.2f %.2f])",
        kf_.p().x(), kf_.p().y(), kf_.p().z(),
        v_body.x(), v_body.y(), v_body.z(),
        v_kin_body.x(), v_kin_body.y(), v_kin_body.z(),
        omega_b.x(), omega_b.y(), omega_b.z(),
        legs_[0].trust, legs_[1].trust,
        legs_[2].trust, legs_[3].trust,
        legs_[0].contact_prob, legs_[1].contact_prob,
        legs_[2].contact_prob, legs_[3].contact_prob);
  }
}

}  // namespace mbf_estimator

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mbf_estimator::BodyEstimator>());
  rclcpp::shutdown();
  return 0;
}
