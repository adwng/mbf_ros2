// linear_kf.hpp — 18-state linear Kalman filter for a quadruped's base.
//
// State (all in WORLD frame):
//   x[0:3]   = p_body
//   x[3:6]   = v_body
//   x[6:9]   = p_foot_0      (lf)
//   x[9:12]  = p_foot_1      (rf)
//   x[12:15] = p_foot_2      (lh)
//   x[15:18] = p_foot_3      (rh)
//

// References:
//   - MIT Cheetah-Software: common/src/Controllers/PositionVelocityEstimator.cpp
//   - qiayuanl/legged_control: legged_estimation/src/LinearKalmanFilter.cpp
//
// Per-foot `trust` ∈ [0, 1] is the LP-filtered contact probability —
// rises gradually after touchdown, falls gradually at liftoff. It drives
// THREE distinct mechanisms (all combined):
//
//   1. Foot process-noise inflation: swing feet have ~100× larger
//      process noise on their world position, so the filter "forgets"
//      where they were and accepts the new touchdown position quickly.
//
//   2. Measurement blending: low-trust feet have their velocity and
//      height "measurements" set to the predicted value, so the
//      innovation is ~0 and the update has no effect on the state
//      (MIT's preferred way to gate measurements — numerically nicer
//      than crank-R-to-infinity).
//
//   3. Measurement-noise inflation: belt-and-suspenders on top of (2),
//      multiplicatively inflates R for vel and height blocks by
//      (1 + (1 - trust) * swing_inflate).
//
// Position residuals are ALWAYS applied at full trust — the kinematic
// identity p_foot = p_body + R·r_b is always meaningful regardless of
// whether the foot is on the ground.

#pragma once

#include <Eigen/Dense>
#include <array>

namespace mbf_estimator {

class LinearKF {
 public:
  static constexpr int kStateDim    = 18;
  static constexpr int kNFeet       = 4;
  static constexpr int kMeasPerFoot = 7;   // 3 pos + 3 vel + 1 height
  static constexpr int kMeasDim     = kNFeet * kMeasPerFoot;

  using State    = Eigen::Matrix<double, kStateDim, 1>;
  using StateCov = Eigen::Matrix<double, kStateDim, kStateDim>;

  struct Params {
    // Process-noise std-devs.
    double sigma_acc{0.5};                  // body accel noise   [m/s²]
    double sigma_foot_pos_stance{0.002};    // foot drift stance  [m]
    // Measurement-noise std-devs.
    double sigma_meas_pos{0.005};           // kinematic position [m]
    double sigma_meas_vel{0.05};            // zero-vel constraint [m/s]
    double sigma_meas_height{0.005};        // flat-ground height [m]
    // Multiplicative inflation factor for swing legs (MIT calls this
    // "high_suspect_number"). Applied as `1 + (1 - trust) * N` to both:
    //   - foot-position process noise (lets the filter forget swing-foot
    //     world position so touchdown can update it freely)
    //   - velocity and height measurement noise (additional safety on
    //     top of the y-blends-to-prediction trick below)
    // The position residual block is intentionally NOT inflated.
    double swing_inflate{100.0};
    // World-frame z assumed for the contact point. The flat-ground height
    // residual is `p_foot.z == foot_height_offset`. Set to the signed
    // offset from the URDF foot frame to the actual contact point in z
    // (e.g. +0.02 m if the foot frame is at the ankle and the foot has a
    // 2 cm radius below it).
    double foot_height_offset{0.0};         // [m]
  };

  struct FootObs {
    // Foot position relative to body, in WORLD frame:
    //   p_rel_world = R_body_to_world · r_hip_to_foot_body
    Eigen::Vector3d p_rel_world{Eigen::Vector3d::Zero()};
    // Foot velocity relative to body, in WORLD frame:
    //   v_rel_world = omega_world × p_rel_world
    //                 + R_body_to_world · v_foot_in_body
    Eigen::Vector3d v_rel_world{Eigen::Vector3d::Zero()};
    // LP-filtered contact probability ∈ [0, 1]. Drives both Q/R inflation
    // and measurement blending (see header comment).
    double trust{0.0};
  };

  void init(const Params& p);
  void reset(const Eigen::Vector3d& p0 = Eigen::Vector3d::Zero());

  
  void step(double dt,
            const Eigen::Vector3d& a_world,
            const std::array<FootObs, kNFeet>& feet);

  Eigen::Vector3d p()      const { return x_.segment<3>(0); }
  Eigen::Vector3d v()      const { return x_.segment<3>(3); }
  Eigen::Vector3d foot(int i) const { return x_.segment<3>(6 + 3 * i); }
  const State&    state()  const { return x_; }
  const StateCov& cov()    const { return P_; }

 private:
  Params   params_{};
  State    x_ = State::Zero();
  StateCov P_ = StateCov::Identity() * 100.0;
};

}  // namespace mbf_estimator
