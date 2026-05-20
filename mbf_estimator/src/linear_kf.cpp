#include "mbf_estimator/linear_kf.hpp"

#include <algorithm>

namespace mbf_estimator {

void LinearKF::init(const Params& p) {
  params_ = p;
  reset();
}

void LinearKF::reset(const Eigen::Vector3d& p0) {
  x_.setZero();
  x_.segment<3>(0) = p0;
  P_.setIdentity();
  P_ *= 100.0;          // start uncertain; first stance update collapses it.
}

void LinearKF::step(double dt,
                    const Eigen::Vector3d& a_world,
                    const std::array<FootObs, kNFeet>& feet) {
  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

  // ── Per-foot trust, clamped to [0, 1] ──────────────────────────────
  std::array<double, kNFeet> trust;
  for (int i = 0; i < kNFeet; ++i) {
    trust[i] = std::clamp(feet[i].trust, 0.0, 1.0);
  }

  // ── A, B ───────────────────────────────────────────────────────────
  // A: integrate body pos with vel; body vel constant; foot pos constant.
  Eigen::Matrix<double, kStateDim, kStateDim> A =
      Eigen::Matrix<double, kStateDim, kStateDim>::Identity();
  A.block<3, 3>(0, 3) = I3 * dt;

  // B: a_world drives body pos (½ dt²) and body vel (dt).
  Eigen::Matrix<double, kStateDim, 3> B =
      Eigen::Matrix<double, kStateDim, 3>::Zero();
  B.block<3, 3>(0, 0) = I3 * 0.5 * dt * dt;
  B.block<3, 3>(3, 0) = I3 * dt;

  // ── Q (process noise) ──────────────────────────────────────────────
  // Body block: canonical [dt⁴/4, dt³/2; dt³/2, dt²] driven by sigma_acc.
  StateCov Q = StateCov::Zero();
  const double s_a2 = params_.sigma_acc * params_.sigma_acc;
  Q.block<3, 3>(0, 0) = I3 * s_a2 * dt * dt * dt * dt / 4.0;
  Q.block<3, 3>(3, 3) = I3 * s_a2 * dt * dt;
  Q.block<3, 3>(0, 3) = I3 * s_a2 * dt * dt * dt / 2.0;
  Q.block<3, 3>(3, 0) = Q.block<3, 3>(0, 3);

  // Per-foot process noise: inflated multiplicatively for swing legs so
  // the world-position estimate of a lifted foot can drift freely. When
  // the foot lands again, the position residual will pull it to the new
  // location instead of being resisted by an over-confident prior.
  const double s_f2 = params_.sigma_foot_pos_stance *
                      params_.sigma_foot_pos_stance;
  for (int i = 0; i < kNFeet; ++i) {
    const double inflate = 1.0 + (1.0 - trust[i]) * params_.swing_inflate;
    Q.block<3, 3>(6 + 3 * i, 6 + 3 * i) = I3 * s_f2 * dt * inflate;
  }

  // ── Capture pre-predict state for measurement blending ─────────────
  // Following MIT Cheetah-Software's pattern: low-trust feet have their
  // velocity / height "measurement" set to the pre-predict body vel /
  // foot z, so the innovation is ~0 and that measurement has no effect.
  // This is numerically nicer than inflating R to infinity, and the
  // outcome is the same for trust → 0.
  const Eigen::Vector3d v_pre = x_.segment<3>(3);
  std::array<double, kNFeet> foot_z_pre;
  for (int i = 0; i < kNFeet; ++i) {
    foot_z_pre[i] = x_(6 + 3 * i + 2);
  }

  // ── Predict ────────────────────────────────────────────────────────
  x_ = A * x_ + B * a_world;
  P_ = A * P_ * A.transpose() + Q;

  // ── H, y, R ────────────────────────────────────────────────────────
  Eigen::Matrix<double, kMeasDim, kStateDim> H =
      Eigen::Matrix<double, kMeasDim, kStateDim>::Zero();
  Eigen::Matrix<double, kMeasDim, 1> y =
      Eigen::Matrix<double, kMeasDim, 1>::Zero();
  Eigen::Matrix<double, kMeasDim, kMeasDim> R =
      Eigen::Matrix<double, kMeasDim, kMeasDim>::Zero();

  for (int i = 0; i < kNFeet; ++i) {
    const int r0       = i * kMeasPerFoot;     // row offset
    const int foot_col = 6 + 3 * i;            // state column for foot i
    const auto& obs    = feet[i];
    const double t     = trust[i];
    const double inflate = 1.0 + (1.0 - t) * params_.swing_inflate;

    // 1) Kinematic position: p_foot_i - p_body == p_rel_world.
    //    ALWAYS applied at nominal noise — it's a kinematic identity that
    //    holds regardless of contact state.
    H.block<3, 3>(r0,     foot_col) =  I3;
    H.block<3, 3>(r0,     0)        = -I3;
    y.segment<3>(r0)                = obs.p_rel_world;
    R.block<3, 3>(r0,     r0)       =
        I3 * params_.sigma_meas_pos * params_.sigma_meas_pos;

    // 2) Zero-velocity constraint at the contact:
    //      foot world velocity = 0  ⇒  v_body = -v_rel_world.
    //    Blended toward pre-predict v_body when low trust (zero innov
    //    when t = 0; full constraint when t = 1).
    H.block<3, 3>(r0 + 3, 3)        = I3;
    y.segment<3>(r0 + 3)            = (1.0 - t) * v_pre +
                                      t * (-obs.v_rel_world);
    R.block<3, 3>(r0 + 3, r0 + 3)   =
        I3 * params_.sigma_meas_vel * params_.sigma_meas_vel * inflate;

    // 3) Flat-ground height: p_foot_i.z == foot_height_offset.
    //    Same blending trick — low trust ⇒ y ≈ predicted foot z ⇒ no
    //    innovation, so we don't try to pull a swing foot to z = 0.
    H(r0 + 6, foot_col + 2) = 1.0;
    y(r0 + 6)               = (1.0 - t) * foot_z_pre[i] +
                              t * params_.foot_height_offset;
    R(r0 + 6, r0 + 6)       =
        params_.sigma_meas_height * params_.sigma_meas_height * inflate;
  }

  // ── Standard linear Kalman update ──────────────────────────────────
  const Eigen::Matrix<double, kMeasDim, 1>          innov = y - H * x_;
  const Eigen::Matrix<double, kMeasDim, kMeasDim>   S     = H * P_ * H.transpose() + R;
  const Eigen::Matrix<double, kStateDim, kMeasDim>  K     =
      P_ * H.transpose() *
      S.ldlt().solve(Eigen::Matrix<double, kMeasDim, kMeasDim>::Identity());

  x_ = x_ + K * innov;
  P_ = (StateCov::Identity() - K * H) * P_;

  // ── Symmetrize P (numerical hygiene, MIT + legged_control) ─────────
  P_ = 0.5 * (P_ + P_.transpose());
}

}  // namespace mbf_estimator
