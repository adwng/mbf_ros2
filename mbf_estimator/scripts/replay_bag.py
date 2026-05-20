#!/usr/bin/env python3
"""replay_bag.py — Offline replay of mbf_estimator on a recorded ros2 bag.

Re-runs the same linear KF + per-foot trust pipeline that body_estimator
runs online, but driven by messages from a rosbag2 (sqlite3) recording.
Lets you tune sigmas / gating without restarting Gazebo and compare to
ground-truth odometry on the same input stream.

What it does, end-to-end:

  1. Builds a Pinocchio model from a URDF or xacro.
  2. Streams the bag in timestamp order, caching the latest IMU and
     joint state.
  3. Per joint-state message:
       - Synthesises a per-foot GRF magnitude from joint efforts (same
         math as mbf_se_bridge/grf_estimator), so the replay does not
         depend on which contact wrenches were recorded.
       - Runs forward kinematics + frame velocity to get p_rel_world
         and v_rel_world per foot.
       - Computes contact_prob via sigmoid in ||F||, low-pass filters
         it into per-foot trust.
       - Calls the Python LinearKF (mirrors src/linear_kf.cpp).
  4. Aligns truth odom onto the KF timeline (interpolation) and writes
     plots + an RMSE summary.

Why not just deserialize the live /state_estimator/odom in the bag and
compare? Because that doesn't let you change parameters and re-run --
the whole point of this script is to make tuning fast.

Usage:

  pixi run python src/state_estimators/mbf_estimator/scripts/replay_bag.py \\
      rosbag2_2026_05_19-14_35_34 \\
      --xacro src/mbf_description/xacro/robot.xacro \\
      --config src/state_estimators/mbf_estimator/config/estimator.yaml \\
      --out /tmp/replay_run_0 \\
      --override kf.sigma_acc=0.5 \\
      --override kf.sigma_meas_vel=0.02 \\
      --override lp_tau=0.04 \\
      --override contact_force_center=15.0 \\
      --override kf.foot_height_offset=0.023

Each --override updates the loaded estimator.yaml in-place (dotted key),
so the C++ defaults are mirrored exactly unless you change something.
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pinocchio as pin

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Standard message types are looked up lazily by get_message(), but we
# import these so the names appear in stack traces if a topic is missing.
from sensor_msgs.msg import Imu, JointState  # noqa: F401
from geometry_msgs.msg import WrenchStamped  # noqa: F401
from nav_msgs.msg import Odometry  # noqa: F401


# ---------------------------------------------------------------------------
# Linear KF — must stay byte-equivalent to src/linear_kf.cpp.
# ---------------------------------------------------------------------------

@dataclass
class KFParams:
    sigma_acc: float = 0.5
    sigma_foot_pos_stance: float = 0.002
    sigma_meas_pos: float = 0.005
    sigma_meas_vel: float = 0.05
    sigma_meas_height: float = 0.005
    swing_inflate: float = 100.0
    foot_height_offset: float = 0.0


class LinearKF:
    N_FEET = 4
    STATE_DIM = 18
    MEAS_PER_FOOT = 7
    MEAS_DIM = N_FEET * MEAS_PER_FOOT  # 28

    def __init__(self, params: KFParams):
        self.p = params
        self.x = np.zeros(self.STATE_DIM)
        self.P = np.eye(self.STATE_DIM) * 100.0

    def reset(self, p0: Optional[np.ndarray] = None):
        self.x[:] = 0.0
        if p0 is not None:
            self.x[0:3] = p0
        self.P = np.eye(self.STATE_DIM) * 100.0

    def step(self, dt: float, a_world: np.ndarray,
             feet: List[Tuple[np.ndarray, np.ndarray, float]]):
        """One predict+update cycle.

        feet[i] = (p_rel_world, v_rel_world, trust_in_[0,1]).
        """
        I3 = np.eye(3)
        trust = np.clip(np.array([f[2] for f in feet]), 0.0, 1.0)

        # State transition.
        A = np.eye(self.STATE_DIM)
        A[0:3, 3:6] = I3 * dt
        B = np.zeros((self.STATE_DIM, 3))
        B[0:3, :] = I3 * 0.5 * dt * dt
        B[3:6, :] = I3 * dt

        # Process noise.
        Q = np.zeros((self.STATE_DIM, self.STATE_DIM))
        s_a2 = self.p.sigma_acc ** 2
        Q[0:3, 0:3] = I3 * s_a2 * (dt ** 4) / 4.0
        Q[3:6, 3:6] = I3 * s_a2 * (dt ** 2)
        Q[0:3, 3:6] = I3 * s_a2 * (dt ** 3) / 2.0
        Q[3:6, 0:3] = Q[0:3, 3:6]

        s_f2 = self.p.sigma_foot_pos_stance ** 2
        for i in range(self.N_FEET):
            inflate = 1.0 + (1.0 - trust[i]) * self.p.swing_inflate
            Q[6 + 3 * i:9 + 3 * i, 6 + 3 * i:9 + 3 * i] = I3 * s_f2 * dt * inflate

        # Snapshot pre-predict state for measurement blending.
        v_pre = self.x[3:6].copy()
        foot_z_pre = [self.x[6 + 3 * i + 2] for i in range(self.N_FEET)]

        # Predict.
        self.x = A @ self.x + B @ a_world
        self.P = A @ self.P @ A.T + Q

        # Build H, y, R.
        H = np.zeros((self.MEAS_DIM, self.STATE_DIM))
        y = np.zeros(self.MEAS_DIM)
        R = np.zeros((self.MEAS_DIM, self.MEAS_DIM))

        for i in range(self.N_FEET):
            r0 = i * self.MEAS_PER_FOOT
            fc = 6 + 3 * i
            p_rel, v_rel, _ = feet[i]
            t = float(trust[i])
            inflate = 1.0 + (1.0 - t) * self.p.swing_inflate

            # (1) Kinematic position residual — always full trust.
            H[r0:r0 + 3, fc:fc + 3] = I3
            H[r0:r0 + 3, 0:3] = -I3
            y[r0:r0 + 3] = p_rel
            R[r0:r0 + 3, r0:r0 + 3] = I3 * (self.p.sigma_meas_pos ** 2)

            # (2) Zero-velocity at contact, blended to v_pre when trust low.
            H[r0 + 3:r0 + 6, 3:6] = I3
            y[r0 + 3:r0 + 6] = (1.0 - t) * v_pre + t * (-v_rel)
            R[r0 + 3:r0 + 6, r0 + 3:r0 + 6] = I3 * (self.p.sigma_meas_vel ** 2) * inflate

            # (3) Flat-ground height, same blending trick.
            H[r0 + 6, fc + 2] = 1.0
            y[r0 + 6] = (1.0 - t) * foot_z_pre[i] + t * self.p.foot_height_offset
            R[r0 + 6, r0 + 6] = (self.p.sigma_meas_height ** 2) * inflate

        # Standard update with a stable solve.
        innov = y - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.solve(S, np.eye(self.MEAS_DIM))
        self.x = self.x + K @ innov
        I_n = np.eye(self.STATE_DIM)
        self.P = (I_n - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)


# ---------------------------------------------------------------------------
# Helpers: bag iteration, URDF loading, quat -> rotmat.
# ---------------------------------------------------------------------------

def open_bag(bag_dir: Path):
    if not (bag_dir / "metadata.yaml").exists():
        raise FileNotFoundError(f"No metadata.yaml under {bag_dir}")
    storage = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    conv = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, conv)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, type_map


def iter_bag(reader, type_map):
    """Yields (topic, msg, t_ns) for every record in arrival order."""
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        msg_cls = get_message(type_map[topic])
        yield topic, deserialize_message(raw, msg_cls), t_ns


def load_urdf(xacro_path: Optional[Path], urdf_path: Optional[Path]) -> str:
    if urdf_path is not None:
        return urdf_path.read_text()
    if xacro_path is None:
        raise ValueError("Pass either --urdf or --xacro.")
    # Match the launch file: `xacro robot.xacro use_sim:=true`.
    out = subprocess.run(
        ["xacro", str(xacro_path), "use_sim:=true"],
        check=True, capture_output=True, text=True,
    )
    return out.stdout


def quat_to_R(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),   2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)],
    ])


# ---------------------------------------------------------------------------
# Config + per-leg setup.
# ---------------------------------------------------------------------------

@dataclass
class LegSpec:
    name: str
    joint_names: List[str]
    foot_frame: str
    contact_topic: str
    frame_id: int = 0
    idx_v: List[int] = field(default_factory=list)


def load_node_params(cfg_path: Path) -> dict:
    """Mirrors ros2 param resolution: returns body_estimator.ros__parameters."""
    with cfg_path.open() as f:
        doc = yaml.safe_load(f)
    return doc["body_estimator"]["ros__parameters"]


def apply_overrides(params: dict, overrides: List[str]) -> dict:
    """Apply k=v overrides, supporting dotted keys (kf.sigma_acc=0.5)."""
    for kv in overrides:
        if "=" not in kv:
            raise ValueError(f"--override must be key=value, got {kv!r}")
        k, v = kv.split("=", 1)
        # Try numeric, then bool, else string.
        try:
            val = float(v)
        except ValueError:
            if v.lower() in ("true", "false"):
                val = v.lower() == "true"
            else:
                val = v
        params[k] = val
    return params


def build_legs(model: pin.Model, params: dict) -> List[LegSpec]:
    keys = params["leg_names"]
    if len(keys) != 4:
        raise ValueError("Expected exactly 4 legs.")
    legs = []
    for k in keys:
        leg = LegSpec(
            name=k,
            joint_names=list(params[f"leg_{k}.joints"]),
            foot_frame=params[f"leg_{k}.foot_frame"],
            contact_topic=params[f"leg_{k}.contact_topic"],
        )
        if not model.existFrame(leg.foot_frame):
            raise RuntimeError(f"URDF has no frame '{leg.foot_frame}'.")
        leg.frame_id = model.getFrameId(leg.foot_frame)
        for jn in leg.joint_names:
            if not model.existJointName(jn):
                raise RuntimeError(f"URDF has no joint '{jn}'.")
            jid = model.getJointId(jn)
            leg.idx_v.append(model.joints[jid].idx_v)
        legs.append(leg)
    return legs


# ---------------------------------------------------------------------------
# Replay record (one row per KF step).
# ---------------------------------------------------------------------------

@dataclass
class Record:
    t: float                     # seconds since bag start
    p: np.ndarray                # KF body position (world / odom frame)
    v_body: np.ndarray           # KF body-frame velocity
    v_kin_body: np.ndarray       # kinematic-implied body vel (raw-contact-only)
    n_stance: int                # number of feet with raw contact_prob > 0.5
    a_world: np.ndarray          # IMU-derived world accel (for diagnostics)
    raw: np.ndarray              # 4-vector of contact_prob
    trust: np.ndarray            # 4-vector of LP-filtered trust


@dataclass
class TruthSample:
    t: float
    p: np.ndarray
    R: np.ndarray
    v_world: np.ndarray


# ---------------------------------------------------------------------------
# The main pipeline.
# ---------------------------------------------------------------------------

def run_replay(bag_dir: Path, urdf_xml: str, params: dict,
               *, truth_twist_in_world: bool) -> Tuple[List[Record], List[TruthSample]]:
    # --- pinocchio model -------------------------------------------------
    model = pin.buildModelFromXML(urdf_xml)
    data = model.createData()
    legs = build_legs(model, params)

    nv = model.nv

    # Build incoming-name -> pin v-index map lazily on first JointState.
    idx_in_v: Optional[np.ndarray] = None

    # KF config and contact gating.
    kf_p = KFParams(
        sigma_acc=float(params["kf.sigma_acc"]),
        sigma_foot_pos_stance=float(params["kf.sigma_foot_pos_stance"]),
        sigma_meas_pos=float(params["kf.sigma_meas_pos"]),
        sigma_meas_vel=float(params["kf.sigma_meas_vel"]),
        sigma_meas_height=float(params["kf.sigma_meas_height"]),
        swing_inflate=float(params["kf.swing_inflate"]),
        foot_height_offset=float(params["kf.foot_height_offset"]),
    )
    kf = LinearKF(kf_p)
    gravity_world = np.array([0.0, 0.0, float(params["kf.gravity_z"])])

    fc_center = float(params["contact_force_center"])
    fc_scale = max(1e-6, float(params["contact_force_scale"]))
    lp_tau = float(params["lp_tau"])
    gravity_compensate = bool(params.get("gravity_compensate_grf", True))

    contact_prob = np.zeros(4)
    trust = np.zeros(4)

    latest_imu: Optional[Imu] = None
    last_step_s: Optional[float] = None

    reader, type_map = open_bag(bag_dir)
    t0_ns: Optional[int] = None

    records: List[Record] = []
    truths: List[TruthSample] = []

    js_topic = "/joint_states"
    imu_topic = "/imu/data"
    truth_topic = "/odom_truth"

    for topic, msg, t_ns in iter_bag(reader, type_map):
        if t0_ns is None:
            t0_ns = t_ns
        t_rel = (t_ns - t0_ns) * 1e-9

        if topic == imu_topic:
            latest_imu = msg
            continue

        if topic == truth_topic:
            ot = msg
            R_t = quat_to_R(ot.pose.pose.orientation.x, ot.pose.pose.orientation.y,
                            ot.pose.pose.orientation.z, ot.pose.pose.orientation.w)
            p_t = np.array([ot.pose.pose.position.x, ot.pose.pose.position.y,
                            ot.pose.pose.position.z])
            v_t = np.array([ot.twist.twist.linear.x, ot.twist.twist.linear.y,
                            ot.twist.twist.linear.z])
            # Gazebo p3d publishes twist in the WORLD frame even though
            # child_frame_id=base_link. Honour the CLI flag to convert.
            if not truth_twist_in_world:
                v_t = R_t @ v_t
            truths.append(TruthSample(t=t_rel, p=p_t, R=R_t, v_world=v_t))
            continue

        if topic != js_topic:
            continue
        # --- JointState path: this is where one KF step happens ---------
        if latest_imu is None:
            continue

        # Build idx_in_v lazily on the first JointState that names all joints.
        if idx_in_v is None or len(msg.name) != idx_in_v.size:
            idx_in_v = -np.ones(nv, dtype=int)
            for k in range(nv):
                jname = model.names[k + 1]
                try:
                    idx_in_v[k] = msg.name.index(jname)
                except ValueError:
                    idx_in_v[k] = -1

        # Pack q, v, tau from the JointState in pinocchio order.
        q = np.zeros(model.nq)
        v = np.zeros(nv)
        tau = np.zeros(nv)
        pos = msg.position
        vel = msg.velocity
        eff = msg.effort
        for k in range(nv):
            i = int(idx_in_v[k])
            if i < 0:
                continue
            if i < len(pos):
                q[k] = pos[i]
            if i < len(vel):
                v[k] = vel[i]
            if i < len(eff):
                tau[k] = eff[i]

        # Compute dt the same way the C++ does: from JointState stamps.
        msg_t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.005
        if last_step_s is not None:
            d = msg_t_s - last_step_s
            if 0.0 < d < 0.1:
                dt = d
        last_step_s = msg_t_s

        # IMU -> R, omega, a_world.
        q_b_w = latest_imu.orientation
        R = quat_to_R(q_b_w.x, q_b_w.y, q_b_w.z, q_b_w.w)
        omega_b = np.array([latest_imu.angular_velocity.x,
                            latest_imu.angular_velocity.y,
                            latest_imu.angular_velocity.z])
        omega_w = R @ omega_b
        a_imu = np.array([latest_imu.linear_acceleration.x,
                          latest_imu.linear_acceleration.y,
                          latest_imu.linear_acceleration.z])
        a_world = R @ a_imu + gravity_world

        # Forward kinematics (and Jacobians for GRF synthesis).
        pin.computeJointJacobians(model, data, q)
        pin.forwardKinematics(model, data, q, v)
        pin.updateFramePlacements(model, data)

        # Synthesise per-foot GRF magnitude (mbf_se_bridge/grf_estimator).
        if gravity_compensate:
            pin.computeGeneralizedGravity(model, data, q)
            tau_eff = tau - data.g
        else:
            tau_eff = tau.copy()

        force_mags = np.zeros(4)
        for i, leg in enumerate(legs):
            J6 = pin.getFrameJacobian(model, data, leg.frame_id,
                                      pin.LOCAL_WORLD_ALIGNED)
            J3 = J6[:3, leg.idx_v]
            tau_leg = tau_eff[leg.idx_v]
            try:
                F = np.linalg.solve(J3.T, tau_leg)
            except np.linalg.LinAlgError:
                F = np.linalg.lstsq(J3.T, tau_leg, rcond=None)[0]
            force_mags[i] = float(np.linalg.norm(F))

        # contact_prob = sigmoid((||F|| - center) / scale), clamped.
        zs = np.clip((force_mags - fc_center) / fc_scale, -30.0, 30.0)
        contact_prob = 1.0 / (1.0 + np.exp(-zs))

        # LP filter contact_prob -> trust at this step's dt.
        alpha = dt / (max(1e-6, lp_tau) + dt)
        trust = (1.0 - alpha) * trust + alpha * contact_prob

        # Per-leg p_rel_world, v_rel_world (same formula as body_estimator).
        feet_obs: List[Tuple[np.ndarray, np.ndarray, float]] = []
        v_kin_world = np.zeros(3)
        w_sum = 0.0
        n_stance = 0
        for i, leg in enumerate(legs):
            r_b = data.oMf[leg.frame_id].translation
            v_rel_b = pin.getFrameVelocity(
                model, data, leg.frame_id, pin.LOCAL_WORLD_ALIGNED
            ).linear
            p_rel_world = R @ r_b
            v_rel_world = R @ v_rel_b + np.cross(omega_w, p_rel_world)
            feet_obs.append((p_rel_world, v_rel_world, float(trust[i])))

            # v_kin diagnostic uses RAW contact, not LP trust, so a leg that
            # just lifted does not contribute its swing velocity. This is
            # the change recommended in the live-log analysis.
            if contact_prob[i] > 0.5:
                w = float(contact_prob[i])
                v_kin_world += w * (-v_rel_world)
                w_sum += w
                n_stance += 1
        if w_sum > 1e-3:
            v_kin_world /= w_sum
        v_kin_body = R.T @ v_kin_world

        # KF predict + update.
        kf.step(dt, a_world, feet_obs)

        records.append(Record(
            t=t_rel,
            p=kf.x[0:3].copy(),
            v_body=R.T @ kf.x[3:6],
            v_kin_body=v_kin_body,
            n_stance=n_stance,
            a_world=a_world.copy(),
            raw=contact_prob.copy(),
            trust=trust.copy(),
        ))

    return records, truths


# ---------------------------------------------------------------------------
# Truth alignment + metrics.
# ---------------------------------------------------------------------------

def interp_truth(records: List[Record], truths: List[TruthSample]) -> dict:
    """Returns dict with truth arrays sampled at record times.

    Strategy:
      - Linearly interpolate truth position, world velocity, and yaw to
        each record timestamp.
      - To compare positions across frames (truth: world / est: odom),
        subtract the truth-vs-est offset at t=t_align (3 s in, to skip
        startup transients).
      - Body velocity is derived as R_truth^T @ v_world_truth.
    """
    if not records or not truths:
        return {}

    t_est = np.array([r.t for r in records])
    t_tr = np.array([s.t for s in truths])
    p_tr_w = np.array([s.p for s in truths])
    v_tr_w = np.array([s.v_world for s in truths])

    # Truth yaw (we only really need yaw for body-frame conversion of the
    # truth velocity; pitch/roll matter less if the robot stays upright).
    def yaw_from_R(R):
        return math.atan2(R[1, 0], R[0, 0])
    yaw_tr = np.array([yaw_from_R(s.R) for s in truths])
    # Unwrap so linear interpolation is meaningful across the ±pi jump.
    yaw_tr = np.unwrap(yaw_tr)

    def interp(col, target):
        return np.interp(target, t_tr, col)

    p_tr_at = np.column_stack([interp(p_tr_w[:, i], t_est) for i in range(3)])
    v_tr_at = np.column_stack([interp(v_tr_w[:, i], t_est) for i in range(3)])
    yaw_at  = interp(yaw_tr, t_est)

    # Convert truth world velocity to body frame using truth yaw (and assume
    # near-level: pitch/roll contribute < few % typically).
    cy, sy = np.cos(yaw_at), np.sin(yaw_at)
    v_tr_body = np.column_stack([
        cy * v_tr_at[:, 0] + sy * v_tr_at[:, 1],
       -sy * v_tr_at[:, 0] + cy * v_tr_at[:, 1],
        v_tr_at[:, 2],
    ])

    # Align positions by subtracting the truth-vs-est offset at t_align.
    # Use index instead of time so we don't fall off the end of short runs.
    align_idx = min(len(records) - 1, int(np.searchsorted(t_est, 3.0)))
    p_est_at = np.array([r.p for r in records])
    offset = p_tr_at[align_idx] - p_est_at[align_idx]
    p_tr_aligned = p_tr_at - offset

    return dict(
        t=t_est,
        p_tr=p_tr_aligned,
        p_est=p_est_at,
        v_tr_body=v_tr_body,
        v_est_body=np.array([r.v_body for r in records]),
        v_kin_body=np.array([r.v_kin_body for r in records]),
        raw=np.array([r.raw for r in records]),
        trust=np.array([r.trust for r in records]),
        n_stance=np.array([r.n_stance for r in records]),
        yaw=yaw_at,
        align_idx=align_idx,
    )


def compute_metrics(aligned: dict) -> dict:
    """RMSE on position and body velocity over the second half of the run.

    First half is dropped so startup transients (KF convergence + the
    first few seconds of alignment) don't dominate the score.
    """
    n = len(aligned["t"])
    half = n // 2
    p_err = aligned["p_est"][half:] - aligned["p_tr"][half:]
    v_err = aligned["v_est_body"][half:] - aligned["v_tr_body"][half:]
    rmse = lambda a: float(np.sqrt(np.mean(np.sum(a ** 2, axis=1))))
    rmse_x = lambda a, i: float(np.sqrt(np.mean(a[:, i] ** 2)))
    return dict(
        rmse_p_xy=float(np.sqrt(np.mean(np.sum(p_err[:, :2] ** 2, axis=1)))),
        rmse_p_z=rmse_x(p_err, 2),
        rmse_v_body_x=rmse_x(v_err, 0),
        rmse_v_body_y=rmse_x(v_err, 1),
        rmse_v_body_z=rmse_x(v_err, 2),
        n_steps=n,
    )


# ---------------------------------------------------------------------------
# Plotting.
# ---------------------------------------------------------------------------

def make_plots(aligned: dict, out_dir: Path, leg_names: List[str]):
    out_dir.mkdir(parents=True, exist_ok=True)
    t = aligned["t"]

    # 1. Body-frame linear velocity, truth vs est, with v_kin overlay.
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 7))
    for i, ax_name in enumerate(["x", "y", "z"]):
        ax = axes[i]
        ax.plot(t, aligned["v_tr_body"][:, i], label="truth", lw=1.2, color="k")
        ax.plot(t, aligned["v_est_body"][:, i], label="KF",   lw=1.0, color="C0")
        ax.plot(t, aligned["v_kin_body"][:, i], label="v_kin (raw stance)",
                lw=0.6, color="C3", alpha=0.5)
        ax.set_ylabel(f"v_body.{ax_name} [m/s]")
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle("Body-frame linear velocity")
    fig.tight_layout()
    fig.savefig(out_dir / "v_body.png", dpi=120)
    plt.close(fig)

    # 2. Position (after frame alignment).
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 7))
    for i, name in enumerate(["x", "y", "z"]):
        ax = axes[i]
        ax.plot(t, aligned["p_tr"][:, i], label="truth (aligned)", lw=1.2, color="k")
        ax.plot(t, aligned["p_est"][:, i], label="KF", lw=1.0, color="C0")
        ax.set_ylabel(f"p.{name} [m]")
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle("Position (truth aligned to KF at t≈3s)")
    fig.tight_layout()
    fig.savefig(out_dir / "p_world.png", dpi=120)
    plt.close(fig)

    # 3. Top-down xy trace.
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(aligned["p_tr"][:, 0], aligned["p_tr"][:, 1], "k-", lw=1.2, label="truth")
    ax.plot(aligned["p_est"][:, 0], aligned["p_est"][:, 1], "C0-", lw=1.0, label="KF")
    ax.set_aspect("equal", "datalim")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "xy_trace.png", dpi=120)
    plt.close(fig)

    # 4. Per-leg contact_prob vs trust.
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 8))
    for i, name in enumerate(leg_names):
        ax = axes[i]
        ax.plot(t, aligned["raw"][:, i],   lw=0.7, label="raw")
        ax.plot(t, aligned["trust"][:, i], lw=1.0, label="LP trust")
        ax.set_ylim(-0.05, 1.05)
        ax.set_ylabel(name)
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle("Per-foot contact probability vs LP-filtered trust")
    fig.tight_layout()
    fig.savefig(out_dir / "contacts.png", dpi=120)
    plt.close(fig)

    # 5. Stance count over time (sanity check on contact gating).
    fig, ax = plt.subplots(figsize=(10, 3))
    ax.plot(t, aligned["n_stance"], lw=0.6)
    ax.set_xlabel("t [s]"); ax.set_ylabel("# feet with raw>0.5")
    ax.set_ylim(-0.5, 4.5)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / "stance_count.png", dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Entry point.
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag_dir", type=Path)
    ap.add_argument("--config", type=Path,
                    default=Path("src/state_estimators/mbf_estimator/config/estimator.yaml"))
    ap.add_argument("--xacro", type=Path,
                    default=Path("src/mbf_description/xacro/robot.xacro"))
    ap.add_argument("--urdf", type=Path, default=None,
                    help="Optional pre-rendered URDF (skips xacro).")
    ap.add_argument("--out", type=Path, default=Path("/tmp/mbf_replay"))
    ap.add_argument("--override", action="append", default=[],
                    metavar="key=value",
                    help="Override estimator.yaml params; repeatable.")
    ap.add_argument("--truth-twist-in", choices=["world", "body"], default="world",
                    help="Frame of /odom_truth twist (Gazebo p3d uses world).")
    args = ap.parse_args()

    params = load_node_params(args.config)
    apply_overrides(params, args.override)

    urdf_xml = load_urdf(args.xacro, args.urdf)

    print(f"[replay] bag       = {args.bag_dir}")
    print(f"[replay] config    = {args.config}")
    print(f"[replay] overrides = {args.override}")
    print(f"[replay] out       = {args.out}")

    records, truths = run_replay(
        args.bag_dir, urdf_xml, params,
        truth_twist_in_world=(args.truth_twist_in == "world"),
    )

    if not records:
        print("[replay] no JointState messages produced KF steps; nothing to plot.",
              file=sys.stderr)
        sys.exit(2)

    aligned = interp_truth(records, truths)
    if not aligned:
        print("[replay] no truth samples to align; skipping plots.", file=sys.stderr)
        sys.exit(3)

    args.out.mkdir(parents=True, exist_ok=True)
    make_plots(aligned, args.out, params["leg_names"])

    metrics = compute_metrics(aligned)
    print("\n=== Metrics (second half of the run) ===")
    for k, v in metrics.items():
        print(f"  {k:18s} = {v}")
    with (args.out / "metrics.yaml").open("w") as f:
        yaml.safe_dump({"metrics": metrics,
                        "params": {k: params[k] for k in sorted(params)}}, f)
    print(f"\n[replay] wrote plots + metrics to {args.out}")


if __name__ == "__main__":
    main()
