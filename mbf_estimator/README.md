# mbf_estimator

Linear Kalman filter base-state estimator for the MBF quadruped. Replaces
MUSE. One node, one algorithm, identical in Gazebo and on real hardware.

## What it does

Fuses two streams already published by the rest of MBF:

- `sensor_msgs/Imu` on `/imu/data` — base orientation, angular velocity,
linear acceleration. From `imu_node` on hardware, from the Gazebo IMU
plugin in sim.
- `sensor_msgs/JointState` on `/joint_states` — joint positions,
velocities **and efforts** (the same topic the controllers consume).
Effort is used to synthesise per-foot ground-reaction force in-process
via `F = J⁻ᵀ (τ − g(q))`, the same math `mbf_se_bridge/grf_estimator`
runs as a standalone node. Folding it into this node removes the
inter-process wrench round-trip — the force used by the KF in each
cycle was computed from the same cycle's `q`.

Publishes:

- `nav_msgs/Odometry` on `/state_estimator/odom`
- `geometry_msgs/WrenchStamped` on
`/state_estimator/contact_force_<lf|rf|lh|rh>_foot` — synthesised
per-foot ground-reaction force. Published for downstream consumers /
plotjuggler; the KF uses the values internally without going through
the topic.
- TF: `odom` → `base_link`

`mbf_se_bridge/grf_estimator` is still available as a standalone node
for topic-level debugging, but is **not required** at runtime. Running
both nodes simultaneously creates duplicate publishers on the wrench
topics — pick one.

## Filter

18-state linear KF, MIT Cheetah-Software formulation:

```
x = [ p_body(3) ; v_body(3) ; p_foot[0..3](3) ]   (all in world frame)
```

Body orientation is taken directly from the IMU and is **not** in the
filter state — the KF only has to solve for body position and velocity,
plus the world positions of each foot. Per foot in contact, three
measurement equations apply:

- Position: `p_foot - p_body == R * r_hip_to_foot(q)` (kinematic constraint)
- Velocity: `v_body == -(omega × R r + R J q_dot)` (foot stationary)
- Height:   `p_foot.z == 0` (flat-ground assumption; bounds vertical drift)

Per-foot contact probability (binary, Schmitt-triggered on
`||F||`) inflates the measurement noise on feet that aren't on the
ground, so swing legs don't pollute the body-velocity estimate.

## Limitations (shared with every IMU + leg-odometry filter)

- Yaw drifts unboundedly — there's no absolute heading reference. This is
fine for MPC (which only needs body velocity); not fine for global
navigation.
- `p_body` slowly drifts even in steady trot. Again fine for MPC.
- Assumes rigid, non-slipping contact. Failure modes on ice or thick
carpet are expected; tune `contact_force_on/off` accordingly.
- Assumes the IMU and base_link are mounted with the same orientation
(no IMU mounting offset compensation yet — easy to add later if needed).

## Bringup

```
ros2 launch mbf_estimator estimator.launch.py
```

That's the whole stack — no separate bridge needed.