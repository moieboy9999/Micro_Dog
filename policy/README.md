# Walking policy (`micro_dog_walk.pt`)

This is the velocity-tracking walking policy that runs on the author's Micro Dog, on gamepad button A
(deployed 2026-09-20). It was trained in Isaac Lab with RSL-RL (PPO) and walked on the real robot.

| File | Contents |
|---|---|
| `micro_dog_walk.pt` | RSL-RL checkpoint, update 30000: actor, critic, optimizer state, curriculum state |
| `micro_dog_walk.json` | Deployment contract: observation layout, action formula, joint order, limits, hardware settings |

## Network

- **Actor:** MLP 45 → 512 → 256 → 128 → 12, ELU activations, in `actor_state_dict` (`mlp.0/2/4/6`).
  It has no observation normalisation and no history.
- **Output:** the deterministic action is the MLP output. `distribution.log_std_param` is the training
  exploration noise; ignore it at deployment.
- **Timing:** it runs at 50 Hz (`step_dt` 0.02 s).

## Observation (45 values, raw SI units, no scaling)

| Index | Term | Meaning |
|---|---|---|
| 0–2 | `base_ang_vel` | IMU gyro, body frame, rad/s |
| 3–5 | `projected_gravity` | gravity unit vector in the body frame (complementary filter, α = 0.01) |
| 6–8 | `velocity_commands` | [vx m/s, vy m/s, yaw rate rad/s] |
| 9–20 | `joint_pos` | joint angle **minus** `default_joint_pos`, rad |
| 21–32 | `joint_vel` | joint velocity, rad/s |
| 33–44 | `actions` | previous raw policy output |

**Joint order is grouped by joint type, not by leg.** It is the `joint_names` list in the JSON:

```
FL_hip  FR_hip  RL_hip  RR_hip  FL_thigh  FR_thigh  RL_thigh  RR_thigh  FL_calf  FR_calf  RL_calf  RR_calf
```

This differs from the per-leg order of the URDF, so map by name.

## Action

```
target = clip(raw_action * 0.25 + default_joint_pos, clip_low, clip_high)     # rad, joint_names order
```

`default_joint_pos`, `clip_low` and `clip_high` are in the JSON. Send `target` to the servos as
position setpoints.

## Hardware assumptions

- **Servos:** XL330-M288-T in position mode, firmware P gain 200. Profile velocity is 843 and
  profile acceleration 0, so the servo's own velocity profile shapes the step. The host sends raw
  targets.
- **Angle mapping:** servo tick ↔ joint angle uses `docs/specs.md` §4.
- **Training commands:** the curriculum's final stage covered |vx| ≤ 0.70 m/s, |vy| ≤ 0.50 m/s and
  |yaw rate| ≤ 1.8 rad/s. Speeds validated on hardware are lower. Start slow.
- **Physics:** the simulated actuator, IMU and friction models it was trained against are described in
  the JSON (`physics_contract`, `imu_contract`). The training environment code is not included.
- **Torque limit:** training used a PWM/voltage torque model with an effort limit of 0.549 N·m
  (`physics_contract.effort_limit_nm`). That is stricter than the 0.6405 N·m on the URDF joints, which
  is the plain rigid-body description.

## Safety

- This is a learned controller for a small robot.
- Test with the robot lifted, then at low commanded speed.
- Keep a torque-off button within reach.
- The servos' current-limit register is not enforced in position mode (see `physics_contract`).

License: CC BY-NC-SA 4.0, like the rest of the design files.
