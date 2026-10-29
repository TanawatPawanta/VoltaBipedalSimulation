# Simulation notebooks

`VoltaSimulation_lean.ipynb` is the main notebook. It builds the same closed loop as
`volta/experiments/runner.py`, but cell by cell, so you can change gains, wiring or plots and
re-run only what you need. The robot is shown in Meshcat at the URL printed by the first cell.

Other notebooks here are component tests and older versions, kept for reference:

| Notebook | Purpose |
|---|---|
| `test_ISMPC.ipynb`, `test_FootstepPlanner.ipynb`, `test_WBC.ipynb` | test the MPC, footstep planner and WBC on their own |
| `test_falling_object.ipynb`, `hydroelatic_contact.ipynb` | contact-model experiments |
| `OfflinePlanning.ipynb`, `CoMPlanning.ipynb`, `Init_planning.ipynb` | offline LIP / CoM planning |
| `VoltaSimulation.ipynb`, `VoltaSimulation_auto.ipynb`, `robot_simulation.ipynb` | earlier versions of the main notebook |

## Settings

The **Simulation parameters** cells:

| Variable | Meaning |
|---|---|
| `save_graph` | save logs (`.npz`) and figures (`.pdf`), see [Saving results](#saving-results) |
| `fixed_base` | weld the torso to the world, lifted by `fixed_base_offset` (0.5 m) |
| `gravity_enable` | `False` sets gravity to zero |
| `force_viz` | draw contact forces in Meshcat |
| `enable_gait_planner` | run the IS-MPC; `False` holds the initial references |
| `enable_noise` | add Gaussian noise to the measured robot state (`RobotStateNoiseGenerator`) |
| `exp_id` | experiment to load with `GetConfigByName` (floor, damping, velocity command, push, `sim_time`) |
| `sim_time` | simulated duration [s]; defaults to `exp_config.sim_time` |
| `set_initial_pose_by` | how the initial pose is found, see below |
| `plot_interval` | time window `(start, end)` [s] used by the plot cells |

Experiment ids (`volta/experiments/configs.py`): `0a`, `1a`–`1g` walking profiles,
`2a`–`2d` soft floors, `3a`–`3f` low contact dissipation, `4a`–`4b` mixed floors,
`5a`–`5b` joint damping, `6a`–`6b`, `7a`–`7d` external pushes.

### Initial pose modes

| `set_initial_pose_by` | Pose | LIP height `com_high` |
|---|---|---|
| `"c"` (recommended) | IK puts the CoM at 0.35 m above the left foot | 0.35 |
| `"h"` | IK puts the torso at 0.4 m | 0.22, **but the real CoM is ≈ 0.28 m** (see troubleshooting) |
| `"q"` | all joints at zero | not set by this branch |

`"c"` is the consistent choice: the runner (`COM_HEIGHT = 0.35`) and the initial estimate in
`StateEstimator` (`[0, 0, 0.35]`) both assume 0.35 m.

## Closed loop

```
                    ┌──────────── applied_spatial_force ◄── DisturbanceGenerator (push)
                    ▼
   ┌──────────► Drake plant ──state──► RobotStateNoiseGenerator ──noisy state──┐
   │              │  contact results                                           │
   │              ▼                                                            ▼
   │      ContactForceSensor ◄─────────────────────────────────── TaskSpaceFeedback
   │        │ ZMP      │ CoP in foot frames                         │ CoM   │ feet
   │        ▼          │                                            ▼       │
   │   StateEstimator (KF on LIP) ◄─── ZOH 1 ms ◄── u ──┐                   │
   │        │ filtered CoM, ZMP                         │                   │
   │        ▼                                           │                   │
   │   GaitPlanner (IS-MPC, every 10 ms) ───────────────┘                   │
   │        │ desired CoM / feet / contact                                  │
   │        ▼                                                               │
   └── PassivityBasedWBC2 ◄─────────────────────────────────────────────────┘
        joint torques
```

A second `TaskSpaceFeedback` on the clean state gives the ground-truth CoM for comparison.

Ports (from the class docstrings in `volta/Leaf_systems/`):

| System | Inputs | Outputs |
|---|---|---|
| `GaitPlanner` | 0 CoM state [12], 1 ZMP [3] | 0 desired CoM [18], 1 desired lfoot [12], 2 desired rfoot [12], 3 contact flags [2], 4 desired ZMP [3], 5 ZMP velocity u [3] |
| `StateEstimator` | 0 CoM state [12], 1 measured ZMP [3], 2 MPC input u [3] | 0 filtered CoM [12], 1 filtered ZMP [3] |
| `PassivityBasedWBC2` | 0 robot state [37], 1–3 desired CoM/lfoot/rfoot, 4 contact flags, 5–7 CoM/lfoot/rfoot state, 8 CoP in foot frames [4] | 0 joint torques [12], 1 desired foot wrenches [12], 2 Fz bounds + force impedance [5], 3 walking phase [1] |
| `ContactForceSensor` | 0 contact results, 1 robot state | 0 GRF per foot [12], 1 ZMP in world [3], 2 CoP in each foot frame [4], 3 total contact wrenches [12] |
| `TaskSpaceFeedback` | 0 robot state (q, v) | 0 CoM [12], 1 lfoot [12], 2 rfoot [12] |

12-vectors are `[rpy, xyz, angular vel, linear vel]`; 18-vectors add `[angular acc, linear acc]`.
So `data[3:6]` is position, `data[9:12]` linear velocity and `data[15:18]` linear acceleration.

## Logged signals

After the run, the first **Plot** cell turns the loggers into arrays shaped `(dims, samples)`:

| Variable | Source |
|---|---|
| `actual_CoM_state_data`, `actual_lfoot_state_data`, `actual_rfoot_state_data` | `TaskSpaceFeedback` on the noisy state |
| `groundtruth_CoM_state_data` | `TaskSpaceFeedback` on the clean state |
| `filtered_CoM_state_data`, `filtered_ZMP_data` | `StateEstimator` |
| `desired_CoM_state_data`, `desired_lfoot_state_data`, `desired_rfoot_state_data`, `contact_data`, `desired_zmp_data` | `GaitPlanner` |
| `GRF_data`, `actual_ZMP_data`, `ZMP_in_foot_frame_data`, `total_contact_wrenches_data` | `ContactForceSensor` |
| `torque_command_data`, `desired_foot_wrenchs_data` | `PassivityBasedWBC2` |
| `net_torque_data` | plant net actuation |

`walking_phase` is built from `contact_data`: 0 = left stance, 1 = right stance, 2 = double support.

## Saving results

With `save_graph = True`:

- logs → `Result/Final/Data/<exp id>[_noise].npz`
- figures → `Result/Final/Pics/<exp_name>[_noise]/`

Two cells write files whatever `save_graph` is set to:
`Result/Final/Data/Fz_with_ref.npz` (Fz comparison cell) and `Simulation/sigma_plot.png`
(the **Backyard** section). Running them changes tracked files.

## Troubleshooting

**`Opti::solve ... Solver failed ... 'maximum iterations reached'`**
The first MPC solve starts cold. At h = 0.35 OSQP needs about 3,900 iterations to converge;
later solves need about 25. The limit is set by `max_iter` in `volta/planning/ismpc.py`
(currently 10000). Set `"verbose": True` there to see the OSQP log.

**`... 'primal infeasible'`**
The QP has no solution. This happens when the CoM height fed to the MPC is more than about
`foot_size / 2` (0.04 m) away from `params['h']`. The vertical ZMP is boxed to ±0.04 m, so the
height can't be stabilised. A typical cause is `"h"` mode: `com_high = 0.22`, but the real CoM is
≈ 0.28 m and the estimator starts at 0.35 m. Use `"c"` mode, or set `com_high` to the real CoM
height of the pose.

**`ThrowIfInputAlreadyWired` when connecting the pusher**
Newer Drake (1.57 here) adds a Meshcat mouse-drag spring on the plant's `applied_spatial_force`
input. The notebook and runner turn it off with
`vis_config.mouse_interaction_stiffness = None` so `DisturbanceGenerator` can connect.

**`Warning: No data in interval (...)` or plot cells failing on short runs**
`plot_interval` must start before `sim_time`. The reference-velocity cell skips
`offset = 20000` samples (2 s), so it needs `sim_time > 2`.

**Slow runs**
`simulator.set_target_realtime_rate(0.15)` caps the speed, so a 15 s experiment takes at least
100 s. Set it to `0` to run as fast as possible.
