# Volta Bipedal Simulation

Walking simulation and whole-body control for the Volta biped, built on [Drake](https://drake.mit.edu/).

## Overview

The closed loop runs in Drake with compliant point contact:

- **Gait planner**: an intrinsically stable MPC (IS-MPC) on the linear inverted pendulum (LIP) that
  plans footsteps, CoM and ZMP from a velocity command (solved with CasADi + OSQP every 10 ms).
- **State estimator**: a Kalman filter on the LIP model that fuses the measured CoM and ZMP.
- **Whole-body controller**: a passivity-based WBC that tracks the CoM and swing-foot references,
  distributes contact wrenches between the feet and outputs joint torques.
- **Experiments**: walking patterns, soft and mixed floors, low contact dissipation, joint damping,
  walking speed and external pushes, each with optional sensor noise.

Robot models: Volta rev5 (used by the experiments), Volta rev4, and `hanuman04` (an older ROS 2 package).

## Repository layout

```
volta/                  Python package
  Leaf_systems/         Drake LeafSystems (GaitPlanner, StateEstimator, PassivityBasedWBC2, ContactForceSensor, …)
  planning/             IS-MPC, footstep planner, foot trajectory generator, offline planning
  control/              contact wrench solver, stance-leg state machine
  estimation/           Kalman filter
  robot/                kinematics, dynamics, state containers
  experiments/          experiment configs, runner, scene descriptions, velocity profiles
  plotting.py           tracking / CoM-ZMP / footstep plots
  prelude.py            one-line import for notebooks: from volta.prelude import *
Models/                 URDF / xacro, meshes and floor SDFs
Simulation/             notebooks (main: VoltaSimulation_lean.ipynb), see Simulation/README.md
Visualization/          model viewer notebooks and LIP phase portrait
M_result/               saved logs, figures and Meshcat recordings (Final/ and Progress/)
```

## Installation

Python 3.12 on Linux (tested with Drake 1.57).

```bash
git clone <repo-url> VoltaBipedalSimulation
cd VoltaBipedalSimulation
python3 -m venv .venv
source .venv/bin/activate
pip install drake numpy scipy matplotlib casadi python-statemachine pydot ipython ipykernel
```

Optional:

- `plotly`, only for `plot_interactive_comparison` in `volta/plotting.py`
- `xacrodoc`, only for regenerating URDFs with `Models/create_urdf.py`

There is no install step for `volta` itself. Run from the repository root (or put it on
`PYTHONPATH`) so `import volta` resolves; all model and result paths are resolved from
`volta/paths.py`, not from the working directory.

## Quick start

**Notebook.** Open `Simulation/VoltaSimulation_lean.ipynb`, set `exp_id` in the
*Simulation parameters* cell and run all cells. The robot is shown in Meshcat at the URL printed
by the first cell.

**Script.**

```python
from volta.experiments.runner import run_experiment

run_experiment("1a", save_graph=True)
```

Useful keyword arguments: `enable_noise` (default `True`), `show_plot`, `fixed_base`,
`set_initial_pose_by` (`"c"` recommended) and `realtime_rate` (default 0.15; `0` runs as fast as
possible).

## Experiments

Defined in `volta/experiments/configs.py` and loaded with `GetConfigByName("<id>")`.
Defaults: floor stiffness 5e6 N/m, dissipation 50 s/m, joint damping 0.3, walking speed 0.1 m/s.

| Id | Name | What changes |
|---|---|---|
| `0a` | stabilization | standing, push of [12, 10, 15] N at 1 s |
| `1a`–`1g` | walking patterns | straight, diagonal, circular, rotate in place, L-shape, combined, forward-backward |
| `2a`–`2d` | soft floor | stiffness 5e3 (`2a`, `2b`) and 2e3 (`2c`, `2d`), combined and L walks |
| `3a`–`3f` | low dissipation | dissipation 30 / 20 / 10 on a soft floor; `3d`–`3f` short straight walks. `3f` uses the same config as `3e`: for this run the stance-leg stabilizer was turned off by hand in `PassivityBasedWBC2.py` (`K=0.0`) |
| `4a`–`4b` | mixed floor | tiled terrain with different stiffness / dissipation per tile; `4b` all stiff |
| `5a`–`5b` | joint damping | damping 0.2 and 0.1 |
| `6a`–`6b` | walking speed | 0.15 and 0.2 m/s |
| `7a`–`7d` | external push | push at 3 s: Fx, Fy, Fx + yaw torque, none (baseline) |

## Results

With `save_graph=True`, each run writes:

- **Logs** to `M_result/Final/Data/<exp id>[_noise].npz`: timestamps, desired / raw / filtered /
  ground-truth CoM, desired and actual feet, desired / raw / filtered ZMP, walking phase, ground
  reaction forces, joint torque commands and desired foot wrenches.
- **Figures** to `M_result/Final/Pics/<exp_name>[_noise]/`.

The figures for each experiment:

| Group | Figures |
|---|---|
| State estimation | filtered vs raw vs ground-truth CoM position; same for CoM velocity; filtered vs raw ZMP |
| CoM | CoM orientation tracking; CoM position tracking (with error); planned CoM acceleration |
| Velocity command | reference velocity (Vx, Vy, ω) vs CoM velocity |
| Feet | left / right foot orientation tracking; left / right foot position tracking (with error) |
| ZMP | planned ZMP-CoM; ZMP tracking (desired vs actual); actual ZMP-CoM |
| Top view on footstep plan | tracking performance (all four trajectories); actual CoM and desired ZMP; ZMP tracking performance; planned CoM and ZMP |

Time-series plots shade the walking phase (left stance, right stance, double support).

Other result folders:

- `M_result/Final/Record/`: Meshcat recordings of the runs as standalone `.html` files; open one in a browser to replay the 3D animation.
- `M_result/Final/Script/plot.ipynb`: plots across experiments built from the saved `.npz` logs.
- `M_result/Progress/`: figures from earlier development stages (stabilizing controller, orientation
  controller, solver tests, static walking).

## Further documentation

[`Simulation/README.md`](Simulation/README.md) covers the notebook parameters, initial-pose modes,
the closed-loop diagram with port tables, the logged signals and troubleshooting
(MPC solver failures, Meshcat wiring, slow runs).
