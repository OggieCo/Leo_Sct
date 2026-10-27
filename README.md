# Socially Aware Multi-Robot Navigation in Human-Populated Environments

Code for my B.Sc. thesis at TU Darmstadt. Multiple Leo rovers navigate a shared,
human-populated arena in Ignition Gazebo while their navigation stacks are
compared under identical conditions.

The question is whether explicit social behavior — giving way, passing on the
proper side, keeping distance to people — scales to a small fleet, or whether a
plain point-to-point planner survives just as well.

## Compared navigation stacks

| Arm | Launcher | Navigation | Decision layer |
| --- | --- | --- | --- |
| **Nav2** | `--bt stock` | Nav2 bringup, stock `navigate_to_pose` tree | none |
| **BT** | `--bt reactive` | Nav2 + custom BehaviorTree.CPP tree (`config/bt/reactive_nav.xml`) | deterministic |
| **BT_LLM** | `--bt ai` | Nav2 + social tree (`config/bt/social_nav.xml`) | rule layer + an LLM supervisor (`llm_planner.py`, `gpt-4o-mini`, temperature 0) that picks one of seven high-level actions per tick; a deterministic guard can always override it |

All arms share the world, robot poses, goals, timing, and sensor setup, so the
comparison isolates the decision layer.

## What is in the loop

- **Perception** — YOLOv8n person detection on each rover's RGB-D camera, plus
  ground-truth peer poses for robot–robot state. No global state is used for
  navigation decisions.
- **Reactive shaping** — `velocity_adaptor.py` scales Nav2's velocity command by
  human and robot proximity terms, so the same planner behaves cautiously near
  people and near peers.
- **Social maneuvers** — `social_bt_plugins/` adds C++ tree nodes for
  human proximity, adaptive yielding, and speed limiting; `robot_proximity.py`
  and `llm_planner.py` track bearing, distance of closest approach, and
  right-of-way state.
- **Instrumentation** — every run writes per-rover CSVs: coverage and path
  length, goal outcomes, social events, robot–robot and robot–human contact.

## Requirements

- ROS 2 Humble and Ignition Fortress (developed inside a Docker container)
- `behavior_tree_cpp_v3` as used by Nav2
- Python: `ultralytics` (bundles torch), `numpy`, `opencv-python`
- `OPENAI_API_KEY` in the environment or in a gitignored `.env` (only for
  `--bt ai`; the planner calls the API over HTTPS, so no client library)
- GPU optional — YOLO runs on CPU by default

## Build

The repository is mounted as the workspace source folder:

```bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

A single experiment (starts the world, the rovers and the chosen stack, then
sends the goals and stops itself):

```bash
bash src/swarm_basics/scripts/start_experiment.sh \
     --bt ai --scenario 3_rovers_3_humans --goals fixed --run-time 300
```

Other useful options: `--headless` (no GUI), `--goals random|fixed|walk`,
`--scenario N` (1–14, see `worlds/scenario_generator.py`).

The full experiment matrix (arms × scenarios, sequential, resumable) runs
unattended:

```bash
bash src/swarm_basics/scripts/run_matrix.sh          # resume or start next pass
DRY_RUN=1 bash src/swarm_basics/scripts/run_matrix.sh # list the cells only
```

## Results

Runs land in `results/coverage_logs/<ARM>/<scenario>/batch_N/run_<timestamp>/`,
with `summary.csv`, `goal_stats.csv` and the per-rover event logs. The data is
not tracked in this repository — only the analysis scripts are:

| Script | Purpose |
| --- | --- |
| `build_run_metrics.py` | Aggregate all runs into one metrics table |
| `collision_exposure.py`, `collision_table.py` | Contact episodes per robot-hour, rates, fits |
| `count_bt_maneuvers.py`, `table_maneuvers.py` | Count reactive and social maneuvers |
| `llm_decision_audit.py`, `verify_llm_execution.py` | Audit LLM decisions against execution |
| `count_social_actions.py`, `plot_coverage_from_csv.py` | Social action counts, coverage plots |

## Repository layout

```
swarm_basics/        Main package: launch files, worlds, configs, nodes, scripts
  swarm_basics/      Nodes (planner, adaptor, perception, logging, tracking)
  config/            Nav2 params, BT trees, supervisor configs
  worlds/            Gazebo worlds and the scenario generator
  scripts/           Experiment driver, batch matrix, analysis
social_bt_plugins/   Custom BehaviorTree.CPP nodes (C++)
leo_image/           Image processing helpers (C++)
leo_common-ros2/     Leo Rover description, messages, teleop (upstream)
leo_simulator-ros2/  Gazebo/Ignition simulation packages (upstream)
results/             Run outputs (not tracked)
```

## Attribution

This repository is a fork of
[`ecemisildar/Leo_Sct`](https://github.com/ecemisildar/Leo_Sct), which provides
the multi-robot simulation setup this work builds on. The Leo Rover packages
(`leo_common-ros2`, `leo_simulator-ros2`, `leo_image`) originate there and from
the Leo Rover open-source project; their licenses are unchanged.

Everything under `swarm_basics/swarm_basics/`, most of `swarm_basics/launch/`,
`swarm_basics/config/`, `swarm_basics/worlds/`, `swarm_basics/scripts/` and all
of `social_bt_plugins/` was written for this thesis. The repository is MIT
licensed, see `LICENSE`.

## Contact

OggieCo — ognyan.laz@gmail.com
