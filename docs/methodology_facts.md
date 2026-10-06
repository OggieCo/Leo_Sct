# Methodology fact sheet (extracted from the code, 2026-09-23)

Every number here is read from the implementation, not remembered. Use it as the
source of truth while writing Chapter 3; anything marked **VERIFY** needs one more
look before it goes in print, and anything marked **YOU** is your decision.

---

## 0. Verdict on the current outline

Your eight sections describe the **system design**, and they do it in the right
order (perception → symbolic state → behaviour → integration → scale). Two problems:

1. **The methodology half is missing.** As it stands the chapter says *what was
   built*, not *how it was tested*. A methodology chapter also needs: experimental
   design (conditions, scenarios, repetitions), metrics with definitions, the
   experimental protocol, and threats to validity. An examiner will look for those
   headings.
2. **There is no LLM section** — and LLM-grounded social navigation is your stated
   contribution. Right now the chapter would describe a reactive behaviour tree and
   never explain the decision layer.

### Proposed restructure

| New section | Source | Notes |
|---|---|---|
| 3.1 System Overview | yours | keep; add the three behaviour conditions here |
| 3.2 Platform and Simulation Environment | **new** | host, container, simulator, sensors, robot — the config table |
| 3.3 Perception Pipeline | yours | LiDAR + YOLO, exact parameters |
| 3.4 Symbolic State Extraction | yours | state variables, thresholds, rates |
| 3.5 Social Behaviour Tree Design | yours | tree structure, timings, custom plugins |
| 3.6 Social Semantics and the LLM Decision Layer | yours ("Social Semantics") **+ LLM** | action space, rules, prompt, invocation policy, cost |
| 3.7 Integration with Nav2 | yours | costmaps, controller, velocity chain, frames |
| 3.8 Multi-Robot Extension | yours | per-robot stacks/maps, TF, proximity layer, bring-up |
| 3.9 Experimental Design | **new** | 3 arms × scenario matrix, goal protocol, repetitions |
| 3.10 Metrics and Instrumentation | **new** | every metric: definition + artifact |
| 3.11 Experimental Protocol and Reproducibility | **new** | run recipe, gates, validity checks, exclusions |
| 3.12 Design Rationale and Alternatives Considered | yours | keep — examiners like it; this is where rejected designs go |
| 3.13 Threats to Validity | **new** | honest limitations before anyone else raises them |

If that is too long for your page budget: merge 3.2 into 3.1 and 3.11 into 3.9.
Do **not** drop 3.9–3.11 or 3.13 — those are what make it a methodology chapter.

---

## 3.2 Platform and Simulation Environment

| Item | Value |
|---|---|
| Host CPU | Intel Core i7-11800H (8C/16T), capped at **2400 MHz** for thermal stability |
| Host GPU | NVIDIA RTX 3050 Ti, 4 GB (YOLO inference: CUDA) |
| Runtime | Ubuntu 22.04 container: **ROS 2 Humble** + **Ignition Fortress** (Gazebo 6), Python 3.10 |
| Simulator step | physics clock published natively at physics rate; relayed to `/clock` at **50 Hz** |
| Sensor rates (frozen config) | 2D LiDAR **5 Hz**, RGB-D **3 Hz @ 320×240** |
| DDS | FastDDS, **UDPv4-only transport**, fd limit raised to 65536 |
| ROS domain | 31 |
| Thermal precondition | run starts only when package temperature ≤ 60 °C |
| Run harness | `scripts/run_parallel.py` → `scripts/start_experiment.sh`; one run = one results folder |

**VERIFY:** exact host RAM and container image tag for the table.

## Robot platform and scenarios

| Item | Value |
|---|---|
| Robot | Leo Rover (differential drive), `leo_common-ros2` description |
| Footprint | circular, radius **0.30 m** |
| Sensors | 2D LiDAR (scan range ~10 m), RGB-D camera, bumper contact sensor |
| Speed limits | `max_vel_x = 0.5 m/s`, `max_vel_theta = 1.0 rad/s` |
| Velocity chain | controller → `/{ns}/cmd_vel_social` → `velocity_adaptor` (social scaling) → `/{ns}/cmd_vel` → Gazebo |
| Arena | 14 × 14 m, walls on all sides; coverage grid 14 × 14 = **196 cells of 1 m** |
| Rover spawns | distinct arena corners, each facing the arena centre |
| Humans | kinematic Gazebo actors (`walk.dae`), lane span ≤ 7 m, walk speed ≤ 0.5 m/s, **bumpable** (contact sensors) |
| Scenario matrix | {2,3,4,5} rovers × {1,2,3} humans + `3_rovers_and_obstacles` + `5_rovers_and_obstacles` (boxes + cylinders) |

## 3.3 Perception Pipeline

**LiDAR path:** Gazebo scan → `lidar_republish` (rewrites Gazebo's mangled frame id to
`{ns}/lidar_link`) → `/{ns}/lidar/scan_clean` → SLAM **and** both costmaps.

**Human detection (`yolo_human_processor.py`):**

| Parameter | Value |
|---|---|
| Model | Ultralytics **YOLOv8n**, COCO class 0 (person) |
| Inference size / confidence | `imgsz = 640`, `conf = 0.4` |
| Rate limit | one inference per **0.33 s** (≈3 Hz), stale frames dropped |
| Range | depth at box centre (5-px cross, then median over box); valid 0.2–50 m |
| Bearing | pinhole: `fx = (w/2)/tan(hfov/2)`, `hfov = 1.51844 rad` (≈87°); `angle = -(cx - w/2)/fx`, **+ = left** |
| Representative person | largest-area box = nearest human (multi-human scenes sampled at the nearest) |
| `human_close` | `0 < range < 1.5 m` |
| Detection hold | 1.0 s on dropout |
| Robot-as-person suppression | if nearest robot < **6.0 m** and bearing differs by < **40°** → report `ROBOT_NOT_HUMAN` (no human) |
| Outputs (namespaced) | `human_detected` (Bool), `human_close` (Bool), `human_distance` (Float32), `human_angle` (Float32), `human_info` (String) |

## 3.4 Symbolic State Extraction

| State | Definition | Threshold / note | Source node |
|---|---|---|---|
| `human_detected` | a person is currently in view | Conf ≥ 0.4, depth valid | YOLO detector |
| `human_close` | person within stopping-relevant distance | < 1.5 m | YOLO detector |
| `human_distance`, `human_angle` | range and signed bearing to nearest person | ±87° FOV | YOLO detector |
| `robot_close` | another rover within interaction distance | proximity layer, 30° cone | `robot_proximity` |
| `robot_angle`, `nearest_robot_dist`, `robot_dca` | bearing, range, distance of closest approach to nearest rover | — | `robot_proximity` |
| `obstacle_min` | nearest LiDAR return | min over scan | costmap / scan |
| `speed_scale` | social speed multiplier applied to the controller output | ≤ 1.0 | `velocity_adaptor` |

State is refreshed continuously and logged at ~4 Hz to `social_state.csv`.

## 3.5 Social Behaviour Tree Design

Structure (stock Nav2 nodes + BT.CPP built-ins + four custom condition/action nodes):

```
RecoveryNode(retries=6) "NavigateRecovery"
└── PipelineSequence "NavigateWithReplanning"          # correct 1 Hz replanning
    ├── RateController(hz=1.0)
    │   └── RecoveryNode(retries=1) → ComputePathToPose / ClearEntireCostmap
    └── RecoveryNode(retries=1) "SocialFollow"
        └── ReactiveFallback "HumanAwareFollow"
            ├── Timeout(4000 ms)   → keep yielding to a human, give up after 4 s
            └── ReactiveFallback "RobotAwareFollow"
                ├── Timeout(10000 ms) → keep yielding to a rover
                └── ReactiveFallback "ArcOrFollow"
                    └── Arc(adaptive, 0.3 m/s, 6 s, direction=away) → FollowPath
```

Design points worth stating explicitly (they are results of your own engineering):

- `RateController(1.0)` + `PipelineSequence`: a `SequenceStar` root flooded
  `ComputePathToPose` and caused planner timeouts — documented, citable rationale.
- Yields are **bounded** (4 s human / 10 s rover) so a blocked robot resumes rather
  than deadlocking.
- Nested `ReactiveFallback` nodes with **at most two async children each**: a single
  fallback with three async children stopped re-ticking the yield branches while
  `FollowPath` was active (observed 2026-08-17).
- Maneuver is a **continuous arc**, never an in-place spin (adhesive wheels).
- Custom nodes in `social_bt_plugins/`: `IsHumanClose`, `IsRobotClose` (also sets
  `{robot_avoid}`, head-on tie-break, arc gating), `IsRearClear` (rear clearance from
  `/lidar/scan_clean`), `CheckLlmAction` (LLM → branch), `Arc`.

**VERIFY:** enumerate the plugin list from `social_bt_plugins/CMakeLists.txt` and
`plugin_lib_names` so the appendix matches the code exactly.

## 3.6 Social Semantics and the LLM Decision Layer

| Item | Value |
|---|---|
| Model | `gpt-4o-mini` (hosted), JSON-only response |
| Action space | `proceed, slow, yield, arc_left, arc_right, stop, back` |
| Invocation policy | **event-driven**: one call when the situation signature changes (not per tick) |
| Input | structured situation report: nearest human (range, bearing, range-rate, close?), nearest rover (range, bearing, speed, right-of-way), free space left/right/back, wall proximity |
| Output | `{"action": ..., "reason": ...}` — the reason string is logged verbatim |
| Rules encoded in the prompt | humans always have priority; **right-of-way locked for the whole encounter** (right-hand traffic); no in-place rotation; adaptive passing (arc only with room, `back` only for a genuine deadlock with a clear rear); stall escape only for the yielding robot |
| Logging | `llm_usage.csv` (per call: action, reason, in/out/total tokens, cost) + `llm_summary.csv` (per robot totals) |
| Measured cost | **$0.011–0.013 per rover per 300 s run**, 38–401 calls per run depending on fleet size |

The full system prompt should be reproduced **verbatim** in an appendix (it is a
designed artifact and central to your contribution — copy it from
`swarm_basics/swarm_basics/llm_planner.py`).

## 3.7 Integration with Nav2

| Setting | Value |
|---|---|
| Planner / controller | `NavfnPlanner` (`allow_unknown = True`), `DWBLocalPlanner` |
| Costmap resolution | 0.05 m, robot radius 0.30 m |
| Inflation | global **0.25 m**, local **0.7 m** |
| Progress checker | required movement **0.25 m in 30 s** (relaxed: the social layer deliberately pauses, and a stricter checker aborted goals whenever the robot yielded) |
| Goal tolerance | xy 0.25 m, yaw 3.14 rad (heading-agnostic arrivals) |
| Lifecycle manager | wall clock (`use_sim_time = false`) + `bond_timeout` 60 s, `service_timeout` 15 s |
| Social speed control | `velocity_adaptor` scales the controller's `cmd_vel` by `speed_scale` (human/robot proximity terms) |

## 3.8 Multi-Robot Extension

- **Per-robot maps, no merging:** each rover runs its own `slam_toolbox` publishing
  `/{ns}/map` and the `{ns}/map → {ns}/odom` transform; each Nav2 stack is namespaced
  and plans in its own map frame. Viz-only static transforms align all map frames to
  `robot_0/map` for a single RViz view (they do not affect navigation).
- **Perception sharing:** none — each rover perceives humans and other rovers with its
  own sensors; coordination is purely local (reactive BT rules + the LLM's
  right-of-way protocol).
- **Bring-up orchestration:** the stacks are started **one robot at a time**; a robot
  is accepted only when its `bt_navigator` is lifecycle-active *and* its map topic is
  publishing *and* its scan is flowing. Failures are retried per robot, so one flaky
  stack cannot poison the boot. Detector models load only after the gate signal.
- **Why:** with simultaneous construction the last stack's lifecycle manager aborted
  its own activation under load (measured, 2026-09-23).

## 3.9 Experimental Design — **YOU**

This section needs your research question written out first; the implementation
already supports exactly three arms, which map onto the obvious comparison:

| Arm | Command | Description |
|---|---|---|
| **Nav2** (baseline) | `--bt stock` | stock Nav2, no social layer, no human awareness |
| **BT** | `--bt reactive` | reactive social behaviour tree |
| **BT+LLM** | `--bt ai` | same tree + LLM decision layer |

Conditions = arm × scenario (matrix in 3.2). Goal protocol: each rover receives a
fixed tour of waypoints, plus **scan goals** that expand the map until 30 % of the
arena is known. Nominal goal phase 300 s.

**YOU:** state the hypotheses (H1 social layer vs Nav2; H2 LLM vs reactive BT),
the number of repetitions per cell, and whether the two 5-rover-obstacle scenarios
are part of the design or appendix material.

## 3.10 Metrics and Instrumentation

| Metric | Definition | Artifact |
|---|---|---|
| Coverage | visited cells / 196, on a 1 m grid over the arena | `coverage_final.csv`, `coverage_map.png` |
| Path length | integrated odometry per rover (m) | `trajectories.csv`, `summary.csv` |
| Goals | sent / succeeded / failed / scan per rover | `goal_stats.csv` |
| Bumps | contact episodes between a rover and any object (human, rover, wall) | `events.csv` (`CONTACT_BEGIN/END`), `social_events.csv` |
| Social events | `BACKWARD_START/END` (yield/back-off), `ROBOT_NEAR/GONE`, `OBSTACLE_NEAR`, `BT` | `social_events.csv`, `social_summary.csv` |
| Human interaction | detection samples, `human_close` episodes, distance/bearing distributions | `social_state.csv` (~4 Hz) |
| LLM behaviour | action mix, reason strings, tokens, cost | `llm_usage.csv`, `llm_summary.csv` |
| Run integrity | duration (sim/wall), RTF, gate time, detector-load confirmation | harness log, `summary.csv` |

Suggested derived quantities (state them in the text): goal success rate, coverage
per metre travelled, yield count per encounter, mean speed while moving, minimum
recorded human distance, LLM cost per completed goal.

## 3.11 Experimental Protocol and Reproducibility

1. Host precondition: package ≤ 60 °C, all cores capped at 2.4 GHz.
2. Launch through the harness (never a bare `ros2 launch`) so the fd limit,
   UDP-only DDS, clock relay, sensor rates and detector gate are all applied.
3. Bring-up: sequential, with the readiness gate described in 3.8; the run proceeds
   only when every robot passes.
4. Validators (state these as inclusion criteria): the launcher logs
   `Human detectors loaded YOLO for all robots`; the gate has passed; the run reaches
   its full duration without a manual stop.
5. Artifacts per run: the CSVs in 3.10 plus `coverage_map.png`, all in one folder
   named `batch_N_run_<timestamp>`.
6. **Era declaration — YOU must decide this.** Runs made before 2026-09-23 used
   10 Hz LiDAR / 5 Hz camera @ 640×480 and an unthrottled `/clock` (383 Hz); runs
   from 2026-09-23 onward use 5/3 Hz @ 320×240 and a 50 Hz clock relay, plus relaxed
   progress checking. Pooling them silently is a validity problem. Options: re-run the
   grid, re-run a reduced headline grid, or declare the earlier runs as pilot data.

## 3.13 Threats to Validity

- **Simulation only**; no real-robot trials (state the porting target, e.g. Jetson
  Orin Nano, and what would change).
- **Humans are kinematic and non-reactive**: they follow fixed lanes at ≤ 0.5 m/s,
  do not avoid robots, and can be bumped. Consequently "social" results concern the
  *robot's* behaviour toward people, not mutual interaction. State this explicitly.
- **Perception limits**: one class, nearest-person representation, ~3 Hz update,
  87° FOV, depth-derived range; a person standing next to a rover can be suppressed
  as `ROBOT_NOT_HUMAN`.
- **LLM**: hosted model version is outside your control (record the model name and
  date); sampling is event-driven, so call counts vary with scenario activity; the
  prompt is hand-designed — the appendix must reproduce it verbatim.
- **Environment**: single host, thermally capped CPU; RTF < 1 in crowded scenarios;
  results are hardware-dependent.
- **Bring-up determinism**: multi-robot startup can still require a retry on the
  5-rover arm (one robot's lifecycle activation); runs are accepted only if the gate
  passes, so this affects yield, not correctness.
- **Metric dependence**: coverage is bounded by SLAM map quality, which depends on
  LiDAR range and the scan tour; bumper contacts depend on actor physics.
