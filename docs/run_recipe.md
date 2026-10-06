# Validated run recipe — multi-robot social navigation experiments

Measured on the thesis host (MSI Katana GF66 11UD, i7-11800H, RTX 3050 Ti 4 GB)
inside the `humble_thesis_engine` container (ROS 2 Humble + Ignition Fortress,
Ubuntu 22.04, Python 3.10).

The point of this document: **every setting below exists because its absence
caused a specific, reproducible failure.** A run that silently loses a robot is
almost always missing one of them.

---

## 1. Host preparation

```bash
sudo swarm_basics/scripts/host_cpu_cap.sh            # cap all cores at 2400 MHz
sudo swarm_basics/scripts/host_cpu_cap.sh --install-service   # survive reboots
```

**Why:** uncapped, this chassis reaches ~96 °C and throttles to 2.2–2.8 GHz with
large swings; under that load a robot's `slam_toolbox` init or Nav2 lifecycle
transition fails (measured: `robot_0_slam_toolbox` SIGSEGV exit -11 at 96 °C,
while the same boot from 76 °C passed 5/5). At a 2.4 GHz cap the clocks sit flat
at 2400 MHz and the package at 76–88 °C.

The active cap is printed in the startup log of every run, so a dataset collected
on a throttled host is identifiable after the fact.

## 2. Container prerequisites

```bash
pip install "setuptools>=77,<80"        # 79.0.1 verified; setuptools 83 breaks colcon
cd /root/ros2_ws && colcon build --packages-select swarm_basics
cd /root/ros2_ws/src/swarm_basics && colcon build      # BOTH trees
```

- `--symlink-install` is **not supported** by this package's `setup.py`.
- `launch/` and `config/` are installed as **real copies**: a launch-file edit
  does nothing until both trees are rebuilt. Python modules are imported from the
  src tree when running from `swarm_basics/`, so `.py` edits apply immediately.
- The fd limit and DDS transport are set inside `scripts/start_experiment.sh`, so
  no entry point can forget them; starting the container with
  `--ulimit nofile=65536:524288` is still recommended belt-and-braces.

## 3. The recipe

```bash
cd swarm_basics
python3 scripts/run_parallel.py --mode solo --bt reactive --goals fixed \
    --scenario 4_rovers_2_humans \
    --lidar-hz 5 --camera-hz 3 --camera-res 320x240 --clock-hz 50 \
    --dds-transports UDPv4 --detector-delay 200 --detector-wait-gate \
    --sim-budget 300 --standard-layout --tries 2 --cool-to 60
```

Run experiments through `run_parallel.py`; it injects the harness-only env and
retries a failed bring-up. The safety-critical defaults now also live in
`start_experiment.sh`, so `run_matrix.sh` and manual runs are no longer silently
unprotected.

## 4. Failure modes this recipe prevents

| # | Fault | Symptom observed | Fix |
|---|---|---|---|
| 1 | Sensor + YOLO CPU load | RTF 0.12, starved bring-up | YOLO on CUDA, reduced sensor rates/resolution |
| 2 | Thermal throttling (96 °C) | lifecycle transitions time out, SLAM segfault | 2.4 GHz cap (`host_cpu_cap.sh`) |
| 3 | `/clock` at 383 Hz (one Clock per physics step, bridged verbatim) | costmap clocks frozen at "0.7 s", slam_toolbox drops scans | `clock_throttle` relay republishes at 50 Hz |
| 4 | FastDDS SHM + fd exhaustion (301 segments vs `ulimit -n 1024`) | participants silently lose discovery, a *different* robot fails each run | UDPv4 transport + `ulimit -n 65536` |
| 5 | Bring-up contention (stall window < stagger; detectors firing mid-recovery) | healthy robots declared stalled, escalates to full restarts | 420 s window, 180 s stall for 4+, `YOLO_WAIT_GATE` (detectors load only after the gate) |
| 6 | Goal-phase logic | goals sent at 1 % mapped, aborted, retried forever | arena-window map metric, waypoint-steered scan rescue, 0.25 m/30 s progress checker |

## 5. Validated results (2026-09-23)

| Scenario | Bring-up | Sim time | Coverage | Path | Goals |
|---|---|---|---|---|---|
| 2 rovers / 2 humans, BT | clean | 376.5 s (RTF 0.97) | 47.45 % | 167.5 m | 17/17 + 23 scan goals, 1 bump |
| 4 rovers / 2 humans, BT | 4/4 in 65 s, 0 recovery | 473.7 s | 47.96 % | 236.6 m | 25 reached / 3 failed, 23 bumps |

Bring-up times are measured from launch to the `READY` line; coverage is the
fraction of arena cells observed by the fleet's maps.

## 6. Known limitation

The 5-rover arm still shows an intermittent **"one laggard robot"** race: during
simultaneous bring-up one robot's lifecycle activation can fail even when the
preceding fixes are all in place. The recovery ladder now (a) refuses to relaunch
a stack whose old processes have not died (duplicate node names cannot complete a
bring-up), (b) restarts a dead sim-side scan republisher when the probe reports a
robot with no LiDAR scan at all, (c) uses 60 s bond / 15 s service timeouts, and
(d) adds a final single-robot surgical rung before the attempt is abandoned.

For data collection, prefer 2–4 rovers: those configurations boot reliably and
yield the same social-navigation measurements. Treat 5-rover bring-up as
environment-limited on laptop-class hardware, and let `--tries` re-attempt.
