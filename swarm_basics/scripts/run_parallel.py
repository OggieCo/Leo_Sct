#!/usr/bin/env python3
"""run_parallel.py — run Leo_Sct swarm simulations on THIS workstation.

Built for the MSI Katana GF66 11UD (Intel i7-11800H: 8 cores / 16 threads,
no E-cores; Intel UHD + NVIDIA RTX 3050 Ti).  Two modes:

  solo   one heavy run, using 7 physical cores + the GPU, tuned for a high
         real-time factor.  Pins the whole stack, puts YOLO on the CUDA GPU,
         caps per-worker torch threads, and samples RTF live.

  slots  N independent simulations in PARALLEL, each fully isolated:
         its own ROS_DOMAIN_ID, Ignition partition, CPU pair, ROS log dir and
         results folder — so several scenarios can be collected at once.

Every invocation starts FRESH runs and never reuses older results; if you do
want to skip scenarios that are already complete, opt in with --resume.

Why these knobs matter here (measured on this box):

  * YOLO/torch defaults to 8 CPU threads per worker.  Five rovers => 40
    threads on 16 => context-switch thrash that starves SLAM/Nav2 and tanks
    the real-time factor.  So each worker is capped (--yolo-threads, default
    2) and inference is moved to the GPU (--yolo-device cuda).
    * Gazebo renders its sensors headless through EGL.  The container ships
    only a Mesa vendor file, and Mesa REJECTS this laptop's Intel iGPU
    ('Driver does not support the 0x25a0 PCI ID' -> 'egl: failed to create
    dri2 screen'), i.e. sensor rendering fell back to software (llvmpipe) and
    stole CPU from physics.  --render defaults to nvidia, which creates the
    missing glvnd vendor file on first use and renders on the RTX 3050 Ti:
        nvidia -> RTX 3050 Ti via glvnd (DEFAULT, the only working path here)
        intel  -> Mesa/iris on the Intel iGPU (broken in this container)
        auto   -> nvidia, then intel as a fallback
    Note YOLO also uses the dGPU (CUDA), so rendering + inference share the
    RTX 3050 Ti; if a 5-rover GUI run gets choppy, raise --yolo-period.
  * Results go to results/coverage_logs_parallel/<slot>/run_<ts>/ via
    SWARM_RUN_DIR, so experimental data never mixes with the thesis dataset.

Must run INSIDE the container (as root):

    cd /root/ros2_ws/src/swarm_basics
    python3 scripts/run_parallel.py --mode solo --scenario 5_rovers_2_humans \
        --bt ai --goals fixed --run-time 300

The script sources ROS + the workspace itself, so no setup is needed first.
"""

from __future__ import annotations

import argparse
import fcntl
import os
import re
import signal
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

# --------------------------------------------------------------------------
# Paths / fixed environment (container layout)
# --------------------------------------------------------------------------
PKG_DIR = Path('/root/ros2_ws/src/swarm_basics')
START_SCRIPT = PKG_DIR / 'scripts' / 'start_experiment.sh'
RESULTS_ROOT = PKG_DIR / 'results' / 'coverage_logs_parallel'
LOG_ROOT = PKG_DIR / 'scripts' / 'parallel_logs'

ROS_SETUP = ('source /opt/ros/humble/setup.bash && '
             'source /root/ros2_ws/install/setup.bash')

# --------------------------------------------------------------------------
# CPU topology — Intel i7-11800H (8C / 16T, all P-cores, no E-cores)
#
# Physical core k owns logical threads {2k, 2k+1}, so a slot pinned to one
# core never shares it with another slot.  Threads 12-15 (cores 6-7) are left
# free for the OS, the GPU driver and the DDS daemon — oversubscribing every
# core is what makes Gazebo stutter.
# --------------------------------------------------------------------------
SLOT_CPUS: tuple[tuple[int, ...], ...] = (
    (0, 1), (2, 3), (4, 5), (6, 7), (8, 9), (10, 11),
)
MAX_SLOTS = len(SLOT_CPUS)
SOLO_CPUS: tuple[int, ...] = tuple(range(0, 14))   # 7 cores, 14-15 reserved

_MISSING_ICD = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
_NVIDIA_ICD_JSON = ('{\n    "file_format_version": "1.0.0",\n'
                    '    "ICD": {\n        "library_path": "libEGL_nvidia.so.0"\n'
                    '    }\n}\n')


def log(msg: str) -> None:
    print(f'[{time.strftime("%H:%M:%S")}] {msg}', flush=True)


# --------------------------------------------------------------------------
# GPU / rendering
# --------------------------------------------------------------------------
def ensure_nvidia_egl() -> bool:
    """Create the glvnd vendor file for the NVIDIA EGL client if the
    container lacks it (it only ships 50_mesa.json).  Idempotent."""
    if Path(_MISSING_ICD).is_file():
        return True
    lib = Path('/usr/lib/x86_64-linux-gnu/libEGL_nvidia.so.0')
    if not lib.exists():
        return False
    try:
        Path(_MISSING_ICD).write_text(_NVIDIA_ICD_JSON)
        return True
    except OSError:
        return False


def render_env(kind: str) -> dict:
    """Environment for Gazebo's headless EGL rendering."""
    if kind == 'nvidia' and ensure_nvidia_egl():
        return {
            '__EGL_VENDOR_LIBRARY_FILENAMES': _MISSING_ICD,
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
            'DRI_PRIME': '1',
        }
    if kind == 'nvidia':
        log('WARNING: NVIDIA EGL unavailable — falling back to the Intel iGPU')
    # Intel iGPU via Mesa/iris (never software llvmpipe).
    # WARNING 2026-09-23: this path does NOT work on this laptop — the
    # container's Mesa/iris rejects the TigerLake-H device ('Driver does not
    # support the 0x25a0 PCI ID') and EGL falls back to a software screen.
    # Kept only for completeness / other machines.
    return {
        '__EGL_VENDOR_LIBRARY_FILENAMES':
            '/usr/share/glvnd/egl_vendor.d/50_mesa.json',
        'MESA_LOADER_DRIVER_OVERRIDE': 'iris',
        'DRI_PRIME': '0',
        'LIBGL_ALWAYS_SOFTWARE': '0',
    }


# --------------------------------------------------------------------------
# Run folders
# --------------------------------------------------------------------------
def newest_run_dir(root: Path) -> Path | None:
    if not root.is_dir():
        return None
    # The standard layout names folders batch_N_run_<ts>; the isolated parallel
    # layout uses run_<ts>.  Match BOTH — globbing only 'run_*' made
    # --standard-layout runs report "INCOMPLETE / None" with no metrics even
    # though their data was on disk (seen 2026-09-23 09:38).
    dirs = sorted(p for p in root.iterdir() if p.is_dir() and 'run_' in p.name)
    return dirs[-1] if dirs else None


def read_summary(run_dir: Path | None) -> dict:
    if run_dir is None:
        return {}
    f = run_dir / 'summary.csv'
    if not f.is_file():
        return {}
    out = {}
    for line in f.read_text(errors='replace').splitlines()[1:]:
        if ',' in line:
            k, v = line.split(',', 1)
            out[k.strip()] = v.strip()
    return out


def run_finished(run_dir: Path | None, run_time: int) -> bool:
    """Is this run COMPLETE?  Only used by the opt-in --resume path.

    Duration alone is NOT enough.  A boot-flaked run keeps its idle coverage
    clock running (observed: a 254 s "run" of a 300 s budget where
    total_path_length_m was 0.03 m and no robot ever cleared the readiness
    gate), so a flake would masquerade as a finished run and never be
    retried.  Require all three:
        1. it lasted >= 80 % of the requested sim time,
        2. the robots actually drove (>= ~0.5 m of path per rover),
        3. the goal driver produced goal_stats.csv.
    """
    if run_dir is None:
        return False
    s = read_summary(run_dir)
    if not s:
        return False
    try:
        if float(s.get('duration_sec', 0)) < 0.8 * run_time:
            return False
        robots = int(float(s.get('total_robots', 0)))
        path_m = float(s.get('total_path_length_m', 0))
    except ValueError:
        return False
    if robots <= 0 or path_m < 0.5 * robots:
        return False
    return (run_dir / 'goal_stats.csv').is_file()


# --------------------------------------------------------------------------
# Live RTF sampling (sim seconds per wall second)
# --------------------------------------------------------------------------
def swarm_processes_running() -> bool:
    """Any swarm_basics simulation already up?  (Mirrors the guard inside
    start_experiment.sh so an overlap fails fast with a clear message instead
    of a confusing 3-second abort.)"""
    for pat in ('ros2 launch swarm_basics', '/swarm_basics/lib/swarm_basics/'):
        try:
            if subprocess.run(['pgrep', '-f', pat],
                              capture_output=True).returncode == 0:
                return True
        except OSError:
            return False
    return False


def acquire_lock():
    """Single-instance lock: two run_parallel.py processes can never fight
    over the container (the failure mode that aborted a run in 3 s on
    2026-09-23).  Returns an open file handle, or None when busy."""
    LOG_ROOT.mkdir(parents=True, exist_ok=True)
    handle = (LOG_ROOT / '.run_parallel.lock').open('w')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        handle.close()
        return None
    handle.write(str(os.getpid()))
    handle.flush()
    return handle


def purge_stale_shm() -> int:
    """Delete stale FastDDS shared-memory segments and semaphores.

    A SIGKILLed/messy shutdown leaves /dev/shm/fastrtps_*, fastdds_* and
    sem.fastrtps_port* (plus _port*/ *_el locks) behind.  The NEXT boot then
    logs
        [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_portNNNNN:
        open_and_lock_file failed
    and its lifecycle service calls fail ("change_state service is not
    available!", "async_send_request failed") -> "Aborting bringup" -> gate
    FAILED, with robots that never move (observed 2026-09-23 09:04 on a
    2-rover run started right after a forced shutdown; the same signature
    shows up as random robots failing a 5-rover boot).

    Only call this while no swarm nodes are running.
    """
    removed = 0
    for pat in ('*fastrtps*', '*fastdds*', '_port*', '*_el'):
        for path in Path('/dev/shm').glob(pat):
            try:
                path.unlink()
                removed += 1
            except OSError:
                pass
    return removed


def wait_until_clean(timeout: float = 90.0) -> bool:
    """Stop a stale simulation and wait until the container is idle again.

    Safe by default: the single-instance lock means no OTHER run_parallel is
    active, so any swarm_basics process still alive is a leftover from a
    crashed/killed run — exactly what used to block the next launch.
    """
    if not swarm_processes_running():
        return True
    subprocess.run(['bash', str(PKG_DIR / 'scripts' / 'stop_sim.sh')],
                   capture_output=True)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not swarm_processes_running():
            time.sleep(2.0)      # let FastDDS/ign teardown settle
            return True
        time.sleep(1.0)
    return False


def slot_gate_passed(slot_idx: int) -> bool:
    """True once start_experiment.sh printed its READY line (gate passed).

    The RTF sampler stays OFF before that on purpose: EVERY `ros2 topic echo`
    spawns a fresh DDS participant, and during bring-up that competes with the
    very service discovery we need to complete.  Symptom it caused on
    2026-09-23: lifecycle managers stuck on "Waiting for service
    planner_server/get_state" for minutes while my sampler probed every 15 s.
    """
    try:
        txt = (LOG_ROOT / f'slot_{slot_idx}.log').read_text(errors='replace')
    except OSError:
        return False
    return ('READY —' in txt) or ('READY -' in txt)


def _res_part(spec: str, idx: int, fallback: str) -> str:
    """Pick the width (idx 0) or height (idx 1) out of a 'WxH' string."""
    parts = str(spec).lower().replace('*', 'x').split('x')
    if idx < len(parts) and parts[idx].strip().isdigit():
        return parts[idx].strip()
    return fallback


def read_cpu_temp() -> float | None:
    """SoC temperature in C, or None when no thermal zone is readable."""
    for zone in ('/sys/class/thermal/thermal_zone0/temp',
                 '/sys/class/thermal/thermal_zone1/temp'):
        try:
            with open(zone) as fh:
                return int(fh.read().strip()) / 1000.0
        except (OSError, ValueError):
            continue
    return None


def wait_for_cool(target: float, timeout: float = 900.0) -> None:
    """Block until the SoC is at/below `target` C before starting a run.

    MEASURED 2026-09-23: the 5-rover boot launched at 96 C failed with a
    `slam_toolbox` SIGSEGV (gate FAILED robot_0 robot_4) while the same boot
    from 76 C passed 5/5 in 71 s.  Bring-up is the heaviest phase, so thermal
    headroom before it is the cheapest reliability win that needs no root.

    Aborts with KeyboardInterrupt when the harness has been signalled:
    a SIGTERM during this wait must NOT be followed by a launch (observed
    2026-09-23 10:03 — the worker sat in the gate and would have booted the
    whole simulation ~15 min after the harness was asked to stop).
    """
    if target <= 0:
        return
    t0 = time.monotonic()
    temp = read_cpu_temp()
    if temp is None:
        return
    if temp <= target:
        log(f'thermal gate open ({temp:.0f} C <= {target:.0f} C)')
        return
    log(f'waiting for the box to cool: {temp:.0f} C -> {target:.0f} C '
        f'(a 96 C boot is what SIGSEGVed slam_toolbox at 09:55)')
    last_log = time.monotonic()
    plateau_ref = (temp, time.monotonic())
    while True:
        if _STOP.is_set():
            raise KeyboardInterrupt('stop requested while waiting to cool')
        time.sleep(1.0)
        temp = read_cpu_temp()
        if temp is None:
            return
        if temp <= target:
            log(f'thermal gate open ({temp:.0f} C) — starting')
            return
        if time.monotonic() - t0 > timeout:
            log(f'thermal gate: still {temp:.0f} C after '
                f'{timeout / 60:.0f} min — starting anyway')
            return
        # Plateau escape: on a laptop that idles warm, waiting for an
        # unattainable target would burn the whole timeout.  Once the curve
        # flattens (<=1 C change in 90 s) the headroom we wanted is gone, so
        # start as long as we are clearly below the 96 C crash zone.
        if (time.monotonic() - plateau_ref[1]) >= 90.0:
            if plateau_ref[0] - temp < 1.0 and temp <= target + 15.0:
                log(f'thermal gate: cooling plateaued at {temp:.0f} C '
                    f'(target {target:.0f} C, still well below 96 C) — '
                    f'starting')
                return
            plateau_ref = (temp, time.monotonic())
        if time.monotonic() - last_log >= 30.0:
            log(f'  cooling: {temp:.0f} C (target {target:.0f} C, '
                f'{(time.monotonic() - t0) / 60:.0f} min elapsed)')
            last_log = time.monotonic()


def sample_sim_time(domain: int, timeout: float = 8.0) -> float | None:
    """Read Gazebo's sim clock on a slot's ROS domain."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    try:
        proc = subprocess.run(
            ['ros2', 'topic', 'echo', '--once', '/clock'],
            env=env, capture_output=True, text=True, timeout=timeout)
    except (subprocess.TimeoutExpired, OSError):
        return None
    m = re.search(r'sec:\s*(\d+)', proc.stdout or '')
    return float(m.group(1)) if m else None


# --------------------------------------------------------------------------
# One slot
# --------------------------------------------------------------------------
# Slots whose child processes are still alive.  A signal handler MUST reap
# these: killing only the harness orphans `start_experiment.sh`, which happily
# carries on with its own recovery ladder and keeps relaunching Nav2 (observed
# 2026-09-23: an orphaned run kept booting stacks for minutes after the harness
# was killed, leaving 33 processes and 237 /dev/shm segments behind).
_ACTIVE_SLOTS: list = []
_ACTIVE_LOCK = threading.Lock()
# Set by the signal handler; checked before ANY launch so a stop request can
# never be followed by a fresh boot (see wait_for_cool).
_STOP = threading.Event()


def session_pids(sid: int) -> list[int]:
    """Every live pid whose session id is `sid`.

    `start_new_session=True` makes the child a SESSION LEADER, but that does
    NOT keep its descendants in one process group: Gazebo's ruby/ign children
    end up in their own groups (observed 2026-09-23: PGIDs 73039/73041/73043
    for one slot whose leader was 72897).  `os.killpg` therefore missed them
    and a killed harness left a whole simulation running.  The session id is
    the reliable handle — kill that instead.
    """
    out: list[int] = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        try:
            with open(f'/proc/{pid}/stat', 'rb') as fh:
                data = fh.read().decode('utf-8', 'replace')
        except OSError:
            continue
        # comm is parenthesised and may contain spaces/parens: parse past it.
        try:
            fields = data[data.rindex(')') + 2:].split()
            if fields[0] == 'Z':        # already dead, just not reaped yet
                continue
            if int(fields[3]) == sid:   # state ppid pgrp SESSION
                out.append(pid)
        except (ValueError, IndexError):
            continue
    return out


class Slot:
    def __init__(self, index: int, scenario: str, args):
        self.index = index
        self.scenario = scenario
        self.args = args
        self.domain = args.domain_base + index
        self.partition = f'leo_slot{index}_{os.getpid()}'
        self.run_root = Path(args.results_root).expanduser() / f'slot_{index}'
        self.log_path = LOG_ROOT / f'slot_{index}.log'
        # Create both dirs HERE: `with self.log_path.open('w')` is evaluated
        # before Popen's env= argument, so creating them inside env() is too
        # late (that raised FileNotFoundError: .../parallel_logs/slot_0.log).
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.run_root.mkdir(parents=True, exist_ok=True)
        self.proc: subprocess.Popen | None = None
        self.rtf: float | None = None
        self.wall = 0.0
        # Per-run sampling state (kept on the slot, not in a dict keyed by
        # index: two jobs can share a slot index sequentially, and a sticky
        # "gate already passed" flag would then skip the NEXT job's bring-up
        # guard — that false positive is what fired at t=0 on 2026-09-23).
        self.started = False      # True only once OUR run has been launched
        self.gate_seen = False
        self.sim_base: float | None = None
        self.budget_stopped = False
        self.sample_prev: tuple[float, float] | None = None

    # -- environment ------------------------------------------------------
    def env(self) -> dict:
        e = dict(os.environ)
        e.update({
            # --- isolation: no ROS/TF/DDS cross-talk between slots ---
            'ROS_DOMAIN_ID': str(self.domain),
            'IGN_PARTITION': self.partition,
            'GZ_PARTITION': self.partition,
            'ROS_LOG_DIR': str(LOG_ROOT / f'roslog_slot{self.index}'),
            # --- results isolation (start_experiment.sh does NOT set this) ---
            'ROS_LOCALHOST_ONLY': '0',
            # --- YOLO placement: GPU inference, few CPU threads ---
            'YOLO_DEVICE': self.args.yolo_device,
            'YOLO_TORCH_THREADS': str(self.args.yolo_threads),
            'YOLO_PERIOD': str(self.args.yolo_period),
            'YOLO_HALF': '1' if self.args.yolo_half else '0',
            # Slots run concurrently BY DESIGN (isolated by domain + partition),
            # so the launch's global leftover-process guard must stand down.
            # Solo runs keep it: a stale sim would corrupt their CSVs.
            'SWARM_ALLOW_PARALLEL': '1' if self.args.allow_parallel else '0',
            # depth_to_scan builds a LaserScan from the depth camera, but both
            # costmaps AND SLAM are remapped to /{ns}/lidar/scan_clean and
            # nothing consumes the depth-derived /{ns}/scan — so it is skipped
            # (one less Python node per rover).  --keep-depth-to-scan brings
            # it back if an external tool needs /{ns}/scan.
            'SWARM_NO_DEPTH_TO_SCAN': '0' if self.args.keep_depth_to_scan else '1',
            # Keep the human detector out of the Nav2/SLAM bring-up window.
            'SWARM_DETECTOR_DELAY_S': str(self.args.detector_delay),
            # Sensor rates: the dominant Gazebo cost with several rovers.
            'SWARM_LIDAR_HZ': str(self.args.lidar_hz),
            'SWARM_CAMERA_HZ': str(self.args.camera_hz),
            # Camera resolution: rendering + bridge transport + detector
            # pre-processing all scale with the pixel count.
            'SWARM_CAMERA_W': _res_part(self.args.camera_res, 0, '640'),
            'SWARM_CAMERA_H': _res_part(self.args.camera_res, 1, '480'),
            # Clock relay rate: Gazebo's physics-rate clock is forwarded by the
            # bridge verbatim (~383 Hz at 4 rovers) and floods every sim-time
            # node.  See swarm_basics/clock_throttle.py.
            'SWARM_CLOCK_HZ': str(self.args.clock_hz),
            # DDS transport: see --dds-transports.  This is exported to EVERY
            # process in the sim (launch, bridges, Gazebo plugins, Nav2).
            'FASTDDS_BUILTIN_TRANSPORTS': str(self.args.dds_transports),
            # Event-driven human detectors: the YOLO model + CUDA context are
            # only loaded after start_experiment.sh publishes /swarm_ready, i.e.
            # once the Nav2 gate has passed.  This replaces the fixed
            # SWARM_DETECTOR_DELAY_S gamble (see yolo_human_processor.py).
            'YOLO_WAIT_GATE': '1' if self.args.detector_wait_gate else '0',
            # Out-of-map guard (fixed_goals.py): arena must be mapped to this
            # fraction before tour goals are sent, and 0 disables the
            # proximity bypass so the gate is strict.
            'SWARM_SCAN_THRESHOLD': str(self.args.scan_threshold),
            'SWARM_SCAN_SAFE_DIST': str(self.args.scan_safe_dist),
        })
        if not self.args.standard_layout:
            # Isolated flat layout: <results_root>/slot_N/run_<ts>/.  Left
            # UNSET for --standard-layout so run_utils writes the normal
            # <MODE>/<scenario>/batch_N_run_<ts>/ tree that the rest of the
            # toolchain (plot_coverage_from_csv, purge, run_matrix) expects.
            e['SWARM_RUN_DIR'] = str(self.run_root)
        e.update(render_env(self.args.render))
        return e

    def resolve_run_dir(self) -> Path | None:
        """Where the run actually landed (the report reads it from here)."""
        if self.args.standard_layout:
            mode = {'ai': 'BT_LLM', 'reactive': 'BT',
                    'stock': 'Nav2', 'no': 'Nav2'}[self.args.bt]
            root = (Path('/root/ros2_ws/src/results/coverage_logs')
                    / mode / self.scenario)
            return newest_run_dir(root)
        return newest_run_dir(self.run_root)

    # -- launch -----------------------------------------------------------
    def command(self) -> list[str]:
        a = self.args
        flags = ['--scenario', self.scenario, '--bt', a.bt, '--goals', a.goals,
                 '--run-time', str(a.run_time)]
        flags.append('--gui' if a.gui else '--headless')
        if a.detector:
            flags += ['--detector', a.detector]
        inner = (f'{ROS_SETUP} && cd {PKG_DIR} && '
                 # FastDDS opens one file descriptor per shared-memory segment
                 # (FastDDS SHM ports land in /dev/shm as <300 files with 4-5
                 # rovers).  The container ships ulimit -n 1024, so participants
                 # hit EMFILE and their SHM port allocation fails with
                 # "open_and_lock_file failed" — after which they lose discovery
                 # and their topics never appear (that is the missing
                 # /robot_N/lidar/scan, odom and map).  Raise it for the sim; the
                 # hard limit is far higher, so no privileges are needed.
                 f'ulimit -n 65536 2>/dev/null || true; '
                 f'umask 000 && '   # keep results host-user deletable (see start_experiment.sh)
                 f'exec ./scripts/start_experiment.sh {" ".join(flags)}')
        return ['taskset', '-c', ','.join(str(c) for c in self.cpus),
                'bash', '-c', inner]

    @property
    def cpus(self) -> tuple[int, ...]:
        if self.args.mode == 'solo':
            return SOLO_CPUS
        return SLOT_CPUS[self.index % MAX_SLOTS]

    def reset_attempt(self) -> None:
        """Clear per-run sampling state before a (re)launch of this slot."""
        self.gate_seen = False
        self.sim_base = None
        self.budget_stopped = False
        self.sample_prev = None
        self.rtf = None
        self.started = False

    def start(self) -> None:
        self.reset_attempt()
        with self.log_path.open('w') as fh:
            self.proc = subprocess.Popen(
                self.command(), env=self.env(), stdout=fh,
                stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                start_new_session=True)   # own session => everything is reapable
            self.started = True           # set INSIDE: the log is now truncated,
                                          # so a gate read cannot see a stale
                                          # READY line from the previous run
        with _ACTIVE_LOCK:
            _ACTIVE_SLOTS.append(self)
        log(f'slot {self.index}: {self.scenario}  pid={self.proc.pid}  '
            f'cpus={",".join(map(str, self.cpus))}  domain={self.domain}')

    def _kill_session(self, sig: int) -> int:
        """Signal the whole session; returns how many pids were signalled."""
        if self.proc is None:
            return 0
        sid = self.proc.pid            # session leader pid == session id
        pids = session_pids(sid)
        for pid in pids:
            try:
                os.kill(pid, sig)
            except (ProcessLookupError, PermissionError, OSError):
                pass
        try:                           # belt-and-braces: the direct group too
            os.killpg(sid, sig)
        except (ProcessLookupError, PermissionError, OSError):
            pass
        return len(pids)

    def stop(self) -> None:
        with _ACTIVE_LOCK:
            if self in _ACTIVE_SLOTS:
                _ACTIVE_SLOTS.remove(self)
        if self.proc is None:
            return
        # Escalate only while anything in the session is still alive.  A stale
        # `self.proc.poll()` is NOT enough here: it only reports the direct
        # child, which can exit while Gazebo keeps running.
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            if not session_pids(self.proc.pid):
                return
            self._kill_session(sig)
            for _ in range(10):
                if not session_pids(self.proc.pid):
                    return
                time.sleep(0.5)

    def request_stop(self) -> None:
        """Graceful SIGINT to the slot's whole session.

        start_experiment.sh traps SIGINT, flushes every CSV/PNG and stops the
        stack — so this ends a run WITH its data, unlike a hard kill.
        """
        if self.proc is not None:
            self._kill_session(signal.SIGINT)

    def wait(self) -> int:
        t0 = time.monotonic()
        rc = self.proc.wait() if self.proc else -1
        self.wall = time.monotonic() - t0
        return rc


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--mode', choices=('solo', 'slots'), default='solo',
                   help='solo = one heavy run tuned for FPS (default); '
                        'slots = several isolated runs in parallel')
    p.add_argument('--scenario', action='append', default=[], metavar='NAME',
                   help='scenario name, repeatable (e.g. 5_rovers_2_humans). '
                        'In slots mode every entry becomes a job.')
    p.add_argument('--bt', default='ai',
                   choices=('stock', 'no', 'reactive', 'ai'),
                   help='behaviour-tree arm (stock = plain Nav2, ai = BT+LLM; '
                        '"no" is a legacy alias for stock)')
    p.add_argument('--goals', default='fixed',
                   choices=('none', 'random', 'fixed', 'walk'))
    p.add_argument('--detector', default=None, choices=('yolo', 'depth'),
                   help='human detector backend (default: script default)')
    p.add_argument('--run-time', type=int, default=300,
                   help='WALL-clock run length in seconds (0 = no auto-stop). '
                        'Note the script\'s timer is wall time, so at low RTF '
                        'this yields little simulated time — see --sim-budget.')
    p.add_argument('--sim-budget', type=float, default=0.0, metavar='SEC',
                   help='stop each run once its SIMULATION clock has advanced '
                        'SEC seconds (0 = off, default). Prefer this over '
                        '--run-time for comparable data: 300 s of wall time at '
                        'RTF 0.12 is only ~36 s of sim, in which no goal can '
                        'complete (observed 2026-09-23 09:38: 78 goals sent, '
                        '0 succeeded).')
    p.add_argument('--repeat', type=int, default=1,
                   help='repeat the whole job list N times (seeds/repeats)')
    p.add_argument('--slots', type=int, default=3,
                   help=f'parallel slots in slots mode (max {MAX_SLOTS})')
    p.add_argument('--gui', action='store_true',
                   help='show the Gazebo GUI (solo mode only; needs DISPLAY). '
                        'Headless is the DEFAULT and much lighter.')
    p.add_argument('--headless', action='store_true',
                   help='run headless (already the default; accepted for '
                        'symmetry with --gui)')
    p.add_argument('--render', default='nvidia',
                   choices=('intel', 'nvidia', 'auto'),
                   help="Gazebo's headless sensor renderer (default: nvidia). "
                        "MEASURED 2026-09-23: the container's Mesa CANNOT drive "
                        "this laptop's Intel iGPU ('Driver does not support the "
                        "0x25a0 PCI ID' + 'egl: failed to create dri2 screen') "
                        "-> only the NVIDIA EGL path gives hardware rendering.")
    p.add_argument('--yolo-device', default='cuda',
                   help="YOLO inference device: 'cuda' (GPU), 'cpu', or a "
                        "torch index like '0'")
    p.add_argument('--yolo-threads', type=int, default=2,
                   help='torch CPU threads per YOLO worker (default 2)')
    p.add_argument('--yolo-period', type=float, default=0.33,
                   help='YOLO inference period in seconds (0.33 = ~3 Hz)')
    p.add_argument('--yolo-half', action='store_true',
                   help='FP16 inference (GPU only)')
    p.add_argument('--detector-wait-gate', dest='detector_wait_gate',
                   action='store_true', default=True,
                   help='(DEFAULT ON) load each robot\'s YOLO model only AFTER '
                        'the Nav2 readiness gate passes: start_experiment.sh '
                        'publishes /swarm_ready and the detectors load then. '
                        'The detector therefore consumes no torch/CUDA/CPU '
                        'during bring-up or recovery — a fixed delay cannot '
                        'know whether the gate will take 60 s or 400 s, and it '
                        'kept landing inside recovery windows.')
    p.add_argument('--no-detector-wait-gate', dest='detector_wait_gate',
                   action='store_false',
                   help='disable the event-driven detector and honour '
                        '--detector-delay instead (the old behaviour).')
    p.add_argument('--detector-delay', type=float, default=150.0, metavar='SEC',
                   help='start each robot\'s human detector SEC seconds after '
                        'the robot (default 150): loading 5 YOLO models + CUDA '
                        'contexts + DDS participants must clear the whole Nav2 '
                        'stagger (last stack starts at 68 s, activates ~100 s), '
                        'otherwise the lifecycle transitions abort. 0 = start '
                        'immediately.')
    p.add_argument('--lidar-hz', type=float, default=10.0, metavar='HZ',
                   help='LiDAR update rate (default 10). Lowering it is the '
                        'single biggest Gazebo cost saving with many rovers: '
                        '360 ray-casts x robots x rate. 5 Hz is plenty at '
                        '0.5 m/s rover speed (~10 cm of travel per scan).')
    p.add_argument('--camera-hz', type=float, default=5.0, metavar='HZ',
                   help='RGB-D camera rate (default 5). YOLO infers at ~3 Hz '
                        '(--yolo-period), so 3 Hz loses nothing while cutting '
                        'RGB+depth rendering and bridge bandwidth ~40 %%.')
    p.add_argument('--camera-res', default='640x480', metavar='WxH',
                   help='RGB-D camera resolution (default 640x480). Rendering, '
                        'bridge transport AND detector pre-processing all scale '
                        'with the pixel count, so 320x240 cuts them 4x — this '
                        'is the biggest single lever on the /clock + TF '
                        'starvation that stalls Nav2 bring-up.  Keep it at or '
                        'above --yolo-imgsz for detection quality.')
    p.add_argument('--clock-hz', type=float, default=50.0, metavar='HZ',
                   help='rate of the /clock relay (default 50). Gazebo emits '
                        'one Clock per physics step and the bridge forwards '
                        'every one — MEASURED 383 Hz with 4 rovers, i.e. tens '
                        'of thousands of sim-time callbacks per second across '
                        'the fleet, which starves TF/SLAM/Nav2 bring-up.  '
                        '50 Hz is ample (control 20 Hz, BT tick 100 Hz).')
    p.add_argument('--scan-threshold', type=float, default=0.30,
                   metavar='FRAC',
                   help='fraction of the ARENA that must be mapped before '
                        'fixed tour goals are sent (default 0.30 = 30 %%).  '
                        'Counted inside the fixed arena window, not over the '
                        'growing SLAM canvas (that is why the old 0.6 gate was '
                        'never reached: the canvas grows as the rover '
                        'explores, keeping the ratio at 6-19 %%). See '
                        'fixed_goals.py / SWARM_SCAN_THRESHOLD.')
    p.add_argument('--scan-safe-dist', type=float, default=4.0, metavar='M',
                   help='fixed goals nearer than M metres bypass the '
                        'out-of-map guard (default 4.0). The roof LiDAR sees '
                        '10 m, so a goal within ~4-5 m is already inside '
                        'scanned space; the guard is there to stop goals '
                        'BEYOND the mapped area. Set 0.0 for the strict gate '
                        '(no fixed goal until the arena is mapped to '
                        '--scan-threshold). See fixed_goals.py.')
    p.add_argument('--dds-transports', default='UDPv4', metavar='LIST',
                   help='FASTDDS_BUILTIN_TRANSPORTS for the whole sim '
                        '(default UDPv4, i.e. shared memory DISABLED). With '
                        '4-5 rovers the SHM transport runs out of allocatable '
                        'ports mid-bring-up: "[RTPS_TRANSPORT_SHM Error] Failed '
                        'init_port fastrtps_portNNNNN: open_and_lock_file '
                        'failed" floods the log, the affected participants '
                        'lose discovery and their topics never appear (that is '
                        'why robot_N.lidar.scan / odom / map can simply be '
                        'missing). Use "SHM,UDPv4" to opt back in; localhost '
                        'UDP is fast enough for 320x240 sensors.')
    p.add_argument('--domain-base', type=int, default=31,
                   help='first ROS_DOMAIN_ID; slot i uses base+i')
    p.add_argument('--results-root', default=str(RESULTS_ROOT),
                   help='isolated results root (ignored with '
                        '--standard-layout)')
    p.add_argument('--standard-layout', action='store_true',
                   help='write into the NORMAL dataset tree '
                        'results/coverage_logs/<MODE>/<scenario>/batch_N_run_<ts>/ '
                        'instead of the isolated parallel folder. Solo mode '
                        'only (slots would share one tree).')
    p.add_argument('--min-rtf', type=float, default=0.5,
                   help='warn when the measured real-time factor drops below '
                        'this (default 0.5)')
    p.add_argument('--cool-to', type=float, default=60.0, metavar='C',
                   help='wait until the CPU is at/below C before launching '
                        '(default 60, 0 = off). MEASURED: a 5-rover boot from '
                        '96 C crashed slam_toolbox (SIGSEGV, gate FAILED) while '
                        'the same boot from 76 C passed 5/5.')
    p.add_argument('--tries', type=int, default=2,
                   help='how many times to (re)launch a run whose bring-up '
                        'gate fails (default 2). Each retry waits for the '
                        'thermal gate again.')
    p.add_argument('--rtf-interval', type=float, default=15.0,
                   help='seconds between RTF samples')
    p.add_argument('--resume', action='store_true',
                   help='OPT-IN: skip a scenario only when a COMPLETE run '
                        'already exists in its slot folder (long enough AND '
                        'the rovers actually drove AND goal_stats.csv exists). '
                        'Default: always start a fresh run.')
    p.add_argument('--no-clean', action='store_true',
                   help='do NOT auto-clear stale processes before starting '
                        '(default: clear them automatically)')
    p.add_argument('--keep-depth-to-scan', action='store_true',
                   help='keep the deprecated depth_to_scan node (its /{ns}/scan '
                        'topic is unused by SLAM and both costmaps)')
    p.add_argument('--dry-run', action='store_true',
                   help='print the plan without launching anything')
    args = p.parse_args(argv)
    # 'no' is kept as a legacy alias for the stock Nav2 tree.
    if args.bt == 'no':
        args.bt = 'stock'
    return args


def build_jobs(args) -> list[str]:
    scenarios = args.scenario or ['1_rovers_1_humans']
    jobs = []
    for _ in range(max(1, args.repeat)):
        jobs.extend(scenarios)
    return jobs


def main(argv=None) -> int:
    args = parse_args(argv)

    if not START_SCRIPT.is_file():
        print(f'ERROR: {START_SCRIPT} not found — run this inside the '
              f'container with the workspace mounted.', file=sys.stderr)
        return 2
    if args.gui and args.mode == 'slots':
        print('ERROR: --gui only makes sense in solo mode.', file=sys.stderr)
        return 2
    if args.slots > MAX_SLOTS:
        print(f'ERROR: --slots max is {MAX_SLOTS} on this CPU '
              f'({MAX_SLOTS} free physical cores).', file=sys.stderr)
        return 2
    if args.standard_layout and args.mode == 'slots' and args.slots > 1:
        print('ERROR: --standard-layout requires --slots 1 — parallel slots '
              'would write into the same dataset tree and collide.',
              file=sys.stderr)
        return 2

    jobs = build_jobs(args)
    n_slots = 1 if args.mode == 'solo' else min(args.slots, len(jobs))
    # Concurrent runs are only ever intentional in slots mode (they are isolated
    # by ROS_DOMAIN_ID + Ignition partition + results folder).
    args.allow_parallel = (args.mode == 'slots' and n_slots > 1)

    log(f'mode={args.mode}  bt={args.bt}  goals={args.goals}  '
        f'run_time={args.run_time}s  jobs={len(jobs)}  slots={n_slots}')
    log(f'render={args.render}  yolo={args.yolo_device} '
        f'(threads={args.yolo_threads}, period={args.yolo_period}s, '
        f'half={args.yolo_half})')
    log(f'detector delay={args.detector_delay}s '
        f'(starts after the robot, i.e. out of the bring-up burst)')
    log(f'sensors: lidar={args.lidar_hz} Hz  camera={args.camera_hz} Hz '
        f'(defaults 10/5)')
    if args.standard_layout:
        mode = {'ai': 'BT_LLM', 'reactive': 'BT', 'stock': 'Nav2',
                'no': 'Nav2'}[args.bt]
        log(f'results: STANDARD tree -> results/coverage_logs/{mode}/'
            f'<scenario>/batch_N_run_<ts>/')
    else:
        log(f'results_root={args.results_root}')

    # ---- never collide: one instance, and a clean container before we start ----
    lock = acquire_lock()
    if lock is None:
        log('ERROR: another run_parallel.py is already running (single-instance '
            'lock held). Wait for it, or check scripts/parallel_logs/*.log.')
        return 2

    if not args.allow_parallel and not args.no_clean and not args.dry_run:
        if swarm_processes_running():
            log('stale simulation detected — clearing it before start '
                '(leftovers are what aborted the 09:01 run)...')
            if not wait_until_clean():
                log('ERROR: could not clear leftover processes — stop them '
                    'manually (scripts/stop_sim.sh) and retry.')
                return 2
            log('container is clean.')
        # ALWAYS purge stale SHM: even with nothing running, segments from a
        # SIGKILLed shutdown break DDS discovery on the next boot.
        freed = purge_stale_shm()
        if freed:
            log(f'purged {freed} stale FastDDS SHM segment(s)/semaphore(s).')
    elif args.allow_parallel and not args.no_clean and not args.dry_run:
        if not swarm_processes_running():
            freed = purge_stale_shm()
            if freed:
                log(f'purged {freed} stale FastDDS SHM segment(s)/semaphore(s).')

    if args.dry_run:
        for i, job in enumerate(jobs):
            slot = Slot(i % n_slots, job, args)
            print(f'  slot {slot.index}: {job}  cpus={slot.cpus}  '
                  f'domain={slot.domain}\n    {" ".join(slot.command())}')
        return 0

    # ---------------- job queue + worker pool ----------------
    # Every invocation starts FRESH runs by default: no silent reuse of older
    # results.  --resume is opt-in and refuses to count a run as complete
    # unless the rovers actually drove (see run_finished).
    pending: list[tuple[int, str]] = []
    for i, job in enumerate(jobs):
        slot_idx = i % n_slots
        if args.resume:
            pre = newest_run_dir(Path(args.results_root) / f'slot_{slot_idx}')
            if run_finished(pre, args.run_time):
                log(f'skip (complete run exists): {job} -> {pre.name}')
                continue
        pending.append((slot_idx, job))

    if not pending:
        log('nothing to do.')
        return 0

    log(f'{len(pending)} job(s) to run (fresh runs)')
    results: list[tuple[str, Slot, int]] = []

    # Slots are built HERE (in the main thread) so the sampler can see whether
    # a job has actually been launched yet.
    queue = [(Slot(slot_idx, job, args), job) for slot_idx, job in pending]

    def run_one(item) -> tuple[str, Slot, int]:
        slot, job = item
        rc = -1
        for attempt in range(1, max(1, args.tries) + 1):
            try:
                wait_for_cool(args.cool_to)
            except KeyboardInterrupt:
                log(f'slot {slot.index}: stop requested — not launching')
                break
            if _STOP.is_set():
                log(f'slot {slot.index}: stop requested — not launching')
                break
            slot.reset_attempt()
            slot.start()
            rc = slot.wait()
            slot.stop()   # belt-and-braces: reap anything the script left
            # Only OUR own stop suppresses a retry (the module-level SIGINT
            # handler reaps sessions, so the teardown can return nonzero):
            # exit 3 is the launcher's "bring-up gate failed" code and SHOULD
            # be retried; a graceful trap exits 0.
            if (rc == 0 or slot.budget_stopped
                    or attempt >= max(1, args.tries)):
                break
            log(f'slot {slot.index}: exit {rc} — bring-up/gate failed '
                f'(attempt {attempt}/{args.tries}), relaunching after cooldown')
        return job, slot, rc

    interrupted = False

    def _on_signal(signum, frame):
        nonlocal interrupted
        interrupted = True
        _STOP.set()
        log(f'signal {signal.Signals(signum).name} — reaping child '
            f'sessions...')
        # Reap the children FIRST.  Killing only the harness would orphan
        # start_experiment.sh, which keeps running its recovery ladder (and
        # keeps the machine hot) long after this process is gone.
        for slot, _job in queue:
            try:
                slot.stop()
            except Exception:      # noqa: BLE001
                pass
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    # Live RTF tracking lives on the Slot itself: one sample per slot per
    # --rtf-interval.  The sampler only reads the sim clock, so it can never
    # become a rate limit itself.
    futures: dict = {}
    try:
        with ThreadPoolExecutor(max_workers=n_slots) as pool:
            futures = {pool.submit(run_one, item): item for item in queue}
            while futures:
                for f in [f for f in futures if f.done()]:
                    item = futures.pop(f)
                    try:
                        results.append(f.result())
                    except Exception as exc:      # noqa: BLE001
                        log(f'job {item[1]} failed: {exc}')
                if not futures:
                    break
                now = time.monotonic()
                for f in list(futures):
                    slot, _job = futures[f]
                    # Only sample once OUR run has been launched: the log file
                    # is truncated at that moment, so before it a gate read can
                    # still see the PREVIOUS run's READY line.  That false
                    # positive made the sampler hit DDS from t=0 (2026-09-23),
                    # exactly the contention the gate exists to avoid.
                    if not slot.started:
                        continue
                    # Stay silent until this slot's bring-up gate has passed.
                    if not slot.gate_seen:
                        slot.gate_seen = slot_gate_passed(slot.index)
                        if not slot.gate_seen:
                            continue
                        log(f'slot {slot.index}: gate passed — RTF sampling on '
                            f'(no DDS probes during bring-up)')
                    prev = slot.sample_prev
                    if prev and (now - prev[0]) < args.rtf_interval:
                        continue
                    sim_t = sample_sim_time(args.domain_base + slot.index)
                    if sim_t is None:
                        continue
                    # Sim-time budget: stop on SIMULATED seconds, so results
                    # are comparable whatever the machine's RTF is.
                    if args.sim_budget > 0.0 and not slot.budget_stopped:
                        if slot.sim_base is None:
                            slot.sim_base = sim_t
                        if (sim_t - slot.sim_base) >= args.sim_budget:
                            slot.budget_stopped = True
                            log(f'slot {slot.index}: sim budget reached '
                                f'({sim_t - slot.sim_base:.0f}s >= '
                                f'{args.sim_budget:.0f}s) — graceful stop')
                            slot.request_stop()
                    if prev:
                        dw, ds = now - prev[0], sim_t - prev[1]
                        if dw > 0.5 and ds >= 0.0:
                            slot.rtf = ds / dw
                            if slot.rtf < args.min_rtf:
                                log(f'slot {slot.index}: RTF={slot.rtf:.2f} '
                                    f'(below --min-rtf {args.min_rtf})')
                    slot.sample_prev = (now, sim_t)
                time.sleep(2.0)
    except KeyboardInterrupt:
        log('interrupted — stopping every job...')
        for slot, _job in queue:
            try:
                slot.stop()
            except Exception:      # noqa: BLE001
                pass
        # Salvage whatever the workers did finish, so the report below still
        # shows the runs that produced data.
        for f in list(futures):
            try:
                results.append(f.result(timeout=20))
            except Exception:      # noqa: BLE001
                pass

    # ---------------- report ----------------
    print()
    log('================ RESULTS ================')
    bad = 0
    for job, slot, rc in results:
        run_dir = slot.resolve_run_dir()
        s = read_summary(run_dir)
        sim = s.get('duration_sec', '?')
        rtf = slot.rtf
        ok = run_finished(run_dir, args.run_time)
        if not ok:
            bad += 1
        rtf_txt = f'RTF={rtf:.2f}  ' if rtf is not None else ''
        print(f'  {job:<24} slot {slot.index}  rc={rc:<4} '
              f'sim={sim:>6}s  wall={slot.wall:6.0f}s  {rtf_txt}'
              f'{"OK " if ok else "INCOMPLETE"}')
        if s:
            print(f'      coverage={s.get("coverage_pct", "?")}%  '
                  f'path={s.get("total_path_length_m", "?")}m  '
                  f'bumps={s.get("total_bumps", "?")}')
        print(f'      {run_dir}')
    if interrupted:
        print('  (interrupted before all jobs finished)')
    print(f'  logs: {LOG_ROOT}/slot_*.log')
    return 1 if bad else 0


if __name__ == '__main__':
    raise SystemExit(main())
