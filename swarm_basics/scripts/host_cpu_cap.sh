#!/usr/bin/env bash
# =============================================================================
# host_cpu_cap.sh — thermal CPU cap for the SIMULATION HOST
#                   (run on the HOST, NOT inside the container)
#
# WHY THIS EXISTS
# ---------------
# A swarm boot runs 5 Gazebo + 5 SLAM + 5 Nav2 stacks (plus human detectors) on
# one laptop CPU.  Uncapped, this machine reaches ~96 C and throttles down to
# 2.2-2.8 GHz with large swings; under that load a robot's lifecycle transition
# or slam_toolbox init fails (observed 2026-09-23: robot_0_slam_toolbox SIGSEGV,
# exit -11 — while the same boot started at 76 C passed 5/5).  Capping every core
# at 2400 MHz keeps the clocks FLAT at 2400 MHz and the package at 76-88 C, which
# is what makes a 5-rover boot pass consistently.
#
# The cap is written to /sys, so it is LOST ON REBOOT.  This script reapplies it;
# --install-service makes it survive reboots through systemd.
#
# USAGE (needs root; run from the repository root on the host)
# -----------------------------------------------------------
#   sudo swarm_basics/scripts/host_cpu_cap.sh                 # cap at 2400 MHz
#   sudo swarm_basics/scripts/host_cpu_cap.sh --mhz 2800      # a different cap
#   sudo swarm_basics/scripts/host_cpu_cap.sh --status        # caps + governor
#   sudo swarm_basics/scripts/host_cpu_cap.sh --restore       # hardware maximum
#   sudo swarm_basics/scripts/host_cpu_cap.sh --install-service
#
# Every simulation run prints the active cap in its startup log, so a dataset
# collected on a throttled host is identifiable after the fact.
# =============================================================================
set -euo pipefail

MHZ=2400
ACTION=apply
SELF="$(readlink -f "$0")"
UNIT_PATH=/etc/systemd/system/leo-cpu-cap.service

usage() {
    cat <<'EOF'
host_cpu_cap.sh — thermal CPU frequency cap for the simulation host

  --mhz N            cap value in MHz (default 2400); clamped to the hardware max
  --status           show scaling_min/max/cur + governor per core (and temps)
  --restore          restore the hardware maximum (up to cpuinfo_max_freq)
  --install-service  write + enable a systemd unit so the cap survives reboots
  -h, --help         this text

Run it with sudo, on the host, not inside the container.
EOF
}

cpufreq_dirs() { compgen -G '/sys/devices/system/cpu/cpu*/cpufreq' || true; }

need_root() {
    if [[ "$(id -u)" != 0 ]]; then
        echo "ERROR: needs root (writes /sys and/or /etc)." >&2
        echo "       run: sudo $SELF $*" >&2
        echo "       (e.g. sudo $SELF --mhz $MHZ)" >&2
        exit 1
    fi
}

show_status() {
    local dir count=0
    for dir in $(cpufreq_dirs); do
        count=$((count + 1))
        printf '  %-7s min=%-8s max=%-8s cur=%-8s governor=%s\n' \
            "$(basename "$(dirname "$dir")")" \
            "$(cat "$dir/scaling_min_freq" 2>/dev/null || echo '?')" \
            "$(cat "$dir/scaling_max_freq" 2>/dev/null || echo '?')" \
            "$(cat "$dir/scaling_cur_freq" 2>/dev/null || echo '?')" \
            "$(cat "$dir/scaling_governor" 2>/dev/null || echo '?')"
    done
    if (( count == 0 )); then
        echo "  no cpufreq interface found — VM/container without frequency control?" >&2
        return 1
    fi
    echo "  cores: ${count}"
    if command -v sensors >/dev/null 2>&1; then
        sensors 2>/dev/null | grep -E 'Package id 0|^Core 0' | sed 's/^/  /' || true
    fi
    return 0
}

apply_cap() {
    local dir hw khz=$((MHZ * 1000)) applied=0
    for dir in $(cpufreq_dirs); do
        hw="$(cat "$dir/cpuinfo_max_freq" 2>/dev/null || echo 0)"
        if (( hw > 0 && khz > hw )); then
            echo "WARNING: ${khz} kHz is above the hardware maximum ${hw} kHz — clamping" >&2
            khz="$hw"
        fi
        echo "$khz" > "$dir/scaling_max_freq" 2>/dev/null && applied=$((applied + 1)) || true
    done
    echo "Requested a ${khz} kHz cap on ${applied} core(s)."
    echo "scaling_max_freq now: $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_max_freq 2>/dev/null | sort -u | tr '\n' ' ')"
    if (( applied == 0 )); then
        echo "ERROR: could not write any scaling_max_freq (unsupported hardware?)" >&2
        return 1
    fi
}

restore_cap() {
    local dir
    for dir in $(cpufreq_dirs); do
        echo "$(cat "$dir/cpuinfo_max_freq" 2>/dev/null || echo 0)" \
            > "$dir/scaling_max_freq" 2>/dev/null || true
    done
    echo "Restored the hardware maximum."
}

install_service() {
    need_root "$@"
    cat > "$UNIT_PATH" <<EOF
[Unit]
Description=Thermal CPU frequency cap for the Leo swarm simulation
Documentation=file:$SELF
After=multi-user.target

[Service]
Type=oneshot
RemainAfterExit=yes
# /sys forgets the cap at power-on, and an uncapped host throttles to ~96 C
# mid-boot, which loses robots inside the Nav2 readiness gate.
ExecStart=/bin/bash $SELF --mhz $MHZ

[Install]
WantedBy=multi-user.target
EOF
    systemctl daemon-reload
    systemctl enable --now leo-cpu-cap.service
    echo "Installed and enabled $UNIT_PATH (${MHZ} MHz applied at every boot)."
    echo "If the repository ever moves, re-run: sudo $SELF --install-service"
    systemctl --no-pager --lines=0 status leo-cpu-cap.service || true
}

# Parse arguments only AFTER the functions exist: bash executes top-to-bottom,
# so a loop placed above the definitions would call an undefined 'usage'.
while [[ $# -gt 0 ]]; do
    case "$1" in
        --mhz)             MHZ="${2:-}"; shift 2 ;;
        --status)          ACTION=status;  shift ;;
        --restore)         ACTION=restore; shift ;;
        --install-service) ACTION=install; shift ;;
        -h|--help)         usage; exit 0 ;;
        *) echo "Unknown argument: $1 (try --help)" >&2; exit 2 ;;
    esac
done

case "$ACTION" in
    status)  show_status ;;
    restore) need_root "$@"; restore_cap; show_status || true ;;
    install) install_service "$@" ;;
    apply)   need_root "$@"; apply_cap; echo; show_status || true ;;
esac
