#!/bin/bash
# Doosan M0609 MoveIt bringup - real hardware mode with RT scheduling.
#
# Usage: ./bringup_real.sh [ROBOT_IP] [ROBOT_PORT]
#        ./bringup_real.sh 192.168.1.100
#
# Why this exists
# ---------------
# The pick-stage hardware judder ("뚝딱거림") is driven by servo_time jitter
# in dsr_hardware2::write():
#
#     servo_time = real_loop_dt * 20;   // dsr_hw_interface2.cpp
#     Drfl.servoj_rt(pos, vel, acc, servo_time);
#
# real_loop_dt is the measured controller_manager loop period. On a stock
# (non-RT) kernel that period jitters by several ms because ros2_control_node
# gets preempted and the CPU changes frequency, so servo_time swings ~0.16-0.28s
# and the harmonic-drive servo profile wobbles cycle to cycle. The servo_time
# formula lives in the doosan-robot2 submodule and is off-limits, but we can
# stabilize its *input* (real_loop_dt) from here:
#
#   1. pin the CPU governor to performance      -> kills frequency-scaling jitter
#   2. promote ros2_control_node to SCHED_FIFO  -> stops it being preempted
#   3. pin it to dedicated cores                -> keeps the loop off busy cores
#
# This also reduces the dt-window drops in write() (dt outside [0.3,1.5]*period
# returns OK without sending a command), so fewer servoj_rt waypoints are lost.
#
# Steps 1-3 need sudo. Each is best-effort: if it fails (no sudo / no cpupower)
# the bringup still runs, just without the RT stabilization. For the strongest
# result install a PREEMPT_RT kernel and isolate RT_CPUS via the isolcpus= boot
# arg; see docs/.

set -e

ROS_DISTRO=${ROS_DISTRO:-humble}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

ROBOT_HOST=${1:-192.168.1.100}
ROBOT_PORT=${2:-12345}
MODEL=${MODEL:-m0609}

# RT knobs (override via env: RT_PRIORITY=90 RT_CPUS=3 ./bringup_real.sh ...)
RT_PRIORITY=${RT_PRIORITY:-80}
RT_CPUS=${RT_CPUS:-2,3}
SET_GOVERNOR=${SET_GOVERNOR:-1}

find_workspace_setup() {
    local dir="$SCRIPT_DIR"
    while [ "$dir" != "/" ]; do
        if [ -f "$dir/install/setup.bash" ]; then
            echo "$dir/install/setup.bash"
            return 0
        fi
        dir=$(dirname "$dir")
    done
    return 1
}

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

WORKSPACE_SETUP=$(find_workspace_setup || true)
if [ -n "$WORKSPACE_SETUP" ]; then
    # shellcheck source=/dev/null
    source "$WORKSPACE_SETUP"
else
    echo "[WARN] workspace install/setup.bash not found. Run colcon build first."
fi

# 1. CPU governor -> performance (removes frequency-scaling dt jitter).
if [ "$SET_GOVERNOR" = "1" ] && command -v cpupower >/dev/null 2>&1; then
    echo "[RT] setting CPU governor to performance"
    sudo cpupower frequency-set -g performance \
        || echo "[RT][WARN] governor set failed (need sudo / cpupower); continuing"
fi

# 3 (deferred). Promote the control loop to RT once ros2_control_node is up.
promote_rt() {
    local pid=""
    local i
    for i in $(seq 1 60); do
        pid=$(pgrep -f ros2_control_node | head -n1 || true)
        if [ -n "$pid" ]; then
            echo "[RT] ros2_control_node pid=$pid -> SCHED_FIFO:${RT_PRIORITY} cpus=${RT_CPUS}"
            sudo chrt -f -p "$RT_PRIORITY" "$pid" \
                || echo "[RT][WARN] chrt failed (need sudo / RT limits); continuing"
            sudo taskset -acp "$RT_CPUS" "$pid" \
                || echo "[RT][WARN] taskset failed; continuing"
            return 0
        fi
        sleep 1
    done
    echo "[RT][WARN] ros2_control_node not found after 60s; RT promotion skipped"
}

echo "[REAL] DSR ${MODEL} MoveIt bringup (mode=real host=${ROBOT_HOST} port=${ROBOT_PORT})"

# 2. Launch in the background so promote_rt can reach the spawned control node,
#    then forward Ctrl-C to the launch and block on it (foreground behaviour).
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
    model:="${MODEL}" \
    mode:=real \
    host:="${ROBOT_HOST}" \
    port:="${ROBOT_PORT}" &
LAUNCH_PID=$!

trap 'kill -INT "$LAUNCH_PID" 2>/dev/null' INT TERM

promote_rt &

wait "$LAUNCH_PID"
