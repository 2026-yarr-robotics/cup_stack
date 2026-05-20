#!/bin/bash
# Alternate cup_pyramid and cup_unstack launches in a loop.
# Bringup (bringup_sim.sh or bringup_real.sh) must already be running.
#
# Usage:
#   ./run_pyramid_unstack_loop.sh                 # infinite loop, nest_inc=0.012
#   ./run_pyramid_unstack_loop.sh 5               # 5 cycles
#   ./run_pyramid_unstack_loop.sh 5 0.0127        # 5 cycles, nest_inc=0.0127
#   ./run_pyramid_unstack_loop.sh inf 0.0127      # infinite with custom nest_inc

set -e

ROS_DISTRO=${ROS_DISTRO:-humble}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

CYCLES=${1:-inf}
NEST_INC=${2:-0.012}

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
    echo "[ERROR] workspace install/setup.bash not found. Run build_cup_stack.sh first." >&2
    exit 1
fi

run_launch() {
    local launch_file="$1"
    echo "[LOOP] ros2 launch cup_stack ${launch_file} nest_inc:=${NEST_INC}"
    ros2 launch cup_stack "${launch_file}" "nest_inc:=${NEST_INC}"
}

i=1
while :; do
    echo "[LOOP] === cycle ${i} ==="
    run_launch cup_pyramid.launch.py
    run_launch cup_unstack.launch.py

    if [ "$CYCLES" != "inf" ] && [ "$i" -ge "$CYCLES" ]; then
        echo "[LOOP] reached ${CYCLES} cycle(s), exiting."
        break
    fi
    i=$((i + 1))
done
