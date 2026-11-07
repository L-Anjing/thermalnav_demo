#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
export PYTHONNOUSERSITE=1 ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/thermalnav_ros}"
source /opt/ros/humble/setup.bash
source "$ROOT/../sru-robot-deployment/install/setup.bash"
export PYTHONPATH="$ROOT:${PYTHONPATH:-}"
RUNTIME="/tmp/thermalnav_demo_${UID}"
mkdir -p "$RUNTIME"
alive() {
    [ -f "$RUNTIME/$1.pid" ] || return 1
    local pid expected
    pid="$(cat "$RUNTIME/$1.pid")"
    case "$1" in demo) expected='thermalnav.node';; sru) expected='rl_nav_controller.rl_nav_controller';; sim) expected='thermalnav.sim_entry';; *) return 1;; esac
    kill -0 "$pid" 2>/dev/null && [ -r "/proc/$pid/cmdline" ] && tr '\0' ' ' < "/proc/$pid/cmdline" | grep -q "$expected"
}
case "${1:-doctor}" in
  stop-certified-sim)
    if alive sim; then
      pid="$(cat "$RUNTIME/sim.pid")"
      if tr '\0' ' ' < "/proc/$pid/cmdline" | grep -q 'thermalnav.sim_entry'; then kill -INT "$pid"; fi
    fi
    ;;
  sim-certified)
    if ros2 node list --no-daemon 2>/dev/null | grep -q '/go2_driver_node'; then
      echo 'Existing Isaac Sim detected; refusing duplicate launch'; exit 1
    fi
    source "$ROOT/../go2_omniverse/IsaacSim-ros_workspaces/humble_ws/install/setup.bash"
    source "$ROOT/../go2_omniverse/go2_omniverse_ws/install/setup.bash"
    eval "$(/home/carl/miniforge3/condabin/conda shell.bash hook)"
    conda activate env_isaaclab
    export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-1}"
    export ISAAC_ASSETS_ROOT=/media/user/data1/isaac-sim-assets/merged/Assets/Isaac/4.5
    cd "$ROOT/../go2_omniverse"
    export PYTHONPATH="$PWD:$ROOT:${PYTHONPATH:-}"
    echo $$ > "$RUNTIME/sim.pid"
    exec python -u -m thermalnav.sim_entry --robot_type go2 --controller lab --policy_path checkpoints/moe_cts_lab/policy.pt --policy_config checkpoints/moe_cts_lab/config.yaml --headless --enable_cameras --custom_env hospital --cmd_source ros2 --device cuda
    ;;
  validate) exec /usr/bin/python3 -m thermalnav.validate ;;
  manual-check) exec /usr/bin/python3 -m thermalnav.manual_check ;;
  status) exec ros2 topic echo /thermalnav/status --once --full-length ;;
  start) exec ros2 service call /thermalnav/start std_srvs/srv/Trigger "{}" ;;
  doctor) shift; exec /usr/bin/python3 -m thermalnav.doctor "$@" ;;
  build)
    cd "$ROOT"
    colcon --log-base /tmp/thermalnav_colcon_log build --base-paths . --packages-select thermalnav_demo --build-base /tmp/thermalnav_build --install-base "$ROOT/install" --event-handlers console_direct+
    ;;
  heuristic|vlm|observe)
    if alive demo; then echo 'ThermalNav already running'; exit 1; fi
    # Fail fast instead of leaving SRU/Demo waiting silently when Isaac is off.
    topics="$(ros2 topic list --no-daemon 2>/dev/null)"
    if ! grep -qx '/robot0/odom' <<< "$topics" || ! grep -qx '/robot0/front_cam/depth' <<< "$topics"; then
      echo '[ERROR] Isaac Sim is not publishing Go2 odom/depth. Start the existing simulator, then retry.' >&2
      exit 2
    fi
    # Never launch simulator here, and reuse an existing SRU controller.
    nodes="$(ros2 node list --no-daemon 2>/dev/null)"
    if ! grep -Eq 'navigation_policy_node|rl_nav_controller' <<< "$nodes"; then
      /home/carl/venvs/sru_ros/bin/python3 -m rl_nav_controller.rl_nav_controller --go2-sim --ros-args -p use_sim_time:=false -p depth_topic:=/robot0/front_cam/depth -p odom_topic:=/robot0/odom -p cmd_vel_topic:=/robot0/cmd_vel -p require_joystick:=false -p publish_debug_depth:=false > "$RUNTIME/sru.log" 2>&1 &
      echo $! > "$RUNTIME/sru.pid"
      sleep 3
      if ! grep -q 'navigation model\|Learning model is ready\|ONNX provider' "$RUNTIME/sru.log"; then
        echo '[ERROR] SRU failed to initialize; inspect' "$RUNTIME/sru.log" >&2
        exit 3
      fi
    fi
    mode="$1"; extra=()
    if [ "$mode" = observe ]; then mode=heuristic; extra=(--observe); fi
    /usr/bin/python3 -m thermalnav.node --mode "$mode" "${extra[@]}" > "$RUNTIME/demo.log" 2>&1 &
    echo $! > "$RUNTIME/demo.pid"
    echo "ThermalNav PID=$! mode=$mode. Log: $RUNTIME/demo.log"
    echo 'Waiting for stamped RGB-D/odom/TF; startup status follows.'
    for _ in {1..20}; do
      if ! alive demo; then echo '[ERROR] ThermalNav exited:' >&2; tail -30 "$RUNTIME/demo.log" >&2; exit 4; fi
      if grep -q 'thermalnav_demo' < <(ros2 node list --no-daemon 2>/dev/null); then break; fi
      sleep 0.25
    done
    if ! grep -q 'thermalnav_demo' < <(ros2 node list --no-daemon 2>/dev/null); then
      echo '[ERROR] ThermalNav node did not join ROS graph.' >&2; tail -30 "$RUNTIME/demo.log" >&2; exit 5
    fi
    echo 'Node ready. Inspect live state with: bash run_demo.sh status'
    wait "$(cat "$RUNTIME/demo.pid")"
    ;;
  rviz) exec rviz2 -d "$ROOT/config/demo.rviz" ;;
  stop)
    if alive demo; then
      timeout 5 ros2 service call /thermalnav/stop std_srvs/srv/Trigger '{}' >/dev/null || true
      kill -INT "$(cat "$RUNTIME/demo.pid")"
    fi
    sleep 2
    if alive sru; then kill -INT "$(cat "$RUNTIME/sru.pid")"; fi
    echo 'Stopped only ThermalNav-owned processes; simulator untouched.'
    ;;
  test) cd "$ROOT"; exec /usr/bin/python3 -m unittest discover -s tests -v ;;
  *) echo 'Usage: bash run_demo.sh doctor|heuristic|vlm|rviz|stop|build|test|observe'; exit 2 ;;
esac
