#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
export PYTHONNOUSERSITE=1 ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/thermalnav_ros}"
source /opt/ros/humble/setup.bash
SRU_SETUP="${SRU_SETUP:-$ROOT/../sru-robot-deployment/install/setup.bash}"
if [ -f "$SRU_SETUP" ]; then source "$SRU_SETUP"; fi
if [ "${1:-doctor}" != build ] && [ -f "$ROOT/install/setup.bash" ]; then source "$ROOT/install/setup.bash"; fi
DEMO_BIN="$ROOT/install/thermalnav_demo/lib/thermalnav_demo/thermalnav"
TOOLS_BIN="$ROOT/install/thermalnav_demo/lib/thermalnav_demo/thermalnav_tools"
BUILD_BASE="${THERMALNAV_BUILD_BASE:-/tmp/thermalnav_cpp_build_colcon_${UID}}"
LOG_BASE="${THERMALNAV_LOG_BASE:-/tmp/thermalnav_colcon_log_${UID}}"
TEST_BIN="$BUILD_BASE/thermalnav_demo/test_ros_runtime"
STEREO_BIN="$ROOT/install/thermalnav_demo/lib/thermalnav_demo/thermalnav_stereo"
EVALUATE_BIN="$ROOT/install/thermalnav_demo/lib/thermalnav_demo/thermalnav_evaluate"
MAPPING_SOURCE="${MAPPING_SOURCE:-fused}"
CAMERA_RIG="${CAMERA_RIG:-legacy}"
case "$CAMERA_RIG" in
  legacy)
    DEPTH_TOPIC=/robot0/front_cam/depth; INFO_TOPIC=/robot0/front_cam/camera_info
    LEFT_TOPIC=/robot0/front_cam/rgb; RIGHT_TOPIC=/robot0/front_cam_right/rgb
    LEFT_INFO=/robot0/front_cam/camera_info; RIGHT_INFO=/robot0/front_cam_right/camera_info
    STEREO_REFERENCE=/robot0/front_cam/depth ;;
  d455f_twin612)
    DEPTH_TOPIC=/robot0/d455f_depth/depth; INFO_TOPIC=/robot0/d455f_depth/camera_info
    LEFT_TOPIC=/robot0/twin612_left/rgb; RIGHT_TOPIC=/robot0/twin612_right/rgb
    LEFT_INFO=/robot0/twin612_left/camera_info; RIGHT_INFO=/robot0/twin612_right/camera_info
    STEREO_REFERENCE=/robot0/twin612_left/depth_reference ;;
  *) echo 'CAMERA_RIG must be legacy or d455f_twin612' >&2; exit 2 ;;
esac
STEREO_ARGS=(--ros-args -p left_topic:="$LEFT_TOPIC" -p right_topic:="$RIGHT_TOPIC" -p left_info_topic:="$LEFT_INFO" -p right_info_topic:="$RIGHT_INFO")

case "$MAPPING_SOURCE" in depth_only|stereo_only|fused) ;; *) echo 'MAPPING_SOURCE must be depth_only, stereo_only or fused' >&2; exit 2;; esac

RUNTIME="/tmp/thermalnav_demo_${UID}"
mkdir -p "$RUNTIME"
alive() {
    [ -f "$RUNTIME/$1.pid" ] || return 1
    local pid expected
    pid="$(cat "$RUNTIME/$1.pid")"
    case "$1" in demo) expected="/lib/thermalnav_demo/thermalnav([[:space:]]|$)";; stereo) expected='/lib/thermalnav_demo/thermalnav_stereo([[:space:]]|$)';; sru) expected='rl_nav_controller.rl_nav_controller';; *) return 1;; esac
    kill -0 "$pid" 2>/dev/null && [ -r "/proc/$pid/cmdline" ] && tr '\0' ' ' < "/proc/$pid/cmdline" | grep -Eq "$expected"
}
case "${1:-doctor}" in
  doctor|manual-check|validate)
    task="${1:-doctor}"; if [ "$#" -gt 0 ]; then shift; fi
    if [ "$task" = doctor ] && [ "${1:-}" = --startup ]; then task=startup-audit; shift; fi
    [ -x "$TOOLS_BIN" ] || { echo 'Build the C++ tools first: bash run_demo.sh build' >&2; exit 2; }
    exec "$TOOLS_BIN" --ros-args -p task:="$task" "$@"
    ;;
  evaluate) shift; exec "$EVALUATE_BIN" --ros-args -p depth_topic:="$DEPTH_TOPIC" -p info_topic:="$INFO_TOPIC" -p stereo_reference_topic:="$STEREO_REFERENCE" -p stereo_info_topic:="$LEFT_INFO" -p collision_height:="${COLLISION_HEIGHT:-0.65}" "$@" ;;
  stereo) exec "$STEREO_BIN" "${STEREO_ARGS[@]}" ;;
  status) exec ros2 topic echo /thermalnav/status --once --full-length ;;
  stereo-status)
    nodes="$(ros2 node list --no-daemon 2>/dev/null)"
    if ! grep -qx '/thermalnav_stereo' <<< "$nodes"; then
      echo 'Stereo frontend is not running; no stereo matches can be published.' >&2
      exit 2
    fi
    topics="$(ros2 topic list --no-daemon 2>/dev/null)"
    missing=0
    for topic in "$LEFT_TOPIC" "$RIGHT_TOPIC" "$LEFT_INFO" "$RIGHT_INFO"; do
      if ! grep -qx "$topic" <<< "$topics"; then
        echo "Missing stereo topic: $topic" >&2; missing=1; continue
      fi
      info="$(ros2 topic info --no-daemon "$topic" 2>/dev/null || true)"
      if ! grep -Eq '^Publisher count: [1-9]' <<< "$info"; then
        echo "No sensor publisher on stereo input: $topic" >&2; missing=1
      fi
    done
    if [ "$missing" -ne 0 ]; then
      echo 'Check CAMERA_RIG and launch the simulator with STEREO_RGB=1.' >&2
      exit 3
    fi
    if ! timeout 5 ros2 topic echo /thermalnav/stereo/status --once --full-length; then
      echo 'Stereo node exists but did not publish status within 5 s.' >&2
      exit 4
    fi
    ;;
  start) exec ros2 service call /thermalnav/start std_srvs/srv/Trigger "{}" ;;
  build)
    cd "$ROOT"
    # Isaac/Conda shells can leak incompatible CMAKE_PREFIX_PATH and runtime
    # libraries into CMake's cache/link step. Build in a fresh, system ROS env.
    env -i HOME="${HOME:-/home/carl}" USER="${USER:-carl}" PATH="/usr/bin:/bin" LANG="${LANG:-C.UTF-8}" \
      bash --noprofile --norc -c '
        set -e
        source /opt/ros/humble/setup.bash
        exec colcon --log-base "$1" build --base-paths . --packages-select thermalnav_demo \
          --build-base "$2" --install-base "$3" --event-handlers console_direct+ \
          --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE=/usr/bin/python3 \
          -Dnlohmann_json_DIR=/usr/lib/cmake/nlohmann_json
      ' thermalnav-build "$LOG_BASE" "$BUILD_BASE" "$ROOT/install"
    ;;
  heuristic|vlm|observe)
    [ -x "$DEMO_BIN" ] || { echo "Build the C++ runtime first: bash run_demo.sh build" >&2; exit 2; }
    if alive demo; then echo 'ThermalNav already running'; exit 1; fi
    # Fail fast instead of leaving SRU/Demo waiting silently when Isaac is off.
    topics="$(ros2 topic list --no-daemon 2>/dev/null)"
    if ! grep -qx '/robot0/odom' <<< "$topics" || ! grep -qx '/robot0/front_cam/depth' <<< "$topics"; then
      echo '[ERROR] Isaac Sim is not publishing Go2 odom/depth. Start the existing simulator, then retry.' >&2
      exit 2
    fi
    if [ "$MAPPING_SOURCE" != depth_only ]; then
      [ -x "$STEREO_BIN" ] || { echo 'Build the stereo frontend first' >&2; exit 2; }
      if ! grep -qx "$RIGHT_TOPIC" <<< "$topics"; then
        if [ "$MAPPING_SOURCE" = stereo_only ]; then echo '[ERROR] Right RGB camera missing; start simulator with STEREO_RGB=1' >&2; exit 2; fi
        echo '[WARN] Right RGB camera missing; fused mode will report depth-only fallback.' >&2
      fi
      if ! alive stereo && ! ros2 node list --no-daemon 2>/dev/null | grep -qx '/thermalnav_stereo'; then
        "$STEREO_BIN" "${STEREO_ARGS[@]}" > "$RUNTIME/stereo.log" 2>&1 &
        echo $! > "$RUNTIME/stereo.pid"
      fi
    fi
    if ! grep -qx "$DEPTH_TOPIC" <<< "$topics"; then echo "[ERROR] Mapping depth missing: $DEPTH_TOPIC (check CAMERA_RIG on both terminals)" >&2; exit 2; fi
    # Never launch simulator here, and reuse an existing SRU controller.
    nodes="$(ros2 node list --no-daemon 2>/dev/null)"
    if ! grep -Eq 'navigation_policy_node|rl_nav_controller' <<< "$nodes"; then
      "${SRU_PYTHON:-/home/carl/venvs/sru_ros/bin/python3}" -m rl_nav_controller.rl_nav_controller --go2-sim --ros-args -p use_sim_time:=false -p depth_topic:=/robot0/front_cam/depth -p odom_topic:=/robot0/odom -p cmd_vel_topic:=/robot0/cmd_vel -p require_joystick:=false -p publish_debug_depth:=false > "$RUNTIME/sru.log" 2>&1 &
      echo $! > "$RUNTIME/sru.pid"
      sleep 3
      if ! grep -q 'navigation model\|Learning model is ready\|ONNX provider' "$RUNTIME/sru.log"; then
        echo '[ERROR] SRU failed to initialize; inspect' "$RUNTIME/sru.log" >&2
        exit 3
      fi
    fi
    mode="$1"; extra=()
    if [ "$mode" = observe ]; then mode=heuristic; extra=(-p observe:=true); fi
    "$DEMO_BIN" --ros-args -p mode:="$mode" -p mapping_source:="$MAPPING_SOURCE" -p mapping_depth_topic:="$DEPTH_TOPIC" -p mapping_info_topic:="$INFO_TOPIC" -p stereo_info_topic:="$LEFT_INFO" -p mapping_depth_dropout:="${MAPPING_DEPTH_DROPOUT:-0.0}" -p mapping_depth_noise_sigma:="${MAPPING_DEPTH_NOISE_SIGMA:-0.0}" -p mapping_depth_hole_fraction:="${MAPPING_DEPTH_HOLE_FRACTION:-0.0}" -p mapping_depth_max_range:="${MAPPING_DEPTH_MAX_RANGE:-8.0}" -p collision_height:="${COLLISION_HEIGHT:-0.65}" "${extra[@]}" > "$RUNTIME/demo.log" 2>&1 &
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
  rviz) exec rviz2 -d "$ROOT/config/demo.rviz" --ros-args -r "/thermalnav/rgb_proxy:=$LEFT_TOPIC" ;;
  stop)
    if alive demo; then
      timeout 5 ros2 service call /thermalnav/stop std_srvs/srv/Trigger '{}' >/dev/null || true
      kill -INT "$(cat "$RUNTIME/demo.pid")"
    fi
    sleep 2
    if alive stereo; then kill -INT "$(cat "$RUNTIME/stereo.pid")"; fi
    if alive sru; then kill -INT "$(cat "$RUNTIME/sru.pid")"; fi
    echo 'Stopped only ThermalNav-owned processes; simulator untouched.'
    ;;
  test)
    cd "$ROOT"
    colcon --log-base "$LOG_BASE" test --build-base "$BUILD_BASE" --install-base "$ROOT/install" --packages-select thermalnav_demo --event-handlers console_direct+ --ctest-args -LE integration
    colcon test-result --test-result-base "$BUILD_BASE" --verbose
    ;;
  integration-test)
    [ -x "$TEST_BIN" ] && [ -x "$DEMO_BIN" ] || { echo "Build with BUILD_TESTING=ON first" >&2; exit 2; }
    exec "$TEST_BIN" --binary "$DEMO_BIN" --tools-binary "$TOOLS_BIN" --stereo-binary "$STEREO_BIN" --evaluation-binary "$EVALUATE_BIN"
    ;;
  *) echo 'Usage: bash run_demo.sh build|test|integration-test|doctor|observe|heuristic|vlm|rviz|status|stereo-status|start|stop|manual-check|validate|stereo|evaluate'; exit 2 ;;
esac
