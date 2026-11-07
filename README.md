# ThermalNav Demo

C++17 RGB-D BEV mapping, inspection coverage, frontier planning, person detection
and asynchronous candidate-only VLM selection for the Go2 SRU PointGoal node.
The simulator, SRU weights and low-level controller remain independent ROS2
processes. The only motion output is `/goal_pose` (`PoseStamped`, `odom`);
ThermalNav never publishes Twist.

```bash
cd ~/nav_ws/thermalnav_demo
bash run_demo.sh build
bash run_demo.sh test
bash run_demo.sh integration-test  # isolated synthetic ROS graph; no simulator/SRU
# On carl, start the existing Hospital/Go2 simulator, then:
bash run_demo.sh doctor
bash run_demo.sh observe          # map only, no autonomous goals
bash run_demo.sh rviz             # another terminal
bash run_demo.sh start            # enable exploration in observe mode
bash run_demo.sh stop
```

`heuristic` and `vlm` start exploration immediately. `observe` starts mapping
without autonomous goals. The runner reuses an existing SRU node; otherwise it
uses `SRU_PYTHON` (default `/home/carl/venvs/sru_ros/bin/python3`). `SRU_SETUP`
can override the sibling deployment workspace setup. Local build/test does not
require that sibling workspace. A previous Python demo must be stopped before
starting the C++ runtime. Build artifacts use `/tmp/thermalnav_cpp_build_colcon`,
logs `/tmp/thermalnav_colcon_log`, and the local ROS install is `install/`.

Dependencies: ROS2 Humble (`rclcpp`, messages, `tf2_ros`, `cv_bridge`,
`ament_cmake`, `ament_cmake_python`, `ament_cmake_gtest`), OpenCV development
libraries, `nlohmann-json3-dev`, `libcurl4-openssl-dev`. Python is retained only
for the Isaac SDK entry adapter, read-only diagnostics and live test tools.
There is no Python navigation/mapping runtime or cross-repository import in
the C++ executable.

## Simulator contract

Checked against `go2_omniverse` commit `be46713`, especially `core/omnigraph.py`,
`core/ros2.py`, and `configs/robot_extrinsic.yaml`:

* `/robot0/front_cam/{rgb,depth,camera_info}` share the camera optical frame
  `robot0/front_cam`; resolution is 640x360, fx/fy approximately 733. Intrinsics
  come from CameraInfo, not hardcoded mapper values. Pinhole depth is optical-Z
  in metres (`32FC1`); the mapper also converts `16UC1` millimetres.
* `odom -> robot0/base_link` is dynamic; `base_link -> front_cam` uses the same
  optical-to-base extrinsic as `CameraCfg` with `convention="ros"`. Translation
  is `[0.32487, -0.00095, 0.05362]` relative to the base. Do not add base height
  or rotate the optical axes a second time in the mapper.
* Images/TF/odometry use simulation stamps. Go2 does not publish `/clock`;
  keep `use_sim_time=false`. TF uses the exact depth timestamp, odometry is
  matched from a bounded history (maximum 0.25s skew), watchdogs use steady time.
* Sensor subscriptions accept both reliable and best-effort publishers.
  Maps/images/status use reliable, transient-local QoS for late RViz subscribers.

## Mapping and exploration

The 0.10m global grid grows from 30x30m. Occupancy (`/thermalnav/bev`), coverage
(`/thermalnav/coverage`), near-field inference (`/thermalnav/inferred`), targets
and robot trajectory are separate layers. ROS occupancy values are -1 unknown,
0 free, 100 occupied. Static obstacle history is conservative. The mapper is
single-floor (default ground z=0), not a stair-navigation solution.

Every valid depth pixel contributes obstacle endpoints in the height band
0.12–1.8m, including odd-pixel thin obstacles. Reliable ground endpoints within
±0.06m contribute free cells. `floor_stride` (default 2) only samples floor and
inference observations. Obstacle evidence takes precedence over floor evidence.
Above-ground camera rays do not certify their 2D shadows as free: the former
Python implementation incorrectly cleared the near-field blind zone this way.
No dilation, hole filling or inferred cells are used to fabricate traversability.

Coverage tests floor-cell centres against the current camera FOV and the minimum
valid depth in a five-pixel neighbourhood. Unknown, free and inspected are
different states. Coverage values are -1 unknown, 0 mapped/unscanned, 50 observed
floor, 75 suspected person, 100 occupied/unobservable. Near-field inference uses
supported endpoints within 2m in 3° bearing bins, stops at known obstacles, and
never promotes occupancy or coverage.

The 0.35m circular footprint and every outgoing/active SRU segment require
verified clearance. A simulator startup certificate only covers its original
bounded footprint and cannot bridge an unobserved gap. With the current
horizontal front camera, exploration can legitimately remain blocked by that
gap. C++ migration does not solve missing near-field sensor evidence, and zero
safe candidates is not reported as successful exploration.

The node drops duplicate/out-of-order depth, clears TF and resubscribes static
TF on simulation time rewind, and by default resets the map for the new episode.
`reset_map_on_time_jump:=false` preserves map history only when an unchanged
world/odom frame is guaranteed. Old detections and VLM replies cannot issue
goals after a reset or stop. Stale depth/odom/command feedback holds navigation.
Timers remain responsive while HOG/ONNX detection and VLM requests run in
separate workers. Ctrl-C/SIGTERM requests a current-pose goal before shutdown.

RViz displays an opaque raw occupancy grid in `odom` plus a robot-centred RGB
preview with explicit unknown/free/occupied colours. Coverage remains a separate
optional display. `/thermalnav/status` includes checksums, mapping milliseconds,
input stamps, TF failures, map/epoch counts, policy status and rejection reasons.
Checksums use FNV-1a and include the grid bytes, not an algorithm performance score.

## Detection and VLM

OpenCV HOG is the default person detector (limited recall). Set
`THERMALNAV_DETECTOR_ONNX` to a local COCO YOLOv8/11 ONNX model with output
`[1,84,N]` or `[1,N,84]`. Synchronized RGB-D, median box depth, stamped TF and
multi-frame spatial deduplication produce target tracks. RGB remains a thermal
proxy; this does not validate real infrared detection.

VLM environment: `THERMALNAV_VLM_URL` (OpenAI-compatible base ending `/v1`),
`THERMALNAV_VLM_MODEL`, optional `THERMALNAV_VLM_KEY`. Only an existing candidate
ID is accepted, then revalidated on the current map. Missing/unavailable service
reports an explicit heuristic fallback. `vlm_calls` counts actual HTTP attempts;
an absent service configuration keeps it at zero. HTTP timeouts are bounded.

Optional simulator tools remain available: `sim-certified`, `stop-certified-sim`,
`validate`, `manual-check`, `status`. The certificate only attests the initial
footprint, using nine floor probes and an overlap query. `manual-check` is an
explicit motion test, never an exploration prerequisite.

## Validation scope

C++ tests cover world-coordinate accumulation/rotation, grid growth, unknown and
occlusion handling, thin obstacles, floor evidence, coverage, fixed certificates,
swept-footprint planning, localization, tracking and rendering. The isolated ROS
test publishes best-effort Go2-shaped RGB-D/odom/TF, checks world-map history,
duplicate frames, millimetre conversion, stale inputs, stop/reset and retained
map delivery. These local checks do not replace Hospital/Isaac closed-loop
validation on carl, nor demonstrate improved navigation success rate.
