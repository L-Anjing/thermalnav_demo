# ThermalNav Demo

Online RGB-D exploration adapter for the existing Go2 SRU PointGoal node. No
SRU weights, architecture, controller, or existing project sources are changed.

```bash
cd /media/user/data1/carl/workspace/thermalnav_demo
bash run_demo.sh build
bash run_demo.sh test
bash run_demo.sh doctor
bash run_demo.sh heuristic  # foreground; run stop in another terminal
bash run_demo.sh vlm
bash run_demo.sh rviz
bash run_demo.sh stop
```

Isaac must already be running. The runner starts SRU only when absent, using
its original `--go2-sim` mode with the same parameters as the existing launch.
It never runs Nav2 or publishes Twist. `/goal_pose` PoseStamped (`odom`) is the
only motion interface; no Route/goal_active/relative_goal interfaces exist in
this deployed SRU implementation. Its arrival radius is 0.75m; manager checks
odometry plus fresh zero Twist. Cancellation uses a current-pose PointGoal.

Actual Go2 topics: `/robot0/front_cam/{rgb,depth,camera_info}`,
`/robot0/{odom,imu,cmd_vel}`, `/tf`, `/tf_static`. RGB is rgb8, depth is 32FC1
metres, 640x360, fx=fy≈733, cx=320, cy=180; optical frame
`robot0/front_cam`. Camera clipping is 0.1–30m; this mapper accepts 0.15–8m.
Go2 publishes sensor/TF simulation stamps but no `/clock`; node timers/watchdogs
use wall time and TF queries use exact image stamps. Depth/odom skew >0.25s
is rejected. Do not enable ROS use_sim_time without a clock source.

## Mapping and startup safety

The 0.10m grid grows from 30x30m. Occupancy, observed endpoints, inferred
exploration, sensor-confirmed floor, coverage, trajectory and startup proof
are separate. Occupancy remains unknown for inference-only cells. A floor
endpoint requires a continuous valid depth neighbourhood and height ±0.06m
relative to the hospital ground (z=0). Full-resolution obstacle endpoints in
0.12–1.8m take precedence over floor, including table legs. Static obstacle
history is conservative: it is not cleared by rays projected above the floor.
This version is **single-floor** and is not a stair-navigation solution.

[IntentNav Appendix A](https://arxiv.org/html/2606.08029v1#A1) supplies the
3-degree bearing / 2m nearest reliable endpoint completion rule. Completion
writes only `/thermalnav/inferred` and explored state, stops at known obstacles,
and does not affect collision clearance or scan coverage. Camera blind-zone
intersections are recomputed from live K and full stamped TF, including pitch;
no paper-specific blind distance is hardcoded.

The existing 0.35m circular footprint is retained. Every target and straight
SRU segment requires collision-free swept footprints. Rejection diagnostics
include `unknown_footprint`, `occupied`, `disconnected`, and `no_safe_path`.
No safe candidate means zero new goals, not successful completion. The runner
does not automatically issue a manual bootstrap move.

Optional **simulation-only** initialization certification:

```bash
# Stop the previous Isaac instance first; this command refuses duplicates.
bash run_demo.sh observe       # separate terminal, before simulator
bash run_demo.sh sim-certified # original simulation + bounded PhysX query
bash run_demo.sh validate      # stationary baseline, automatic run, then stop
```

`sim_entry.py` wraps the original simulation entry without modifying it. It
queries only the stationary initial 0.35m footprint: 9 floor support rays and
an obstacle-height overlap box. A fresh successful attestation is accepted
only at the matching pose. It never marks sensor-free cells, overrides an
obstacle, moves with the robot, or certifies any forward region. Without this
attestation, initialization stays uncertified. A safe initial footprint alone
cannot establish a safe path across an unobserved gap; inference is deliberately
insufficient. Status reports this limitation explicitly.

## Detection, coverage and policies

OpenCV HOG supplies a dependency-free person detector (limited recall).
Optionally set `THERMALNAV_DETECTOR_ONNX` to a local COCO YOLOv8/11 ONNX export
with (84,N) output. Detection uses synchronized RGB/depth, median depth,
stamped TF and multi-frame spatial deduplication. No GT target positions are
used. A functioning detector is not evidence that a person was found.

Coverage values: -1 unknown, 0 mapped/unscanned, 50 directly observed floor,
75 suspected target, 100 occupied/unobservable. Ground-centre projection must
be inside the camera image and in front of measured occluding depth. Robot
motion, startup certification and near-field inference never mark coverage.

VLM settings: `THERMALNAV_VLM_URL` (OpenAI-compatible base URL ending /v1),
`THERMALNAV_VLM_MODEL`, optional `THERMALNAV_VLM_KEY`. Sends BEV/RGB, candidate
IDs, history, coverage and detections asynchronously at decision events. Only
an existing ID is accepted and revalidated. Missing service prints **VLM
unavailable**, uses an explicit heuristic fallback, and keeps `vlm_calls=0`.
`/thermalnav/status` contains counts, reasons, sensor geometry and policy mode.

Runtime logs live only in `/tmp/thermalnav_demo_<uid>`. `manual-check` is a
simulation-only comparison test and is **not** an automatic-start prerequisite.

## Latest startup validation

18 unit tests and colcon build pass. On a fresh, stationary Hospital/Go2
instance, PhysX returned nine supported ground probes and no footprint overlap;
the bounded initial-pose certificate was accepted. The camera measured about
0.435m height, 47.10° horizontal / 27.52° vertical FOV, and a 3.55m centre
floor-blind distance including actual robot pitch. Completion accumulated
1.72m² during settling and promoted **zero** inferred cells to verified free.
One frontier component remained disconnected/unknown-footprint blocked;
**zero automatic waypoints were completed**. The robot remained stopped.
The earlier manually bootstrapped inspection run is not counted as an automatic
startup success. More actual near-field sensor evidence (or a separately
validated active-sensing capability) is needed to bridge this gap safely.
