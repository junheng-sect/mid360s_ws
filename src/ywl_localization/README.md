# YWL fixed-map localization

ROS1 Noetic sidecar for MID360S raw `livox_ros_driver2/CustomMsg` and `sensor_msgs/Imu`.
See [UPSTREAM.md](UPSTREAM.md) for source provenance. Baseline development starts at
`52c8665b76a3580fcb2f8aea7ea7c385dc7b55c8`.

## Build on Orange Pi

Use an independent catkin checkout and the board's existing Livox SDK2 installation:

```bash
source /opt/ros/noetic/setup.bash
catkin_make --cmake-only -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release \
  -DLIVOX_SDK_ROOT="$HOME/mid360s_ws/.deps/livox-sdk2" \
  '-DCATKIN_WHITELIST_PACKAGES=livox_ros_driver2;fast_lio;fastlio_gravity_bridge;drone_mapping;drone_relocalization;ywl_localization'
cmake --build build --target fast_lio_generate_messages_cpp livox_ros_driver2_generate_messages_cpp livox_ros_driver2_generate_messages_py -j2
cmake --build build --target fastlio_mapping gravity_bridge_node ywl_localization_node -j2
source devel/setup.bash
python3 src/ywl_localization/test/test_init_adapter.py
```

The explicit first build step creates the message headers the baseline FAST-LIO target includes. Tested
with Ubuntu 20.04 ARM64, GCC 9, ROS Noetic, PCL 1.10.0, Eigen 3.3.7 and OpenSSL 1.1.1f.

## Topics and handoff

Run `roslaunch ywl_localization ywl.launch map_dir:=/absolute/path/to/indoor_map` with the
existing baseline and startup relocalizer configured for that indoor map. Pass
`calibration:=/absolute/path/mid360s.yaml` to override the package calibration, and
`publish_cloud:=true` for map-frame registered points. Each ROS topic can be remapped.
YWL runs at the LiDAR scan frequency and has no TF publisher or connection to vehicle control.

| Topic | Message | Reference |
|---|---|---|
| `/ywl/bootstrap_odom` | `nav_msgs/Odometry` | LiDAR IMU pose in `ywl_odom`, before handoff |
| `/ywl/initial_transform` | `geometry_msgs/TransformStamped` | One confirmed `drone_map ← ywl_odom` transform from the adapter |
| `/ywl/odom` | `nav_msgs/Odometry` | LiDAR IMU pose and pose covariance in `drone_map` |
| `/ywl/velocity` | `geometry_msgs/Vector3Stamped` | IEKF velocity in `drone_map` at the odometry stamp |
| `/ywl/cloud_registered` | `sensor_msgs/PointCloud2` | Optional `drone_map` cloud |
| `/ywl/status` | `diagnostic_msgs/DiagnosticArray` | Workflow, geometry and timing diagnostics |

The adapter requires a confirmed result and status, fresh odometry in both streams, at least 0.9 s
of stationary history within the existing indoor limits, and a same-time pose pair. It allows
interpolation within a 150 ms bracket and never extrapolates. The baseline body pose already
represents the LiDAR IMU. Only the static gravity transform and the confirmed map alignment are
composed onto it.

At the next scan the estimator destroys the bootstrap tree and builds a clean tree from
`map_world.pcd`, transformed into `ywl_odom`. The IEKF origin, rotation, velocity, gravity and
covariance are preserved. The prior tree is queried without any additions or deletions. Treat
`/ywl/odom` twist as unspecified and take velocity from the separate topic.
`TRACKING` means the filter produced a finite update with valid point-to-plane observations.
`STALE` and `FAILED` stop valid output and require a fresh process with a new initialization.

## Isolated replay

The runner uses its own ROS master and publishes only the raw LiDAR and IMU topics from the bag.
Seed mode first regenerates baseline odometry and static TF from raw input, then runs the existing
two-window offline startup simulator with playback held during the solver. This applies the
existing map, static and window gates in sensor time without measuring live initialization latency.
A confirmed seed contains the baseline map alignment and the baseline IMU pose at the handoff
stamp, plus the map identity. Each YWL run computes a new alignment from its own bootstrap pose.

```bash
map="$HOME/test_data/mapping/20261021_110311_indoor"
bag="$HOME/test_data/mapping/indoor_fly_raw_000001.bag"
out="$PWD/validation/example"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
rosrun ywl_localization ywl_replay seed --map "$map" --bag "$bag" \
  --port 11578 --rate 0.5 --seconds 14 --out "$out/seed"
rosrun ywl_localization ywl_replay ywl --map "$map" --bag "$bag" \
  --port 11579 --rate 0.5 --seconds 13 --out "$out/smoke" --seed "$out/seed/seed.json"
rosrun ywl_localization ywl_replay baseline --map "$map" --bag "$bag" \
  --port 11580 --out "$out/baseline"
rosrun ywl_localization ywl_replay ywl --map "$map" --bag "$bag" \
  --port 11581 --out "$out/ywl" --seed "$out/seed/seed.json"
python3 src/ywl_localization/test/compare_replay.py --seed "$out/seed/seed.json" \
  --baseline "$out/baseline" --ywl "$out/ywl" --out "$out/comparison.json"
```

The replay comparison reports matched trajectory separation, end-to-end listener latency,
processing time, queue depth, output coverage, and process CPU/RSS. It uses the same confirmation
for both trajectories. Without independently surveyed reference points, separation and endpoint
displacement measure consistency.

Run the ROS fault and interface checks after the full YWL replay:

```bash
python3 src/ywl_localization/test/test_ros_faults.py --bag "$bag" --map "$map" \
  --ywl "$out/ywl" --seed "$out/seed/seed.json" --out "$out/faults" --port 11582
```
