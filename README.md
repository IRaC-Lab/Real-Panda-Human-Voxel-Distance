# Panda–Human Voxel Distance (Real Robot)

ROS 2 workspace for PeopleSemSegNet-based human voxels, TF-transformed Panda
collision-mesh voxels, and filtered minimum-distance estimation against a
physical Panda + RealSense rig. Companion to the Isaac Sim repo,
[IsaacSim-Panda-Human-Voxel-Distance](https://github.com/IRaC-Lab/IsaacSim-Panda-Human-Voxel-Distance)
— `my_people_nvblox_bringup` here is a copy of that repo's package, not a
shared dependency; the two repos are independent.

## Environment

**Hardware**

- CPU: AMD Ryzen 9 5900X
- GPU: NVIDIA RTX 3070 (8 GB VRAM)
- RAM: 32 GB
- Panda arm, System 4.2.2, with a Franka Hand gripper
- Intel RealSense D435i, fixed or on an ArUco-tracked mount
- Ethernet link to the Panda control box (libfranka's real-time FCI
  connection)

**Software**

- Ubuntu 22.04.5, ROS 2 Humble
- Isaac ROS (NITROS), `release-3.0` apt channel: `isaac-ros-nvblox` /
  `isaac-ros-unet` / `isaac-ros-tensor-rt` / `isaac-ros-triton` /
  `realsense2-camera`
- libfranka 0.9.2 +
  [LCAS/franka_arm_ros2](https://github.com/LCAS/franka_arm_ros2) — Panda
  (System 4.x) needs libfranka 0.9.x; the official
  [franka_ros2](https://github.com/frankaemika/franka_ros2) driver only
  targets FR3+/newer libfranka, so this uses the community fork instead.

Eventual target is an NVIDIA Jetson AGX Orin — match JetPack/L4T and the
Isaac ROS release actually installed on the board. Other combinations
untested.

## Installation

### 1. Prerequisites

- ROS 2 Humble ([install guide](https://docs.ros.org/en/humble/Installation.html))
  plus `python3-colcon-common-extensions`, `python3-rosdep`,
  `python3-vcstool` (`rosdep init && rosdep update` on a fresh install).
- [git-lfs](https://git-lfs.com/): `git lfs install` once per machine, then
  `git lfs pull` after cloning (the vanilla PeopleSemSegNet `.onnx` is
  LFS-tracked).

### 2. Isaac ROS apt packages (NITROS)

```bash
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg

echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-3 jammy release-3.0" \
  | sudo tee /etc/apt/sources.list.d/nvidia-isaac-ros.list
sudo apt update
sudo apt install \
  ros-humble-isaac-ros-nvblox \
  ros-humble-isaac-ros-unet \
  ros-humble-isaac-ros-tensor-rt \
  ros-humble-isaac-ros-triton \
  ros-humble-realsense2-camera
```

`ros-humble-realsense2-camera` doesn't pull in the camera's udev rules, so
without this the RealSense fails to open with a libusb/permission error:

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
  | gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo jammy main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update
sudo apt install librealsense2-udev-rules librealsense2-utils
```

The native Isaac ROS tensor preprocessing also needs a compatible CV-CUDA
runtime. Match its version to the installed Isaac ROS release using
[isaac_ros_common's Dockerfile.base](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/release-3.2/docker/Dockerfile.base),
and choose the package for your CUDA version and CPU architecture. The
recorded Isaac ROS 3.2.10 setup used CV-CUDA v0.5.0-beta; v0.18.0 caused an
ABI symbol error. Installing the latest CV-CUDA release indiscriminately
is not sufficient. See [installation notes](docs/INSTALLATION_NOTES.md).

### 3. libfranka 0.9.2

```bash
sudo apt remove ros-humble-libfranka   # wrong version for Panda, if present
sudo apt install ros-humble-ros2-controllers ros-humble-joint-trajectory-controller
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev \
  libgflags-dev libgoogle-glog-dev libgtest-dev libbenchmark-dev

git clone --recursive https://github.com/frankarobotics/libfranka.git ~/libfranka
cd ~/libfranka
git checkout 0.9.2
git submodule update --init --recursive
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j"$(nproc)"
```

### 4. `~/.bashrc` environment

```bash
source_real_ws() {
    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
    export LD_LIBRARY_PATH="$HOME/libfranka/build:${LD_LIBRARY_PATH:-}"
    source /opt/ros/humble/setup.bash
    source "$HOME/panda_real_ws/install/setup.bash"
}
```

A function, not a permanent overlay — every command under `## Run` starts
with `source_real_ws` to switch that one terminal onto this workspace.

### 5. Workspace setup

```bash
cd "$HOME/panda_real_ws"
vcs import src < isaac_ros_nvblox.repos
git -C src/isaac_ros_nvblox submodule update --init --recursive
git -C src/isaac_ros_nvblox apply "$PWD/patches/nvblox_skip_empty_deletion.patch"
git -C src/isaac_ros_nvblox apply "$PWD/patches/realsense_camera_topic_namespace.patch"
git -C src/isaac_ros_nvblox apply "$PWD/patches/vanilla_segmentation_preprocessing.patch"
git -C src/isaac_ros_nvblox apply "$PWD/patches/nvblox_runtime_sync_timestamps.patch"
vcs import src < franka_arm_ros2.repos
git -C src/franka_arm_ros2 apply "$PWD/patches/franka_runtime_recovery.patch"

# Keep the optional splitter package available; this people pipeline reads
# depth directly and does not depend on splitter metadata.
rm -f src/isaac_ros_nvblox/nvblox_examples/realsense_splitter/COLCON_IGNORE

rosdep install --from-paths src --ignore-src -y \
  --skip-keys "isaac_ros_test isaac_ros_test_cmake ament_python"

colcon build --symlink-install \
  --packages-up-to panda_pick_place panda_real_bringup realsense_splitter \
  --cmake-args -DFranka_DIR="$HOME/libfranka/build" -DBUILD_TESTING=OFF
source install/setup.bash
```

The dependency patches include the color/mask synchronization, overlay
capture timestamps, Franka broadcaster rate limit, and recovery diagnostics.
Apply them to the pinned source revisions on a fresh checkout. On an
existing patched workspace, use `git apply --reverse --check <patch>` to
check whether a patch is already applied before applying it again.

On a memory-constrained board (e.g. Jetson), add `--parallel-workers 1` to
`colcon build` if it gets OOM-killed compiling nvblox_ros's CUDA code.

### 6. Regenerate the TensorRT engine

`models/peoplesemsegnet/vanilla/` holds the source `.onnx` weights; the
`.plan` engine under `1/` is GPU/TensorRT-version specific and not tracked
in git. Build with `trtexec`:

```bash
cd "$HOME/panda_real_ws/models/peoplesemsegnet/vanilla"
mkdir -p 1
/usr/src/tensorrt/bin/trtexec \
  --onnx=peoplesemsegnet_vanilla_unet_dynamic_etlt_fp32.onnx \
  --saveEngine=1/model_vanilla_v2_0_2.plan
```

Keep the validated original engine when optimizing transport and mapping.
Do not overwrite it with an unvalidated precision change: a direct FP16
rebuild produced invalid masks in the local comparison, and the supplied
INT8 calibration reduced person coverage in some frames. Validate reduced
precision against the original masks before deployment.

The separate 424x240 engine can be regenerated with:

```bash
python3 "$HOME/panda_real_ws/src/panda_pick_place/panda_pick_place/scripts/build_native_segmentation_engine.py" \
  --onnx "$HOME/panda_real_ws/models/peoplesemsegnet/vanilla/peoplesemsegnet_vanilla_unet_dynamic_etlt_fp32.onnx" \
  --output "$HOME/panda_real_ws/models/peoplesemsegnet/vanilla/1/model_vanilla_v2_0_2_424x240.plan"
```

## Run

Every terminal starts with `source_real_ws`. Two independent setups below —
`panda_realsense_people.launch.py` picks the right nvblox `global_frame`
(`panda_link0` vs `camera0_link`) automatically from `run_panda`, so nothing
else changes between them.

### With the Panda arm

Activate FCI on the Desk web UI before Terminal 1.

#### Terminal 1 — Panda driver + TF + pick-and-place controller

```bash
ros2 launch panda_pick_place panda_control.launch.py robot_ip:=172.16.0.2
```

#### Terminal 2 — RealSense

```bash
ros2 launch nvblox_examples_bringup realsense.launch.py \
  run_standalone:=True \
  color_profile:=848x480x30 \
  depth_profile:=848x480x30
```

#### Terminal 3 — ArUco camera alignment

Needs a printed ArUco marker, dictionary `DICT_6X6_250`, id `30`, 5 cm per
side, fixed to `panda_link0` (the robot base) and visible to the camera.
`mode:=dynamic` while the mount isn't fixed yet; `mode:=static
num_samples:=15` once it's permanently mounted.

```bash
ros2 launch panda_camera_alignment aruco_align.launch.py \
  mode:=dynamic \
  camera_mount_frame:=camera0_link
```

#### Terminal 4 — People segmentation + nvblox + Panda/human distance

```bash
MODEL_DIR="$HOME/panda_real_ws/models/peoplesemsegnet/vanilla"

ros2 launch panda_real_bringup panda_realsense_people.launch.py \
  run_realsense:=False \
  run_alignment:=False \
  run_rviz:=False \
  people_segmentation:=peoplesemsegnet_vanilla \
  vanilla_engine_file_path:="$MODEL_DIR/1/model_vanilla_v2_0_2.plan" \
  segmentation_output_binding_names:='["argmax_1"]'
```

#### Terminal 5 — RViz

```bash
rviz2 -d ~/panda_real_ws/src/panda_real_bringup/config/panda_realsense_people.rviz
```

Lighter arm-only debug view (Panda RobotModel + collision spheres, no camera
needed):

```bash
rviz2 -d ~/panda_real_ws/src/my_people_nvblox_bringup/config/visualization/panda_sphere_debug.rviz
```

#### Terminal 6 (optional) — Pick-and-place motion

`pick_place.yaml`'s waypoints are joint angles calibrated (via
`compute_waypoints.py`) for this specific desk/mount arrangement — on a
different setup, recompute them before running, or the arm may reach for a
pose that collides with something or isn't reachable at all. Ctrl+C once
returns home then stops; twice halts in place.

```bash
ros2 run panda_pick_place pick_place_node --ros-args \
  --params-file ~/panda_real_ws/src/panda_pick_place/config/pick_place.yaml
```

### Camera-only, no Panda

Debugs the people-detection pipeline (RealSense → segmentation → nvblox) on
its own, without the arm or an FCI connection. `run_panda:=False` switches
`global_frame` to `camera0_link` and skips the Panda-only nodes.

#### Terminal 1 — RealSense

```bash
ros2 launch nvblox_examples_bringup realsense.launch.py \
  run_standalone:=True \
  color_profile:=424x240x30 \
  depth_profile:=424x240x30
```

#### Terminal 2 — People segmentation + nvblox

```bash
ros2 launch panda_real_bringup panda_realsense_people.launch.py \
  run_realsense:=False \
  run_alignment:=False \
  run_rviz:=False \
  run_panda:=False
```

#### Terminal 3 — Color + depth overlay preview

```bash
ros2 run rqt_image_view rqt_image_view /nvblox_node/dynamic_color_frame_overlay &
ros2 run rqt_image_view rqt_image_view /nvblox_node/dynamic_depth_frame_overlay
```

### Avoid starting the same camera twice

The camera launch now reserves the detected D435i USB serial with a local
process lock and checks existing video handles before loading the driver.
A second launch, including one in another ROS domain, exits with an
`already reserved` or `already open by PID` message. Stop the previous camera
launch before retrying. ROS domains do not provide separate access to the
physical USB device. Default-domain operation uses `unset ROS_DOMAIN_ID` in
all terminals. A separate ROS domain does not create a separate camera device.

### Perception performance

`panda_realsense_people.launch.py` enables `optimize_perception:=True` by
default. The camera still publishes its configured raw RGB/depth rate. A
paired RGB/CameraInfo input node keeps at most three segmentation frames
outstanding and forwards the latest available frame when a result arrives,
avoiding a backlog of stale model inputs. Its internal input and result
feedback use reliable QoS; camera acquisition retains sensor-data QoS.

The validated original profile uses 960x544. A separate measured 424x240
profile is available with `segmentation_resolution:=424x240` when the raw
camera is 424x240 (or smaller). It selects
`model_vanilla_v2_0_2_424x240.plan`, whose external input/output are 424x240.
The U-Net internally pads to 448x256 for its stride-32 feature maps and crops
the mask back to 424x240; the camera image is not enlarged. The default
resolution remains 960x544 so existing larger-camera and robot launches keep
their profile. The low-resolution profile changes mask predictions and needs
accuracy evaluation for its intended scene.
Nvblox maps RGB at the native camera resolution. `native_mask_node` now applies
a four-connected component filter at full model resolution by default
(`mask_component_subsampling:=1`), preserving thin attached regions and gaps.
Use `mask_component_subsampling:=2` to compare the previous subsampled filter.
The area threshold is 2000 pixels at 960x544 and 390 at 424x240.
It conservatively reduces the cleaned
mask to camera resolution, and pairs it with that capture's original
CameraInfo. A foreground-covered target pixel remains foreground. Nvblox's
duplicate component filtering is disabled in this mode. The color overlay
now has the raw RGB dimensions (424x240 in the measured setup).

Native mapping requires camera dimensions no larger than the selected model
mask. The original configuration is available with
`segmentation_resolution:=960x544 optimize_perception:=False`.
`max_segmentation_in_flight:=3` is the measured default; two reduces the amount
of outstanding work further, with a potential throughput tradeoff.

To profile the raw images, intermediate images/mask, and final overlays:

```bash
source_real_ws
ros2 run panda_pick_place perception_profile --duration 20 --output /tmp/perception_rates.json
```

The profiler reports observed receive FPS and median/p95 age of the source
header stamp. Header age includes all upstream work and driver timing; it is
not the compute time of that stage. Diagnostic subscribers also add data
conversion work, so compare with the same subscriber set.

The 2026-10-14 comparison used 424x240x30 RGB/depth, an AGX Orin in 50W mode,
and a separate temporary `ROS_DOMAIN_ID=83` to exclude serialization warnings
also seen in the original configuration on domain 0. For isolated camera-only
measurements, set `export ROS_DOMAIN_ID=83` after `source_real_ws` in **every**
camera, perception, and viewer terminal. Robot ROS nodes must use the same
chosen domain when running the full robot setup.

Detailed measurements and validation are in
[src/panda_pick_place/panda_pick_place/PERCEPTION_PERFORMANCE.md](src/panda_pick_place/panda_pick_place/PERCEPTION_PERFORMANCE.md).

The 424x240 experiment reached approximately 30 FPS through mask and both
overlays; source-header median overlay age was about 110 ms in the stage
measurement. Commands, exact rates, engine padding, and same-frame mask
comparison limits are recorded in
[src/panda_pick_place/panda_pick_place/NATIVE_SEGMENTATION_424x240.md](src/panda_pick_place/panda_pick_place/NATIVE_SEGMENTATION_424x240.md).

The follow-up full-resolution component filter retained approximately 30 FPS
at 424x240. Detailed rates and commands:
[src/panda_pick_place/panda_pick_place/MASK_BOUNDARY_EXPERIMENT_2026-10-14.md](src/panda_pick_place/panda_pick_place/MASK_BOUNDARY_EXPERIMENT_2026-10-14.md).

### Known gaps

- `/closest_panda_human/distance` is computed but not read by
  `pick_place_node` — the arm doesn't slow down or stop when a person gets
  close yet.

## Changes and validation

See [CHANGELOG.md](CHANGELOG.md) for the added model profile, perception
pipeline, dependency patches, and robot recovery changes. The
[installation notes](docs/INSTALLATION_NOTES.md) collect cross-platform
setup problems and fixes. The historical
[workspace comparison](src/panda_pick_place/panda_pick_place/WORKSPACE_DIFF_FROM_GITHUB_2026-10-21.md)
records source evidence, tradeoffs, and the limits of past measurements.

The low-resolution profile changes mask predictions. Automatic recovery
allows operation to resume after a latched error; it does not remove the
cause of reflexes. A prior full-system observation recorded four
warning/recovery intervals in ten minutes. These changes do not establish
collision-avoidance accuracy or error-free operation.
