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
curl -sSf https://librealsense.realsenseai.com/Debian/librealsense.pgp \
  | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo jammy main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update
sudo apt install librealsense2-udev-rules librealsense2-utils
```

### 3. libfranka 0.9.2

```bash
sudo apt remove ros-humble-libfranka   # wrong version for Panda, if present
sudo apt install ros-humble-ros2-controllers ros-humble-joint-trajectory-controller
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev

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
git -C src/isaac_ros_nvblox apply patches/nvblox_skip_empty_deletion.patch
git -C src/isaac_ros_nvblox apply patches/realsense_camera_topic_namespace.patch
git -C src/isaac_ros_nvblox apply patches/vanilla_segmentation_preprocessing.patch
vcs import src < franka_arm_ros2.repos

# nvblox_examples' realsense_splitter ships COLCON_IGNORE'd upstream; the
# real-camera depth pipeline needs it.
rm -f src/isaac_ros_nvblox/nvblox_examples/realsense_splitter/COLCON_IGNORE

rosdep install --from-paths src --ignore-src -y

colcon build --symlink-install \
  --packages-up-to panda_pick_place panda_real_bringup realsense_splitter \
  --cmake-args -DFranka_DIR="$HOME/libfranka/build" -DBUILD_TESTING=OFF
source install/setup.bash
```

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
  --saveEngine=1/model_vanilla_v2_0_2.plan \
  --fp16
```

Drop `--fp16` for an FP32 engine.

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
  color_profile:=848x480x30 \
  depth_profile:=848x480x30
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

### Known gaps

- `/closest_panda_human/distance` is computed but not read by
  `pick_place_node` — the arm doesn't slow down or stop when a person gets
  close yet.
