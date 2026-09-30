# Panda–Human Voxel Distance (Real Robot)

ROS 2 workspace that runs the Panda–human minimum-distance system against a
**physical** Panda + RealSense rig — no Isaac Sim. Adds a real Franka driver
and a joint-space pick-and-place motion on top of the same perception stack
(PeopleSemSegNet people voxels, TF-transformed Panda collision-mesh voxels,
filtered minimum distance) used in the companion Isaac Sim repo,
[IsaacSim-Panda-Human-Voxel-Distance](https://github.com/IRaC-Lab/IsaacSim-Panda-Human-Voxel-Distance).
`my_people_nvblox_bringup` here is a copy of that repo's package, not a
shared dependency — the two repos are independent.

## Hardware

- Panda arm, System 4.2.2, with a Franka Hand gripper
- Intel RealSense (tested: D435-class), fixed or on an ArUco-tracked mount
- NVIDIA Jetson AGX Orin — match JetPack/L4T and the Isaac ROS release
  actually installed on your board; the apt channel below is a starting
  point, not a pin
- Ethernet link to the Panda control box for libfranka's real-time (FCI)
  connection

## Why not the official `franka_ros2` driver

Panda (System 4.x) needs **libfranka 0.9.x**. The official
[franka_ros2](https://github.com/frankaemika/franka_ros2) driver targets FR3
and newer libfranka, and doesn't support Panda on ROS 2. This workspace uses
the community fork [LCAS/franka_arm_ros2](https://github.com/LCAS/franka_arm_ros2),
tested against Panda + libfranka 0.9.2 + Humble + ros2_control.

## Installation

### 1. Prerequisites

- Install ROS 2 Humble (desktop or base) following the
  [official ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html).
- Install the standard ROS 2 build tools: `python3-colcon-common-extensions`,
  `python3-rosdep`, `python3-vcstool` (and run `rosdep init` / `rosdep update`
  if this is a fresh ROS 2 install).
- [git-lfs](https://git-lfs.com/): `git lfs install` once per machine, then
  `git lfs pull` after cloning (the vanilla PeopleSemSegNet `.onnx` under
  `models/peoplesemsegnet/vanilla/` is LFS-tracked — without this you'll get
  a small pointer file instead of the actual 119 MB of weights).

### 2. Isaac ROS apt packages (NITROS)

Match the release channel to what's validated for your JetPack/L4T version;
see the [Isaac ROS release notes](https://nvidia-isaac-ros.github.io/releases/index.html).

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

### 3. libfranka 0.9.2

Built standalone (not a colcon package), pinned to the exact release Panda's
FCI expects:

```bash
sudo apt remove ros-humble-libfranka   # if present: wrong version for Panda
sudo apt install ros-humble-ros2-controllers ros-humble-joint-trajectory-controller

git clone --recursive https://github.com/frankarobotics/libfranka.git ~/libfranka
cd ~/libfranka
git checkout 0.9.2
git submodule update --init --recursive
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j"$(nproc)"
```

### 4. `~/.bashrc` environment

Add this as a function, not a permanent overlay — every command in this
README's `## Run` section starts with `source_real_ws` to switch that one
terminal onto this workspace. A plain shell (before you call it) stays on
whatever ROS 2 setup is already in `~/.bashrc`, so this is safe to add
regardless of what else is installed on the machine.

```bash
source_real_ws() {
    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
    export LD_LIBRARY_PATH="$HOME/libfranka/build:${LD_LIBRARY_PATH:-}"
    source /opt/ros/humble/setup.bash
    source "$HOME/panda_real_ws/install/setup.bash"
}
```

Append that to `~/.bashrc`, then `source ~/.bashrc` (or open a new terminal)
before using any command below.

### 5. Workspace setup

```bash
cd "$HOME/panda_real_ws"
vcs import src < isaac_ros_nvblox.repos
git -C src/isaac_ros_nvblox submodule update --init --recursive
git -C src/isaac_ros_nvblox apply patches/nvblox_skip_empty_deletion.patch
git -C src/isaac_ros_nvblox apply patches/realsense_camera_topic_namespace.patch
git -C src/isaac_ros_nvblox apply patches/vanilla_segmentation_preprocessing.patch
vcs import src < franka_arm_ros2.repos

# Upstream ships this package COLCON_IGNORE'd by default; the real-camera depth
# pipeline (Terminal 2/4) needs it to split depth/infra1/infra2 off the RealSense
# driver's combined stream.
rm -f src/isaac_ros_nvblox/nvblox_examples/realsense_splitter/COLCON_IGNORE

colcon build --symlink-install \
  --packages-up-to panda_pick_place panda_real_bringup realsense_splitter \
  --cmake-args -DFranka_DIR="$HOME/libfranka/build" -DBUILD_TESTING=OFF
source install/setup.bash
```

### 6. Regenerate the TensorRT engine

`models/peoplesemsegnet/vanilla/1/model_vanilla_v2_0_2.plan` is a TensorRT
engine, tied to the exact GPU + TensorRT version it was built on, so it's
intentionally not tracked in git — only the source `.onnx` weights (under
`models/peoplesemsegnet/vanilla/`) are. Rebuild it with `trtexec` (installed
as part of TensorRT):

```bash
cd "$HOME/panda_real_ws/models/peoplesemsegnet/vanilla"
mkdir -p 1
/usr/src/tensorrt/bin/trtexec \
  --onnx=peoplesemsegnet_vanilla_unet_dynamic_etlt_fp32.onnx \
  --saveEngine=1/model_vanilla_v2_0_2.plan \
  --fp16
```

Drop `--fp16` to build an FP32 engine instead.

* * *

## Run

Each terminal starts with `source_real_ws` (the `.bashrc` helper from Step 4).
Pick ONE of the two setups below — they don't share terminals, and
`panda_realsense_people.launch.py` picks the right TF frame (`panda_link0` vs
`camera0_link`) automatically from `run_panda`, so nothing else changes
between them.

### With the Panda arm (Desktop + Panda + RealSense)

On the Desk web UI, activate FCI before Terminal 1.

#### Terminal 1 — Panda driver + TF + pick-and-place controller

```bash
source_real_ws
ros2 launch panda_pick_place panda_control.launch.py robot_ip:=172.16.0.2
```

#### Terminal 2 — RealSense

```bash
source_real_ws
ros2 launch nvblox_examples_bringup realsense.launch.py \
  run_standalone:=True \
  color_profile:=848x480x30 \
  depth_profile:=848x480x30
```

#### Terminal 3 — ArUco camera alignment

Use `mode:=dynamic` while the camera mount isn't fixed yet; switch to
`mode:=static` (with `num_samples:=15`) once it's permanently mounted.

```bash
source_real_ws
ros2 launch panda_camera_alignment aruco_align.launch.py \
  mode:=dynamic \
  camera_mount_frame:=camera0_link
```

#### Terminal 4 — People segmentation + nvblox + Panda/human distance

`vanilla_engine_file_path` already defaults to
`~/panda_real_ws/models/peoplesemsegnet/vanilla/1/model_vanilla_v2_0_2.plan`; pass
it explicitly only if your engine lives elsewhere.

```bash
source_real_ws
ros2 launch panda_real_bringup panda_realsense_people.launch.py \
  run_realsense:=False \
  run_alignment:=False \
  run_rviz:=False
```

#### Terminal 5 — RViz

Full monitoring view:

```bash
source_real_ws
rviz2 -d ~/panda_real_ws/src/panda_real_bringup/config/panda_realsense_people.rviz
```

Or, for a lighter arm-only debug view (Panda RobotModel + collision spheres,
no camera/segmentation required — just Terminal 1 and Terminal 4's TF-driven
nodes):

```bash
source_real_ws
rviz2 -d ~/panda_real_ws/src/my_people_nvblox_bringup/config/visualization/panda_sphere_debug.rviz
```

#### Terminal 6 (optional) — Pick-and-place motion

Left/right joint-space pick-and-place, repeating until Ctrl+C (which returns
the arm to its home pose before stopping; a second Ctrl+C halts in place
instead). See `panda_pick_place/config/pick_place.yaml` to adjust waypoints,
speed, and gripper force before running on real hardware.

```bash
source_real_ws
ros2 run panda_pick_place pick_place_node --ros-args \
  --params-file ~/panda_real_ws/src/panda_pick_place/config/pick_place.yaml
```

### Camera-only, no Panda (Desktop + RealSense)

For testing the perception stack on its own. `run_panda:=False` disables the
Panda-only nodes and switches nvblox's `global_frame` to `camera0_link`
automatically — no other arguments change.

#### Terminal 1 — RealSense

```bash
source_real_ws
ros2 launch nvblox_examples_bringup realsense.launch.py \
  run_standalone:=True \
  color_profile:=848x480x30 \
  depth_profile:=848x480x30
```

#### Terminal 2 — People segmentation + nvblox

```bash
source_real_ws
ros2 launch panda_real_bringup panda_realsense_people.launch.py \
  run_realsense:=False \
  run_alignment:=False \
  run_rviz:=False \
  run_panda:=False
```

#### Terminal 3 — Color + depth overlay preview

```bash
source_real_ws
ros2 run rqt_image_view rqt_image_view /nvblox_node/dynamic_color_frame_overlay &
ros2 run rqt_image_view rqt_image_view /nvblox_node/dynamic_depth_frame_overlay
```

### Known gaps

- The Panda/human minimum distance (`/closest_panda_human/distance`) is
  computed but not read by `pick_place_node` — nothing in that node's code
  subscribes to it, so the arm doesn't slow down or stop when a person gets
  close, even though both nodes run side by side. Planned next step.
