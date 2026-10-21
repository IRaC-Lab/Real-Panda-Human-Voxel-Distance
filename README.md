# Panda–Human Voxel Distance (Real Robot)

ROS 2 workspace for human segmentation, Panda collision-mesh voxels, and
minimum-distance estimation with a real Panda and RealSense camera.
Companion project:
[IsaacSim-Panda-Human-Voxel-Distance](https://github.com/IRaC-Lab/IsaacSim-Panda-Human-Voxel-Distance).

## Environment

**Hardware**

- CPU: AMD Ryzen 9 5900X
- GPU: NVIDIA RTX 3070 (8 GB VRAM)
- RAM: 32 GB
- Panda arm, System 4.2.2, with a Franka Hand gripper
- Intel RealSense D435i
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

## Update an existing installation

Stop the running ROS nodes and save local source changes before updating:

```bash
cd "$HOME/panda_real_ws"
git pull --ff-only
git lfs pull
```

Older camera and segmentation patches conflict with the current patches.
Move the external source checkouts to a backup outside the workspace;
step 5 below will import the pinned revisions and apply the current patches:

```bash
DEPENDENCY_BACKUP="$(mktemp -d "$HOME/panda_real_ws-dependencies.XXXXXX")"
for dependency in isaac_ros_nvblox franka_arm_ros2; do
    if [ -d "src/$dependency" ]; then
        mv "src/$dependency" "$DEPENDENCY_BACKUP/"
    fi
done
```

Complete [prerequisites](#1-prerequisites) and [runtime dependencies](#2-runtime-dependencies),
then [build the workspace](#5-import-patch-and-build) and
[generate the engine](#6-build-the-segmentation-engine). Keep the existing
libfranka 0.9.2 installation and `source_real_ws` function.
Custom edits in the backed-up dependencies must be reapplied separately.

## Installation

### 1. Prerequisites

Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) and
prepare the desktop GPU stack for
[Isaac ROS 3.2](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/hardware_setup/compute/index.html).
Install TensorRT 10.3 for your CUDA version from
[NVIDIA's TensorRT downloads](https://developer.nvidia.com/tensorrt/download/10x).
Use the matching NVIDIA apt repository for the Python bindings in step 2.

```bash
sudo apt install git-lfs python3-colcon-common-extensions python3-rosdep \
  python3-vcstool build-essential cmake git libpoco-dev libeigen3-dev \
  libgflags-dev libgoogle-glog-dev libgtest-dev libbenchmark-dev
git lfs install
```

On a fresh ROS installation, run `sudo rosdep init` and `rosdep update`.
For a new workspace:

```bash
git clone https://github.com/IRaC-Lab/Real-Panda-Human-Voxel-Distance.git "$HOME/panda_real_ws"
cd "$HOME/panda_real_ws"
git lfs pull
```

### 2. Runtime dependencies

**Isaac ROS.** The `release-3.0` apt channel also supplies the 3.2 packages:

```bash
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
  | sudo gpg --batch --yes --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg

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

**TensorRT Python bindings.** Match the installed TensorRT 10 runtime;
use the system Python for engine generation:

```bash
TRT_DEB_VERSION="$(dpkg-query -W -f='${Version}' libnvinfer10)"
sudo apt install "python3-libnvinfer=$TRT_DEB_VERSION"
/usr/bin/python3 -c 'import tensorrt; print(tensorrt.__version__)'
```

**CV-CUDA (x86_64, CUDA 12).** Install the 0.5.0-beta packages specified by
[Isaac ROS 3.2](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/release-3.2/docker/Dockerfile.base#L260-L275):

```bash
CV_CUDA_DIR="$(mktemp -d)"
CV_CUDA_URL="https://github.com/CVCUDA/CV-CUDA/releases/download/v0.5.0-beta"
curl -fL "$CV_CUDA_URL/nvcv-lib-0.5.0_beta-cuda12-x86_64-linux.deb" -o "$CV_CUDA_DIR/nvcv-lib.deb"
curl -fL "$CV_CUDA_URL/nvcv-dev-0.5.0_beta-cuda12-x86_64-linux.deb" -o "$CV_CUDA_DIR/nvcv-dev.deb"
sudo apt install "$CV_CUDA_DIR/nvcv-lib.deb" "$CV_CUDA_DIR/nvcv-dev.deb"
sudo ldconfig
```

**RealSense USB access:**

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
  | gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo jammy main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update
sudo apt install librealsense2-udev-rules librealsense2-utils
```

### 3. libfranka 0.9.2

```bash
sudo apt remove ros-humble-libfranka   # if installed
sudo apt install ros-humble-ros2-controllers ros-humble-joint-trajectory-controller

git clone --recursive https://github.com/frankarobotics/libfranka.git "$HOME/libfranka"
cd "$HOME/libfranka"
git checkout 0.9.2
git submodule update --init --recursive
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

### 4. Shell environment

Add to `~/.bashrc` and reload it with `source ~/.bashrc`:

```bash
source_real_ws() {
    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
    export LD_LIBRARY_PATH="$HOME/libfranka/build:${LD_LIBRARY_PATH:-}"
    source /opt/ros/humble/setup.bash
    source "$HOME/panda_real_ws/install/setup.bash"
}
```

### 5. Import, patch, and build

Use a fresh terminal with only ROS 2 Humble sourced. The dependency source
directories must be absent; existing installations should follow the
[update steps](#update-an-existing-installation) first.

```bash
source /opt/ros/humble/setup.bash
cd "$HOME/panda_real_ws"
vcs import src < isaac_ros_nvblox.repos
git -C src/isaac_ros_nvblox submodule update --init --recursive
vcs import src < franka_arm_ros2.repos

git -C src/isaac_ros_nvblox apply "$PWD/patches/nvblox_skip_empty_deletion.patch"
git -C src/isaac_ros_nvblox apply "$PWD/patches/realsense_camera_topic_namespace.patch"
git -C src/isaac_ros_nvblox apply "$PWD/patches/vanilla_segmentation_preprocessing.patch"
git -C src/isaac_ros_nvblox apply "$PWD/patches/nvblox_runtime_sync_timestamps.patch"
git -C src/franka_arm_ros2 apply "$PWD/patches/franka_runtime_recovery.patch"
rm -f src/isaac_ros_nvblox/nvblox_examples/realsense_splitter/COLCON_IGNORE

rosdep install --from-paths src --ignore-src -y \
  --skip-keys "isaac_ros_test isaac_ros_test_cmake ament_python"

colcon build --symlink-install \
  --packages-up-to panda_pick_place panda_real_bringup realsense_splitter \
  --cmake-args -DFranka_DIR="$HOME/libfranka/build" -DBUILD_TESTING=OFF
source install/setup.bash
```

### 6. Build the segmentation engine

Generate engines on the target desktop with its installed TensorRT version.
The Run commands use **424x240**; the engine is selected automatically:

```bash
cd "$HOME/panda_real_ws"
MODEL_DIR="$HOME/panda_real_ws/models/peoplesemsegnet/vanilla"
mkdir -p "$MODEL_DIR/1"
/usr/bin/python3 src/panda_pick_place/panda_pick_place/scripts/build_native_segmentation_engine.py \
  --onnx "$MODEL_DIR/peoplesemsegnet_vanilla_unet_dynamic_etlt_fp32.onnx" \
  --output "$MODEL_DIR/1/model_vanilla_v2_0_2_424x240.plan"
```

For the original **960x544** profile:

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx="$MODEL_DIR/peoplesemsegnet_vanilla_unet_dynamic_etlt_fp32.onnx" \
  --saveEngine="$MODEL_DIR/1/model_vanilla_v2_0_2.plan"
```

## Run

Start every terminal with:

```bash
source_real_ws
```

Use the same `ROS_DOMAIN_ID` in all terminals.
Use 424x240 RGB/depth and the engine built in step 6. Start each camera once.

### Desktop + Panda + RealSense D435i

Enable FCI in Franka Desk before starting.

#### Terminal 1 — Panda + TF

```bash
ros2 launch panda_pick_place panda_control.launch.py robot_ip:=172.16.0.2
```

#### Terminal 2 — RealSense

```bash
ros2 launch nvblox_examples_bringup realsense.launch.py \
  run_standalone:=True color_profile:=424x240x30 depth_profile:=424x240x30
```

#### Terminal 3 — ArUco alignment

Choose one mode:

```bash
# While adjusting the camera
ros2 launch panda_camera_alignment aruco_align.launch.py \
  mode:=dynamic camera_mount_frame:=camera0_link
```

```bash
# After fixing the camera mount
ros2 launch panda_camera_alignment aruco_align.launch.py \
  mode:=static num_samples:=15 camera_mount_frame:=camera0_link
```

#### Terminal 4 — People + nvblox + Panda voxels

```bash
ros2 launch panda_real_bringup panda_realsense_people.launch.py \
  run_realsense:=False run_alignment:=False run_rviz:=False \
  run_panda:=True segmentation_resolution:=424x240
```

#### Terminal 5 — RViz

```bash
rviz2 -d ~/panda_real_ws/src/panda_real_bringup/config/panda_realsense_people.rviz
```

Optional Panda debug view: use
`~/panda_real_ws/src/my_people_nvblox_bringup/config/visualization/panda_sphere_debug.rviz`.

#### Terminal 6 — Pick-and-place (optional)

Adjust the waypoints in `pick_place.yaml` for your robot setup before running.

```bash
ros2 run panda_pick_place pick_place_node --ros-args \
  --params-file ~/panda_real_ws/src/panda_pick_place/config/pick_place.yaml
```

### Desktop + RealSense D435i (no Panda)

#### Terminal 1 — RealSense

```bash
ros2 launch nvblox_examples_bringup realsense.launch.py \
  run_standalone:=True color_profile:=424x240x30 depth_profile:=424x240x30
```

#### Terminal 2 — People + nvblox

```bash
ros2 launch panda_real_bringup panda_realsense_people.launch.py \
  run_realsense:=False run_alignment:=False run_rviz:=False \
  run_panda:=False segmentation_resolution:=424x240
```

#### Terminal 3 — Color + depth overlays

```bash
ros2 run rqt_image_view rqt_image_view /nvblox_node/dynamic_color_frame_overlay &
ros2 run rqt_image_view rqt_image_view /nvblox_node/dynamic_depth_frame_overlay
```

### Options

- `segmentation_resolution`: `424x240` or `960x544` (launch default).
  Camera dimensions must not exceed the selected model resolution.
- `vanilla_engine_file_path`: override the automatically selected engine.
- `optimize_perception:=False`: use the original processing with `960x544`.

The distance output is informational; `pick_place_node` does not use it to
slow down or stop the arm.
