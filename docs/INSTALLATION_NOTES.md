# Installation and runtime notes

These observations were collected while installing and running the real
Panda workspace. This document retains issues that may also affect x86 PCs;
board-specific kernel, power-mode, NIC and arm64 packaging recipes are
excluded. Follow the current README for the installation commands and
patch order. Historical performance observations are not x86 benchmarks.

## Repository key and build dependencies

The old RealSense `.pgp` download did not match the repository signing key
in the recorded installation. The README now uses the ASCII key followed
by `gpg --dearmor`, as documented in the
[upstream installation guide](https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md).

`git -C src/isaac_ros_nvblox apply patches/example.patch` resolves the patch
path inside the dependency checkout. The README uses absolute patch paths
from the workspace root instead.

The tested rosdep sources did not resolve `isaac_ros_test`,
`isaac_ros_test_cmake`, or `ament_python`. The first two are test-only keys
in the affected packages; this workspace builds with `BUILD_TESTING=OFF`.
The README documents the observed skip list rather than silently ignoring
all dependency errors. Native nvblox CMake dependencies also require
`libgflags-dev`, `libgoogle-glog-dev`, `libgtest-dev`, and `libbenchmark-dev`.

## CV-CUDA runtime and ABI compatibility

Isaac ROS tensor preprocessing failed to load without `libcvcuda.so.0` /
`libnvcv_types.so.0`. Installing a newer, incompatible CV-CUDA release then
failed with an undefined `cvcudaConvertToSubmit` symbol. Match the runtime
to the installed Isaac ROS release, CUDA version, and CPU architecture.

The recorded Isaac ROS 3.2.10 setup used CV-CUDA v0.5.0-beta. The
[release-3.2 Dockerfile](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/release-3.2/docker/Dockerfile.base)
also pins that release. For its CUDA 12 x86_64 configuration, its runtime
package is `nvcv-lib-0.5.0_beta-cuda12-x86_64-linux.deb`; the aarch64 package
is different. Do not install an arbitrary latest release to satisfy only
the library filename.

## Camera metadata, depth routing, and process lifetime

The recorded D435i/firmware/SDK combination did not report
`frame_emitter_mode`, so `realsense_splitter_node` forwarded no depth.
Changing the emitter setting or rebuilding with the RSUSB backend did not
resolve that metadata gap. Its exact platform dependence remains unknown.
The current people pipeline bypasses the splitter and reads raw depth;
it does not run cuVSLAM. Reassess that decision if using splitter IR outputs.

Do not load the perception components repeatedly into a container left
running from an older launch. In that configuration, stopping the loader
launch alone can leave components alive and later loads can stack duplicate
nodes. Restart the owning container as well, and check for duplicate
`nvblox_node` entries when interpreting topic rates.

Two processes opening one physical RealSense can disrupt streaming even
if they use different ROS domains. The local camera launch adds a serial
lock and a check for open video handles before opening the device.

## Engine validation and source timestamps

FP16 output was uniformly foreground in the recorded test; FP32 produced
a person-shaped mask. The precise cause and x86 impact are unverified.
Validate every GPU/TensorRT engine with actual images before selecting it.
The optional 424×240 engine uses unchanged weights, internal padding/crop,
and a separate filename; its predictions differ from the 960×544 profile.

The packaged nvblox runtime patch changes color/mask synchronization from
ExactTime to ApproximateTime, enlarges its synchronization queue, and
carries source timestamps into the overlays. This enables useful header
age measurements. Header age includes all upstream timing and is not the
compute duration of one stage or an independently measured capture delay.
The larger synchronization queue and smaller processing queue have
different purposes and should be assessed separately when tuning latency.

## Panda control and recovery

The imported broadcaster read `frequency` but attempted publication on
every controller update. The Franka patch limits those attempts to the
configured frequency. It also warns when hardware writes are skipped due
to a latched error, and reduces the no-error recovery message to DEBUG.

The pick/place node calls the recovery service before moves. The response
`success=False, error='No errors'` means there is no error to recover; it
is accepted as a normal result. Automatic recovery does not fix the source
of reflexes. The full-system observation still recorded warning/recovery
intervals, as detailed in the changelog and observation document.

Trajectory goal tolerances must be specified per joint. A bare
`constraints.goal` setting did not configure those joint tolerances;
the real and fake configurations now set `panda_joint1.goal` through
`panda_joint7.goal` explicitly.

Real-time scheduling permissions and static IP configuration also apply
on x86. The robot-facing interface needs its own static address because
the robot does not provide DHCP. Source the workspace in every terminal
before resolving its packages and RViz resources. If manually assigning
CPU affinity, account for recreation of libfranka control threads after
recovery rather than assuming a one-shot TID assignment persists.

## Reproducing the current source changes

The README applies all five dependency patches to the pinned source
revisions. The new runtime patches capture modifications that previously
existed only inside ignored dependency directories. Generated TensorRT
engines, local build outputs, and raw terminal logs are not required source
files for the update. Measurement JSON and written observation summaries
are included separately from runtime code.
