# Changelog

## 2026-10-21 — Additional perception profiles and runtime fixes

This update adds the locally developed perception and robot-control changes
on top of `65960499ecd0c93b588e51c55e729e350a75afae`.

### Added

- An optional 424×240 PeopleSemSegNet TensorRT engine builder, with internal
  448×256 padding/crop and the original ONNX weights. The default profile
  remains 960×544; generated engines remain local to each GPU/TensorRT setup.
- A paired RGB/CameraInfo input gate, bounded outstanding segmentation work,
  native-resolution mask processing, and a topic rate/header-age profiler.
- Full-resolution connected-component filtering, helper tests, measurement
  summaries, and the raw JSON measurements supporting those summaries.
- Reproducible patches for the previously local-only nvblox synchronization
  and overlay timestamps, plus Franka broadcaster/recovery diagnostics.
- Cross-platform installation notes and a historical workspace comparison
  with explicit tradeoffs and validation limits.

### Changed

- Route nvblox depth directly from the camera when splitter metadata is
  unavailable, disable the unused splitter, and guard against duplicate
  local launches of the same D435i.
- Map color/masks at camera resolution, avoid duplicate component filtering,
  and expose resolution, input-gating, and mask-filter options in the launch.
- Set per-joint trajectory goal tolerances and request error recovery before
  moves, handling the service's normal `No errors` response.
- Reduce the nvblox processing queue and integration range in the local
  profile; keep its timing/rate diagnostics visible in configuration.
- Correct patch paths, rosdep exclusions, missing native build dependencies,
  and the RealSense repository-key instructions in the installation guide.

### Validation and limits

The 2026-10-14 camera-only experiments observed approximately 30 FPS with
the smaller model profile. These are receive-rate measurements on that test
system, not x86 benchmarks or new measurements made while publishing.
The low-resolution mask differs from the original profile and has not been
established as more accurate against ground-truth labels. A 3 m integration
range and bounded queues trade coverage/completeness for processing work
and latency. A prior full-system ten-minute observation recorded four
warning/recovery intervals; automatic recovery does not remove the cause
of reflexes or establish error-free robot operation.

Detailed records:

- [Publication validation](docs/VALIDATION_2026-10-21.md)
- [Installation notes](docs/INSTALLATION_NOTES.md)
- [Workspace comparison and tradeoffs](src/panda_pick_place/panda_pick_place/WORKSPACE_DIFF_FROM_GITHUB_2026-10-21.md)
- [424×240 experiment](src/panda_pick_place/panda_pick_place/NATIVE_SEGMENTATION_424x240.md)
- [Mask boundary experiment](src/panda_pick_place/panda_pick_place/MASK_BOUNDARY_EXPERIMENT_2026-10-14.md)
- [Full-system observation](src/panda_pick_place/panda_pick_place/REFLEX_OBSERVATION_2026-10-20_STAGE11_WHOLE_SYSTEM.md)
