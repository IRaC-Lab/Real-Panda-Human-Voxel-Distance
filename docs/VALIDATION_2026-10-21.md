# Publication validation — 2026-10-21

These checks cover the prepared source update, not a new live robot or
camera experiment.

| Check | Result |
|---|---|
| Existing perception helper tests, using the prepared source checkout | 8 passed |
| Python syntax parsing | 36 source files passed |
| ROS package manifests | XML parsed successfully |
| Published measurement JSON | Parsed successfully |
| README, changelog, installation notes, and comparison document links | Local targets exist; links work from the repository |
| Dependency patches against pinned source revisions | All five patches apply in README order |
| Patched files compared byte-for-byte with current dependency sources | 8 nvblox files and 5 Franka files match |

The helper test command used ROS Humble and the existing workspace message
packages, with `PYTHONPATH` overridden to select the prepared Python source:

```bash
python3 -m pytest -q -p no:cacheprovider \
  src/panda_pick_place/panda_pick_place/tests/test_perception_helpers.py
```

The patch check recreated original files from the revisions pinned by the
two `.repos` manifests, applied each dependency's patches in order, and
compared the results with the already modified local files. It did not
rebuild the CUDA or robot-driver libraries.

`patches/*.patch` disables only trailing-whitespace checks for patch-file
contents in `.gitattributes`: unified-diff context and removed lines may
need to preserve whitespace from the dependency's original source.
The dependency patches themselves were validated with `git apply --check`.

No new TensorRT engines were built, no system packages were installed,
and no robot motion, error recovery, or camera launch was triggered.
The dated performance and reflex records describe earlier experiments.
