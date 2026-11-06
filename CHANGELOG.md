# Changelog

Changes are documented by source state. An entry here does not by itself mean that a GitHub tag or binary release has been published.

## Unreleased

### Added

- A standalone C++11, fixed-capacity implementation of the `qrw_w90_k3` baseline: 90-second causal window, GPS-quality weighting, gyro rotation gate and Huber reweighting.
- An ArduCopter 4.6.3 / `fmuv3` adapter with independent coordinates, explicit invalid states, timed telemetry snapshots and per-update onboard diagnostics.
- A Chinese desktop viewer for raw GPS, original navigation-interface output and the independent estimate, with quality, motion, weight, source-time and compute-cost diagnostics.
- Exact-batch telemetry assembly, duplicate/out-of-order protection and separate message, event and CSV recording.
- Synthetic NumPy reference checks, C++ core/adapter checks, adapter-to-viewer protocol checks and optional hidden-GUI checks.
- Pinned-source preparation, full-firmware build and artifact-manifest tools.
- Quick-start, build, troubleshooting and contribution guides, plus issue and pull-request templates.

### Defined

- The 90-second window includes its integer-microsecond boundary. GPS source gaps exceeding 10 seconds start a new observation segment.
- Rotation preserves historical observations; its weight tag remains attached to each observation until that observation leaves the window.
- Capacity exhaustion is explicit. The estimator does not silently shorten the configured window.
- Snapshot sequence, estimator-update count and GPS source time have separate meanings.
- Public examples and automated checks use synthetic data; private acquisition logs and local credentials are outside the source distribution.

### Validation scope

Host checks and full-firmware compilation have separate evidence. Real-board timing, serial continuity and positioning accuracy remain hardware-validation tasks. The estimator is intended for disarmed fixed-position and rotation-in-place experiments; it does not replace the navigation/control solution or provide general moving-flight inertial navigation.
