# StaticPos QRW

**Quality-weighted robust GPS windows for static-position research on ArduCopter.**

[中文](README.md) · [Quick start](docs/QUICKSTART.zh-CN.md) · [Build instructions](docs/BUILDING.md) · [Troubleshooting](docs/TROUBLESHOOTING.md)

[![Host checks](https://github.com/2209802176-dev/staticpos-qrw/actions/workflows/ci.yml/badge.svg)](https://github.com/2209802176-dev/staticpos-qrw/actions/workflows/ci.yml)
[![GPL-3.0-or-later](https://img.shields.io/badge/license-GPL--3.0--or--later-blue)](LICENSE)

StaticPos QRW provides a standalone C++11 estimator, an ArduCopter integration, a Chinese telemetry monitor, and a NumPy reference with synthetic checks. The flight controller computes an independent horizontal position; the PC displays and records it.

The integration targets **ArduCopter 4.6.3, fmuv3, one M8N receiver, disarmed static-position and in-place rotation experiments**. It leaves the original navigation and control interfaces intact. Hardware timing, telemetry continuity, available memory and positioning performance require measurements on your target.

## How it works

1. Keep valid GPS observations in an inclusive 90-second window.
2. Form a bounded empirical quality scale from horizontal accuracy, HDOP and satellite count.
3. Apply a variance multiplier of four to observations associated with a preceding gyro norm above 0.08 rad/s within two seconds.
4. Compute a Huber-weighted center with `k=3` and at most four reweighting iterations.

Only current and preceding GPS/IMU inputs are used. Reference coordinates, RTK, official navigation outputs and future observations do not enter the estimator. IMU acceleration is not integrated.

The core uses fixed storage for 1024 GPS points, no heap allocation, and approximately 45 KB for the estimator object. Capacity losses explicitly invalidate output rather than silently shortening the window. A source gap greater than ten seconds starts a new segment.

## Try the software

Use Python 3.11+, Tk and a C++11 GCC compiler:

```bash
git clone https://github.com/2209802176-dev/staticpos-qrw.git
cd staticpos-qrw
python -m venv .venv
# Activate .venv for your shell.
python -m pip install -r requirements.txt
python tools/run_checks.py
```

Set `CXX` if GCC is not on PATH. On headless Linux, use `xvfb-run -a python tools/run_checks.py`, or pass `--skip-gui` for checks without a display. The checks use synthetic inputs and simulated device boundaries; no serial port is opened.

## Integrate and build

Include `core/RobustWindowEstimator.h` to use the estimator independently. Supply GPS and gyro samples on the same monotonic microsecond clock and check validity before consuming coordinates. See the [core API](core/README.md) and [standalone synthetic example](examples/README.md).

For the flight controller, follow [BUILDING.md](docs/BUILDING.md). The source preparation script pins the ArduPilot commit and installs the adapter into an isolated local checkout. Building produces a full fmuv3 APJ plus source/firmware checksums. Windows users can start the monitor through `启动显示器.cmd` after installing its dependencies.

The algorithm task is nominally 10 Hz. Telemetry uses a separate 50 Hz hook to send one field per call from an immutable snapshot, targeting one complete batch per second. The monitor checks batch identity, result source time, version, diagnostics and expiry. See the [interface contract](ardupilot/接入接口说明.md).

## Scope and limitations

The finite window trades response speed for smoothing. This single-receiver model cannot guarantee removal of common persistent bias or distinguish genuine translation from drift. The quality scale is a heuristic, and effective weight support is not an independent sample count or confidence interval. This is static research software, not a replacement flight-navigation stack.

Only algorithm source and supporting files are published. No experimental datasets, acquisition logs, result reports, credentials or downloaded dependencies are included. Generated output remains local and is ignored by Git.

## Contributing and license

Read [CONTRIBUTING.md](CONTRIBUTING.md), [SECURITY.md](SECURITY.md), [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md) and [CHANGELOG.md](CHANGELOG.md). Use synthetic reproductions when reporting issues.

Licensed under **GPL-3.0-or-later**. ArduPilot is a separate upstream project; attribution and the pinned integration revision are in [NOTICE](NOTICE). This project is not an official ArduPilot release.
