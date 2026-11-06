# Building and verification

Run commands from the repository root. Host checks and full firmware builds are separate operations: a host test executable is not flashable firmware.

## Host checks

Use Python 3.11+, a C++11-capable GCC, and the packages in `requirements.txt`. Install Tcl/Tk for the display checks. The `CXX` environment variable selects the host compiler.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
export CXX=g++
python tools/run_checks.py
```

On Windows, use `.venv/Scripts/python.exe` and set `$env:CXX = 'g++.exe'` in PowerShell. A compiler executable path is also accepted. On headless Linux, use `--skip-gui` or supply an Xvfb display session. Check `runtime/host_checks.json` for the checks actually performed.

The public checks default to synthetic data and simulated hardware boundaries. They exercise core math, timestamp/window rules, the actual adapter source, telemetry assembly, recording and the GUI when enabled. They do not connect to or flash a device. For optional user-provided CSV replay, inspect `python tools/validate_replay.py --help` and pass the inputs explicitly. Generated reports stay in the ignored `runtime/` directory.

## Pinned firmware target

| Item | Required value |
| --- | --- |
| ArduPilot | ArduCopter 4.6.3 |
| Upstream commit | `92b0cd788ec29406f26c6f9c31d5ceedbd1cc538` |
| Waf board | `fmuv3` |
| APJ board ID | `9` |
| Cross-compiler | GNU Arm Embedded GCC 10.2.1, 10-2020-q4-major |
| Runtime scope | One GPS, disarmed, fixed position / rotation in place |

The adapter installs only into a dedicated source checkout inside this repository. It checks the upstream commit and scheduler layout before installing the core, hook configuration and adapter. Do not apply it to an unrelated working tree.

## Linux / WSL firmware build

Prepare Git, Python, the supported ArduPilot system build prerequisites and the cross-compiler before running these commands. The Python requirements file does not install the operating-system packages or the Arm toolchain. Use a supported Linux environment following the [official ArduPilot build-environment guide](https://ardupilot.org/dev/docs/building-setup-linux.html).

```bash
python3 -m venv .venv-build
source .venv-build/bin/activate
python3 -m pip install -r requirements-build.txt
python3 tools/bootstrap_source.py
```

`bootstrap_source.py` fetches the official ArduPilot repository, checks out the pinned commit, initializes submodules and applies this project's adapter into `vendor/ardupilot/`. It needs network access for the initial checkout. Downloaded sources and dependencies stay outside the source submission through `.gitignore`.

If the compiler is already on `PATH`, run:

```bash
arm-none-eabi-g++ --version
BUILD_JOBS=4 bash tools/build_firmware.sh
```

Alternatively, unpack the toolchain under the ignored `vendor/toolchains/` directory and select it explicitly:

```bash
export ARM_TOOLCHAIN_BIN="$(pwd)/vendor/toolchains/gcc-arm-none-eabi-10-2020-q4-major/bin"
BUILD_JOBS=4 bash tools/build_firmware.sh
```

The build script uses Waf to configure `fmuv3`, compile and link Copter, then exports and checks the APJ/ELF and source patch. `BUILD_JOBS` controls parallelism. Use Linux-native storage where possible; a large source tree on a Windows-mounted filesystem can make WSL builds slower.

An existing complete upstream checkout can be used through `--local-source` instead of downloading. Its HEAD must equal the pin and its `.git` directory must be self-contained. The script copies that input into this repository's `vendor/` tree and restores the pinned tracked files before applying the adapter. Treat the input as a read-only source; do not point it at a directory containing the destination.

## Outputs and provenance

| Path | Meaning |
| --- | --- |
| `release/arducopter-4.6.3-fmuv3-staticpos-v3.apj` | Flashable full firmware for the stated target |
| `release/arducopter-4.6.3-fmuv3-staticpos-v3.elf` | Linked image for inspection and debugging |
| `release/ardupilot-4.6.3-staticpos-v3.patch` | Adapter/core changes relative to pinned upstream |
| `release/firmware_manifest.json` | Board, source, compiler and artifact hashes |
| `runtime/build_firmware.log` | This local full-build log |
| `runtime/firmware_size.txt` | Linked-image size summary |

Review the manifest before using a firmware file. Rebuild after changing the core, adapter or hook configuration. Retain the matching source, upstream pin, toolchain identity and patch with a distributed firmware artifact; see [LICENSE](../LICENSE) and [NOTICE](../NOTICE).

A successful full build establishes compile/link/package compatibility for that source and toolchain. It does not establish target execution time or positioning accuracy. Initial scheduler reservations are 500 μs for the estimator hook and 150 μs for the telemetry hook. `ST_US`, `ST_MAXUS` and `ST_OVR` expose measured calculation cost once the firmware runs on a target; those measurements are not supplied by host simulation.

## Avoid stale or mixed builds

- Keep build products, runtime caches and upstream sources local.
- Use the same pinned source and compiler when comparing outputs.
- After source changes, rerun host checks and rebuild the full firmware.
- Do not attach an old manifest to a newly built binary.
- If `vendor/ardupilot/` already exists, the bootstrap script intentionally refuses to overwrite it. Review that checkout and use `tools/apply_adapter.py` only when it is the intended pinned private build tree.

Upstream reference: [ArduPilot BUILD.md](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/BUILD.md). Project-specific pins and exported artifact names above take precedence over examples for other boards in the upstream guide.
