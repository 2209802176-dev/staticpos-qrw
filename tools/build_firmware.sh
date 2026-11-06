#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE="$TASK_ROOT/vendor/ardupilot"
mkdir -p "$TASK_ROOT/runtime/tmp" "$TASK_ROOT/runtime/cache" "$TASK_ROOT/release"
export TMPDIR="$TASK_ROOT/runtime/tmp"
export PYTHONPYCACHEPREFIX="$TASK_ROOT/runtime/pycache"
export XDG_CACHE_HOME="$TASK_ROOT/runtime/cache"
export CCACHE_DIR="$TASK_ROOT/runtime/ccache"
export PYTHONDONTWRITEBYTECODE=1
if [[ -n "${ARM_TOOLCHAIN_BIN:-}" ]]; then export PATH="$ARM_TOOLCHAIN_BIN:$PATH"; fi
exec > >(tee "$TASK_ROOT/runtime/build_firmware.log") 2>&1
command -v arm-none-eabi-g++
test "$(git -C "$SOURCE" rev-parse HEAD)" = '92b0cd788ec29406f26c6f9c31d5ceedbd1cc538'
python3 "$TASK_ROOT/tools/apply_adapter.py" --source "$SOURCE"
python3 "$TASK_ROOT/tools/restore_executable_modes.py" "$SOURCE"
cd "$SOURCE"
python3 ./waf configure --board fmuv3 --no-submodule-update --disable-tests
python3 ./waf copter -j"${BUILD_JOBS:-4}"
cp build/fmuv3/bin/arducopter.apj "$TASK_ROOT/release/arducopter-4.6.3-fmuv3-staticpos-v3.apj"
cp build/fmuv3/bin/arducopter "$TASK_ROOT/release/arducopter-4.6.3-fmuv3-staticpos-v3.elf"
arm-none-eabi-size build/fmuv3/bin/arducopter > "$TASK_ROOT/runtime/firmware_size.txt"
arm-none-eabi-nm -S build/fmuv3/bin/arducopter > "$TASK_ROOT/runtime/firmware_symbols.txt"
python3 "$TASK_ROOT/tools/package_release.py"
