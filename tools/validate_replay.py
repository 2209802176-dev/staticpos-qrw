"""Validate C++ replay against the RTK-free NumPy reference.

The default input is generated synthetic data. Optional GPS/IMU CSV files must
be supplied explicitly and are never copied or hashed into validation output.
SPDX-License-Identifier: GPL-3.0-or-later
"""
from __future__ import annotations

import argparse
import io
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT / "research"))
import numpy as np
from reference import estimate, load_raw, replay_input, synthetic_raw

COLUMNS = ["time_us", "result", "east_m", "north_m", "latitude", "longitude",
           "sigma_m", "lambda_", "robust_factor", "newest_weight", "effective_n",
           "window_count", "window_span_s", "update_count", "segment_count",
           "overflow_count", "iterations"]


def run(command, *, env, input_text=None):
    result = subprocess.run(list(map(str, command)), cwd=ROOT, env=env,
                            input=input_text, capture_output=True, text=True,
                            encoding="utf-8", errors="replace", timeout=300)
    if result.returncode:
        raise RuntimeError(f"Command failed ({result.returncode})\n"
                           f"{result.stdout}\n{result.stderr}")
    return result.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gps-csv", type=Path, help="Explicit user-provided raw GPS CSV")
    parser.add_argument("--imu-csv", type=Path, help="Explicit user-provided raw IMU CSV")
    parser.add_argument("--synthetic", action="store_true", help="Use generated input (the default)")
    parser.add_argument("--compiler", default=os.environ.get("CXX") or shutil.which("g++"))
    args = parser.parse_args()
    if bool(args.gps_csv) != bool(args.imu_csv):
        parser.error("--gps-csv and --imu-csv must be supplied together")
    if args.synthetic and args.gps_csv:
        parser.error("--synthetic cannot be combined with input CSV files")
    if not args.compiler:
        parser.error("A C++11 compiler is required; set CXX or add g++ to PATH")
    if args.gps_csv:
        if not args.gps_csv.is_file() or not args.imu_csv.is_file():
            parser.error("Both explicitly supplied CSV files must exist")
        gps, imu = load_raw(args.gps_csv, args.imu_csv)
        input_kind = "user_provided"
    else:
        gps, imu = synthetic_raw()
        input_kind = "generated_synthetic"
    if gps.empty:
        parser.error("The input has no valid GPS observations")

    runtime = ROOT / "runtime"
    runtime.mkdir(exist_ok=True)
    report_path = runtime / "replay_validation.json"
    environment = os.environ.copy()
    environment["PATH"] = str(Path(args.compiler).parent) + os.pathsep + environment.get("PATH", "")
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    if os.name == "nt":
        import ctypes
        ctypes.windll.kernel32.SetErrorMode(0x8003)
    extension = ".exe" if os.name == "nt" else ""
    executable = runtime / ("replay" + extension)
    compile_command = [args.compiler, "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                       "-pedantic", "-I", ROOT / "core", ROOT / "tests" / "replay.cpp",
                       "-o", executable]
    started = time.monotonic()
    run(compile_command, env=environment)
    single_executable = runtime / ("replay_single_constants" + extension)
    run(compile_command[:-1] + [single_executable, "-fsingle-precision-constant"], env=environment)

    reference = estimate(gps, imu)
    source_input = replay_input(gps, imu)
    output = run([executable], env=environment, input_text=source_input)
    single_output = run([single_executable], env=environment, input_text=source_input)
    assert single_output == output, "Target constant flag changes C++ results"
    values = np.loadtxt(io.StringIO(output), delimiter=",", ndmin=2)
    assert values.shape == (len(gps), len(COLUMNS)), values.shape
    data = dict(zip(COLUMNS, values.T))
    error = np.max(np.abs(np.column_stack((data["east_m"], data["north_m"])) -
                          reference["xy"]), axis=1)
    assert np.all(data["result"] == 0)
    assert np.array_equal(data["time_us"], reference["time_us"])
    assert np.max(error) < 1e-5, ("Local XY mismatch", np.max(error))
    for name in ("sigma_m", "lambda_", "robust_factor", "newest_weight", "effective_n",
                 "window_count", "window_span_s", "segment_count", "iterations"):
        tol = 1e-8 if name == "effective_n" else 1e-9
        assert np.max(np.abs(data[name] - reference[name])) < tol, name
    assert np.array_equal(data["update_count"], np.arange(1, len(gps) + 1))
    assert np.all(data["overflow_count"] == 0)
    for fraction in (.25, .61):
        n = max(1, int(len(gps) * fraction))
        p = gps.iloc[:n]
        prefix = estimate(p, imu.loc[imu.time_usec <= int(p.time_usec.iloc[-1])])
        assert np.max(np.abs(prefix["xy"] - reference["xy"][:n])) < 1e-12
        cpp_prefix = np.loadtxt(io.StringIO(run([executable], env=environment,
            input_text=replay_input(p, imu))), delimiter=",", ndmin=2)
        assert np.array_equal(cpp_prefix, values[:n])
    report = dict(status="PASS", algorithm="qrw_w90_k3", input_kind=input_kind,
        gps_epochs=len(gps), imu_epochs=len(imu), local_xy_tolerance_m=1e-5,
        maximum_local_xy_error_m=float(np.max(error)),
        exact_window_definition="inclusive integer microseconds: age <= 90000000",
        rtk_or_error_inputs=False, experimental_data_required=False,
        single_precision_constant_build_identical=True, prefix_causality=True,
        python_version=platform.python_version(), numpy_version=np.__version__,
        compiler=run([args.compiler, "--version"], env=environment).splitlines()[0],
        elapsed_s=time.monotonic()-started,
        max_window_count=int(data["window_count"].max()),
        max_window_span_s=float(data["window_span_s"].max()))
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"PASS {input_kind}: {len(gps)} GPS; max_xy_error={np.max(error):.3g} m; "
          "report=runtime/replay_validation.json")


if __name__ == "__main__":
    main()
