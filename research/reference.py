"""RTK-free NumPy reference for qrw_w90_k3.

Only source GPS position/quality and past gyro observations enter estimation.
No reference position, measured error, official navigation output, fitted
model, or future sensor observation is an estimator input.
SPDX-License-Identifier: GPL-3.0-or-later
"""
from __future__ import annotations

from pathlib import Path
import numpy as np
import pandas as pd

GPS_FIELDS = ["time_usec", "lat", "lon", "fix_type", "eph", "h_acc",
              "satellites_visible"]
IMU_FIELDS = ["time_usec", "xgyro", "ygyro", "zgyro"]


def scales(lat: float) -> tuple[float, float]:
    p = np.deg2rad(lat)
    d = 1 - 6.6943799901413165e-3 * np.sin(p)**2
    return (np.pi / 180 * 6378137 / np.sqrt(d) * np.cos(p),
            np.pi / 180 * 6378137 * (1 - 6.6943799901413165e-3) / d**1.5)


def quality_sigma(h_acc_mm, eph, satellites):
    h = np.asarray(h_acc_mm, dtype=float) / 1000
    h = np.where(np.isfinite(h) & (h > 0) & (h < 1000), h, 2.)
    hd = np.asarray(eph, dtype=float) / 100
    hd = np.where(np.isfinite(hd) & (hd > 0) & (hd < 655.35), hd, 2.)
    sat = np.asarray(satellites, dtype=float)
    sat = np.where(np.isfinite(sat) & (sat > 0) & (sat < 255), sat, 6.)
    return np.clip(np.maximum(h, .6) * np.clip(hd / 1.2, .7, 3.) *
                   np.clip(np.sqrt(10 / sat), .85, 1.6), .6, 8.)


def motion_weights(gps_t, imu_t, gyro, hold=2):
    norm = np.linalg.norm(np.asarray(gyro, dtype=float) / 1000, axis=1)
    ans = np.ones(len(gps_t))
    j, last = 0, -np.inf
    for i, t in enumerate(gps_t):
        while j < len(imu_t) and imu_t[j] <= t:
            if norm[j] > .08:
                last = imu_t[j]
            j += 1
        if t - last <= hold:
            ans[i] = 4.
    return ans


def huber_center(z, base_w, sigma, k=3., iterations=4):
    center = np.sum(z * base_w[:, None], axis=0) / np.sum(base_w)
    robust = np.ones(len(z))
    completed = 0
    for completed in range(1, iterations + 1):
        r = np.linalg.norm(z - center, axis=1) / np.maximum(sigma, .6)
        robust = np.minimum(1., k / np.maximum(r, 1e-12))
        w = base_w * robust
        new = np.sum(z * w[:, None], axis=0) / np.sum(w)
        if np.linalg.norm(new - center) < 1e-7:
            center = new
            break
        center = new
    return center, robust, completed


def load_raw(gps_path: Path, imu_path: Path):
    # Only the explicit source-field allowlist is parsed into estimator inputs.
    gps = pd.read_csv(gps_path, usecols=GPS_FIELDS).sort_values("time_usec")
    gps = gps.drop_duplicates("time_usec")
    good = ((gps.fix_type >= 3) & (gps.time_usec > 0) &
            (gps.lat.abs() <= 900000000) & (gps.lon.abs() <= 1800000000) &
            ((gps.lat != 0) | (gps.lon != 0)))
    gps = gps.loc[good].reset_index(drop=True)
    imu = pd.read_csv(imu_path, usecols=IMU_FIELDS).sort_values("time_usec")
    imu = imu.drop_duplicates("time_usec").reset_index(drop=True)
    return gps, imu


def estimate(gps, imu):
    """Estimate each prefix using inclusive integer-microsecond boundaries."""
    time_us = gps.time_usec.to_numpy(np.int64)
    lat = gps.lat.to_numpy(float) / 1e7
    lon = gps.lon.to_numpy(float) / 1e7
    es, ns = scales(lat[0])
    x = np.column_stack(((lon - lon[0]) * es, (lat - lat[0]) * ns))
    sigma = quality_sigma(gps.h_acc, gps.eph, gps.satellites_visible)
    motion = motion_weights(
        time_us, imu.time_usec.to_numpy(np.int64),
        imu[["xgyro", "ygyro", "zgyro"]].to_numpy(float),
        hold=2000000)
    variance = sigma * sigma * motion
    n = len(time_us)
    out = np.empty_like(x)
    robust_now, effective_n, newest_weight = np.ones(n), np.ones(n), np.ones(n)
    count, spans, iterations, segments = (np.zeros(n, dtype=int), np.zeros(n),
                                         np.zeros(n, dtype=int), np.zeros(n, dtype=int))
    left, segment = 0, 0
    for i in range(n):
        if i == 0 or time_us[i] - time_us[i - 1] > 10000000:
            left = i
            segment += 1
        while left < i and time_us[i] - time_us[left] > 90000000:
            left += 1
        z = x[left:i + 1]
        v = variance[left:i + 1]
        w = 1 / v
        center, rob, completed = huber_center(z, w, np.sqrt(v))
        out[i], robust_now[i] = center, rob[-1]
        ww = w * rob
        effective_n[i] = ww.sum()**2 / np.sum(ww * ww)
        newest_weight[i] = ww[-1] / ww.sum()
        count[i] = i - left + 1
        spans[i] = (time_us[i] - time_us[left]) / 1e6
        iterations[i], segments[i] = completed, segment
    return dict(time_us=time_us, xy=out, latitude=lat[0] + out[:, 1] / ns,
                longitude=lon[0] + out[:, 0] / es, sigma_m=sigma, lambda_=motion,
                robust_factor=robust_now, newest_weight=newest_weight,
                effective_n=effective_n, window_count=count, window_span_s=spans,
                iterations=iterations, segment_count=segments)


def replay_input(gps, imu) -> str:
    """Merge source timestamps before serializing the C++ replay allowlist."""
    it = imu.time_usec.to_numpy(np.int64)
    gyro = imu[["xgyro", "ygyro", "zgyro"]].to_numpy(float) / 1000
    lines, j = ["R"], 0
    for row in gps.itertuples(index=False):
        stamp = int(row.time_usec)
        while j < len(it) and it[j] <= stamp:
            x, y, z = gyro[j]
            lines.append(f"I {int(it[j])} {x:.17g} {y:.17g} {z:.17g} 1")
            j += 1
        # Unknown quality fields retain their documented finite defaults.
        h = float(row.h_acc) / 1000
        hd = float(row.eph) / 100
        sat = float(row.satellites_visible)
        if not np.isfinite(h):
            h = 0.
        if not np.isfinite(hd):
            hd = 0.
        if not np.isfinite(sat):
            sat = 0.
        lines.append(f"G {stamp} {int(row.lat)} {int(row.lon)} {int(row.fix_type)} "
                     f"{h:.17g} {hd:.17g} {sat:.17g}")
    return "\n".join(lines) + "\n"


def synthetic_raw():
    """Portable generated input; no private project files or surveyed point."""
    count = 1200
    stamps = 1000000 + np.arange(count, dtype=np.int64) * 200000
    stamps[1050:] += 11000000  # New segment while preserving the first origin.
    lat = np.full(count, 100000000, dtype=np.int64)
    lon = np.full(count, 200000000, dtype=np.int64)
    # Low-frequency synthetic drift, one spike, then a sustained position step.
    lon += np.rint(25 * np.sin(np.arange(count) / 43)).astype(np.int64)
    lon[200] += 9120
    lon[500:] += 182
    gps = pd.DataFrame(dict(time_usec=stamps, lat=lat, lon=lon,
        fix_type=np.full(count, 3), eph=120 + (np.arange(count) % 5) * 20,
        h_acc=900 + (np.arange(count) % 7) * 100,
        satellites_visible=7 + np.arange(count) % 6))
    gps.loc[35, ["eph", "h_acc", "satellites_visible"]] = [65535, 0, 255]
    it = 950000 + np.arange(2550, dtype=np.int64) * 100000
    gyro = np.zeros((len(it), 3), dtype=int)
    gyro[(it >= 40000000) & (it < 42000000), 2] = 90
    imu = pd.DataFrame(dict(time_usec=it, xgyro=gyro[:, 0], ygyro=gyro[:, 1], zgyro=gyro[:, 2]))
    return gps, imu
