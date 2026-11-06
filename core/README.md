# Algorithm core

`RobustWindowEstimator.h` is a standalone C++11 implementation of `qrw_w90_k3`: a 90-second quality-weighted GPS window with gyro-based turn downweighting and up to four Huber reweighting iterations. It uses fixed arrays and double arithmetic, without heap allocation, file access, surveyed coordinates, or navigation writeback.

## Inputs and use

```cpp
staticpos_v3::RobustWindowEstimator estimator;
estimator.update_imu(source_us, gx_rad_s, gy_rad_s, gz_rad_s, healthy);
auto result = estimator.update_gps(source_us, lat_e7, lon_e7, fix,
                                 horizontal_accuracy_m, hdop, satellites);
if (result == staticpos_v3::UPDATE_OK && estimator.valid(now_us)) {
    // Publish latitude_e7()/longitude_e7() as independent coordinates.
}
```

GPS and IMU use the same extended 64-bit boot-microsecond clock. `MillisClock` can extend a sampled 32-bit millisecond clock across its normal rollover and rejects backward jumps. Reset explicitly after a device reboot. The caller must deduplicate receiver epochs, including cached fixes from before an explicit reset.

The first valid GPS establishes the local east/north origin. A GPS source gap greater than 10 seconds clears the window while retaining that session origin. Explicit `reset()` clears the origin, sensor history and counters; publication remains invalid until a valid GPS update is accepted.

## Fixed algorithm

Quality scale, in metres:

```text
sigma = clip(max(h_acc_m, 0.6)
             * clip(HDOP / 1.2, 0.7, 3)
             * clip(sqrt(10 / satellites), 0.85, 1.6), 0.6, 8)
```

Invalid quality values use finite defaults: horizontal accuracy 2 m, HDOP 2, and satellite count 6. This scale defines an empirical weight; it is not a calibrated noise covariance.

Gyro magnitude strictly greater than 0.08 rad/s sets `lambda=4` for observations within the following two seconds, including the endpoint; otherwise `lambda=1`. This label remains attached to each GPS observation. Recent IMU entries with source times later than the GPS epoch do not enter that epoch's estimate. The fixed 256-entry IMU history is designed for the 10 Hz adapter.

Each window starts at the base-weighted centre, with weights `1/(sigma²*lambda)`. Huber weights use the two-dimensional residual divided by `sigma*sqrt(lambda)`, threshold 3, at most four iterations, and an early-stop displacement below `1e-7` m. This is a stationary-position estimator, without acceleration integration, EKF prediction, or absolute-bias correction.

Window membership uses integer microseconds: `age_us <= 90000000`. Thus the point exactly 90 seconds old is included without a floating-seconds subtraction ambiguity. Non-exact constants are explicitly converted from long-double literals so ArduPilot's `-fsingle-precision-constant` does not change them. Inlier Huber calculations skip unnecessary square roots, and the inverse variance is cached on insertion.

## Capacity and validity

The fixed GPS capacity is 1024 points: a 10 Hz inclusive 90-second window normally needs 901. When capacity is exhausted, the core records the omitted observation and returns `UPDATE_OVERFLOW`. Publication remains invalid until all omitted observations have expired from the mathematical window, a new segment begins, or the caller resets. It never silently shortens the 90-second window.

Duplicate and out-of-order GPS epochs do not refresh the result or its age. Invalid fixes, coordinates or zero timestamps invalidate publication without inserting the observation. `valid()` also checks the latest GPS source age, with a default maximum of two seconds.

Diagnostics include the latest `sigma_m`, `lambda`, `robust_factor`, normalized `newest_weight`, Kish `effective_n`, `window_count`, `window_span_s`, `update_count`, `segment_count`, `overflow_count` and `history_complete`. Kish support is not a count of independent samples or an accuracy interval. After overflow, old weight diagnostics must not be published as current valid calculations.

`imu_valid` separately indicates whether the latest preceding IMU entry is healthy and at most one second old. A prior turn label retains its existing two-second lifetime even if a later IMU entry is unhealthy. Low gyro readings do not prove the device has not translated.

## Verification

`python tools/validate_replay.py` generates synthetic inputs, compiles C++ replay binaries, and compares every result against `research/reference.py`, including prefix causality and the target constant-precision flag. Synthetic coordinates are explicitly chosen as 10° latitude / 20° longitude and are not observations. Tests and reports require no experimental dataset; generated outputs stay in ignored `runtime/`.
