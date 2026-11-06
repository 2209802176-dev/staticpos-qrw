# Contributing

Contributions that make the implementation easier to reproduce, inspect and test are welcome. This repository implements `qrw_w90_k3` for a single-GPS, disarmed, fixed-position experiment. Begin with the [README](README.md), [core API](core/README.md), [quick start](docs/QUICKSTART.zh-CN.md) and [build guide](docs/BUILDING.md).

## Propose a focused change

Use an issue to describe a reproducible bug or a concrete feature. Search existing issues first. For an algorithm change, explain the problem, intended behavior and comparison against the current baseline before presenting a claimed accuracy improvement.

A useful pull request states:

- The concrete trigger and resulting behavior.
- Which algorithm, adapter, telemetry, display or documentation files change.
- How the change was checked, including commands and material limitations.
- Any new dependency, upstream revision, parameter or protocol change.

Keep unrelated formatting changes separate when they make a functional diff harder to review. There is no guaranteed response or review schedule.

## Validate the change

Install the requirements and select a C++11-capable GCC through `CXX` when needed, then run:

```bash
python tools/run_checks.py
```

Use `--skip-gui` only when a display session is unavailable, and state that the GUI was not checked. Public checks must work with the repository's synthetic inputs. An external private dataset must not be a prerequisite for a contribution.

For changes to the core or adapter, also build the pinned full `fmuv3` firmware when possible. If the build or hardware experiment was not performed, state that explicitly. Update matching source and artifact manifests after a firmware rebuild; do not reuse evidence from an older binary.

## Preserve the algorithm contract

The current baseline uses a 90-second causal window, GPS-quality scaling, gyro-triggered weight reduction, Huber threshold 3 and at most four reweighting rounds. The public core documentation defines timestamp units, inclusive boundaries, reset behavior and failure handling.

Changes to these rules need explicit configuration/version documentation and same-input comparisons. Keep a changed estimator distinguishable from the confirmed baseline. Reference coordinates, true error and future samples must not enter a causal estimate. A changed coordinate origin must not pull the estimate toward a scoring target.

Maintain fixed-capacity failure reporting and clear handling of invalid, duplicated, reordered and stale input. Telemetry changes must preserve exact-batch assembly and source/result identity. Add a meaningful regression case for the behavior being fixed rather than assertions that merely repeat the implementation.

## Protect the evidence

Use explicitly synthetic examples, such as `10°N, 20°E`, in public demonstrations. Before attaching an input or log, remove real coordinates, device identifiers, personal paths and credentials. Do not commit downloaded toolchains, generated runtime files, raw hardware logs or local configuration.

Report positioning accuracy, centered scatter and availability separately. A smoother trace is not proof of a more accurate mean position. Quality scales are empirical weights; effective weight count is not an independent sample count or a calibrated confidence interval. Preserve negative results and explain timestamp matching when comparing interfaces.

## Licensing and conduct

Submit only material you have the right to contribute under **GPL-3.0-or-later**. Preserve the notices and licenses of reused code and document its source. See [LICENSE](LICENSE), [NOTICE](NOTICE) and the [Code of Conduct](CODE_OF_CONDUCT.md).

For a potentially exploitable security issue, follow [SECURITY.md](SECURITY.md) instead of publishing technical details in an ordinary issue.
