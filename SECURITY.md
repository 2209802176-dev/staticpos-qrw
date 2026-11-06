# Security policy

This repository contains experimental positioning software, an ArduPilot adapter and a local telemetry viewer. Its supported operating scope is documented in the README. A completed host check or firmware build does not establish deployment safety, target timing or flight-navigation suitability.

## Report a vulnerability

If the repository's **Security → Report a vulnerability** action is available, use that private GitHub reporting form. Private vulnerability reporting is a repository setting; this file alone does not mean the feature is enabled.

If that action is unavailable, open an issue asking for a private reporting channel. Include only a broad description such as “telemetry parsing security report”; do not include exploit steps, affected credentials, precise locations or private logs in that public issue. There is no dedicated mailbox or guaranteed response time stated by this project.

A private report is most useful when it includes:

- The affected commit, component and configuration.
- Preconditions and the impact you observed.
- Minimal reproduction steps using synthetic input where possible.
- A proposed fix or mitigation, if available.

Do not include real device credentials or personal location data unless a private channel has been established and those details are necessary to reproduce the issue.

## Scope and status

Please reproduce against the current source when practical. Historical snapshots do not have a separately promised security-maintenance window. Distinguish a software vulnerability from an unsupported experiment, an accuracy limitation or an ordinary bug; uncertainty about that distinction can be explained in the report.

Telemetry validity checks and source filtering do not authenticate a transport peer. This project does not distribute signing keys, claim signed releases or provide an audited security certification. Dependency and upstream vulnerabilities should also be reported through the affected upstream project's own policy when appropriate.

GitHub's reporting workflow is described in the [official private reporting guide](https://docs.github.com/en/code-security/how-tos/report-and-fix-vulnerabilities/report-privately).
