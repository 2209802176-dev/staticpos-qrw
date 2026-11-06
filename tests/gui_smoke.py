"""Exercise all V3 GUI diagnostics with offline synthetic telemetry; no transport.

SPDX-License-Identifier: GPL-3.0-or-later
"""
from pathlib import Path
import sys
import time
sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_monitor import fields, gps, named
from static_monitor import MonitorApp
import tkinter as tk

root = tk.Tk()
root.withdraw()
app = MonitorApp(root)
app.aggregator.connect()
now = time.monotonic()
app.aggregator.ingest(gps(), now)
for name, value in fields(ST_LAM=4., ST_IMU=2, ST_OVR=3, ST_MAXUS=600.).items():
    app.aggregator.ingest(named(name, value), now)
app.render(app.aggregator.snapshot(now))
assert app.raw_labels['lat'].cget('text') == '10.0000000'
assert app.opt_labels['lat'].cget('text') == '10.0000001'
assert app.official_labels['lat'].cget('text') == '10.0000010'
assert app.diagnostic_labels['imu_state'].cget('text') == '转向门控'
assert app.diagnostic_labels['quality_sigma_m'].cget('text') == '1.200'
assert app.diagnostic_labels['budget_overruns'].cget('text') == '3'
assert app.diagnostic_labels['max_compute_us'].cget('text') == '600.000'
app.aggregator.ingest(named('ST_STATE', 3, timestamp=2000), now+.1)
app.render(app.aggregator.snapshot(now+.1))
assert app.opt_labels['lat'].cget('text') == '—'
assert app.diagnostic_labels['quality_sigma_m'].cget('text') == '—'
root.after(350, app.close)
root.mainloop()
print('V3 GUI valid/invalid rendering PASS; no transport or recording opened.')
