"""Run offline display tests and optionally the hidden GUI rendering check.

SPDX-License-Identifier: GPL-3.0-or-later
"""
from pathlib import Path
import argparse
from datetime import datetime, timezone
import hashlib
import io
import json
import platform
import subprocess
import sys
import unittest

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true', help='Also run hidden GUI valid/invalid rendering checks')
    args = parser.parse_args()
    suite = unittest.defaultTestLoader.discover(str(ROOT/'tests'), pattern='test_monitor.py')
    output = io.StringIO()
    result = unittest.TextTestRunner(stream=output, verbosity=2).run(suite)
    gui = dict(status='NOT_RUN', reason='Enable --gui on a host with Tcl/Tk and a display session.')
    if args.gui:
        command = [sys.executable, str(ROOT/'tests'/'gui_smoke.py')]
        run = subprocess.run(command, text=True, encoding='utf-8', capture_output=True, timeout=30)
        gui = dict(status='PASS' if run.returncode == 0 else 'FAIL',
                   exit_code=run.returncode, output=run.stdout.strip(), stderr=run.stderr.strip())
    passed = result.wasSuccessful() and gui['status'] != 'FAIL'
    files = ['monitor/aggregation.py', 'monitor/static_monitor.py',
             'tests/test_monitor.py', 'tests/gui_smoke.py', 'tools/validate_monitor.py']
    report = dict(status='PASS' if passed else 'FAIL',
                  generated_utc=datetime.now(timezone.utc).isoformat(),
                  python=platform.python_version(), platform=platform.platform(),
                  tests_run=result.testsRun, failures=len(result.failures), errors=len(result.errors),
                  gui=gui, opened_hardware_connection=False, hardware_validated=False,
                  source_sha256={name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
                                 for name in files})
    target = ROOT/'runtime'/'monitor_validation.json'
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(f"Monitor {report['status']}: {result.testsRun} tests, GUI {gui['status']}")
    if not passed:
        print(output.getvalue())
        print(gui)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
