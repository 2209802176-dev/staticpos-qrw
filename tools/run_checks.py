"""Public host checks: C++ core/adapter, synthetic reference, monitor and GUI."""
from pathlib import Path
import argparse
import json
import os
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--skip-gui', action='store_true')
    args = parser.parse_args()
    runtime = ROOT / 'runtime'
    runtime.mkdir(exist_ok=True)
    environment = os.environ.copy()
    environment['PYTHONDONTWRITEBYTECODE'] = '1'
    environment['PYTHONIOENCODING'] = 'utf-8'
    environment['PYTHONUTF8'] = '1'
    compiler = environment.get('CXX') or shutil.which('g++')
    if not compiler:
        raise SystemExit('A C++11 compiler is required; set CXX to g++ or install GCC.')
    environment['PATH'] = str(Path(compiler).resolve().parent) + os.pathsep + environment.get('PATH', '')
    results = []
    def check(name, command):
        start = time.perf_counter()
        result = subprocess.run(list(map(str, command)), cwd=ROOT, env=environment,
            capture_output=True, text=True, encoding='utf-8', errors='replace')
        results.append(dict(name=name, passed=result.returncode == 0,
            elapsed_s=round(time.perf_counter()-start, 3),
            stdout=result.stdout, stderr=result.stderr))
        (runtime / 'host_checks.json').write_text(json.dumps(dict(
            status='PASS' if all(x['passed'] for x in results) else 'FAIL',
            scope='Host software checks. No serial device, flashing or hardware timing.',
            checks=results), ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'{name}: {"PASS" if result.returncode == 0 else "FAIL"}', flush=True)
        if result.returncode:
            print(result.stdout, result.stderr)
            raise SystemExit(result.returncode)
    extension = '.exe' if os.name == 'nt' else ''
    for name in ('core_test', 'adapter_test', 'replay'):
        source = ROOT / 'tests' / f'{name}.cpp'
        target = runtime / (name + extension)
        check('compile_' + name, [compiler, '-std=c++11', '-O2', '-Wall', '-Wextra',
            '-Werror', '-pedantic', '-I', ROOT / 'core', '-I', ROOT / 'tests/adapter_stub',
            source, '-o', target])
        if name != 'replay':
            check(name, [target])
    example = runtime / ('standalone_example' + extension)
    check('compile_standalone_example', [compiler, '-std=c++11', '-O2', '-Wall', '-Wextra',
        '-Werror', '-pedantic', '-I', ROOT / 'core', ROOT / 'examples/standalone.cpp', '-o', example])
    check('standalone_example', [example])
    check('synthetic_reference', [sys.executable, ROOT / 'tools/validate_replay.py', '--synthetic'])
    check('adapter_protocol', [sys.executable, ROOT / 'tests/test_adapter_protocol.py'])
    check('monitor', [sys.executable, ROOT / 'tools/validate_monitor.py'] + ([] if args.skip_gui else ['--gui']))
    print('Public host checks PASS', flush=True)

if __name__ == '__main__':
    main()
