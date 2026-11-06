"""Reject tracked datasets, generated output and common credential material."""
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
EXCLUDED_DIRS = {'runtime', 'reports', 'release', 'vendor', 'build', 'monitor_logs',
                 'data', 'datasets', '.venv', '.venv-build', '__pycache__', '.git'}
EXCLUDED_SUFFIXES = {'.csv', '.jsonl', '.tlog', '.ubx', '.obs', '.nav', '.bin',
                     '.apj', '.elf', '.kml', '.kmz', '.zip', '.log'}
SECRET_PATTERNS = [
    re.compile(rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----'),
    re.compile(rb'gh[pousr]_[A-Za-z0-9]{30,}'),
    re.compile(rb'github_pat_[A-Za-z0-9_]{40,}'),
]

def main():
    names = subprocess.check_output(['git', '-C', str(ROOT), 'ls-files', '-z']).split(b'\0')
    errors, count = [], 0
    for encoded in names:
        if not encoded:
            continue
        name = encoded.decode('utf-8')
        relative = Path(name)
        count += 1
        if set(relative.parts) & EXCLUDED_DIRS or relative.suffix.lower() in EXCLUDED_SUFFIXES:
            errors.append(f'Generated output or dataset tracked: {name}')
            continue
        if relative.name == '.env' or relative.name.startswith('.env.'):
            errors.append(f'Environment file tracked: {name}')
            continue
        path = ROOT / relative
        if path.is_file():
            data = path.read_bytes()
            if any(pattern.search(data) for pattern in SECRET_PATTERNS):
                errors.append(f'Credential-like content detected: {name}')
    if not count:
        errors.append('No tracked source files found; stage the intended publication files first.')
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        return 1
    print(f'Publication boundaries PASS: {count} tracked source/support files.')
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
