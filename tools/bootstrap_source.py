"""Prepare a V3-only upstream checkout, either online or from a local source."""
from pathlib import Path
import argparse
import shutil
import subprocess
from apply_adapter import ROOT, PIN, install

URL = 'https://github.com/ArduPilot/ardupilot.git'

def run(*args):
    subprocess.run(list(map(str, args)), check=True)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--local-source', type=Path)
    args = parser.parse_args()
    destination = ROOT / 'vendor/ardupilot'
    if destination.exists():
        raise SystemExit('Destination exists. Use apply_adapter.py to update this V3 checkout.')
    destination.parent.mkdir(parents=True, exist_ok=True)
    if args.local_source:
        original = args.local_source.resolve()
        if destination.resolve().is_relative_to(original):
            raise SystemExit('Destination must not be nested inside the local input checkout.')
        if not (original / '.git').is_dir():
            raise SystemExit('Local input must have its own .git directory; external worktrees are not copied.')
        actual = subprocess.check_output(['git', '-C', str(original), 'rev-parse', 'HEAD'], text=True).strip()
        if actual != PIN:
            raise SystemExit('Local source commit does not match the pinned upstream.')
        # Copy excludes all old build products; no hardlinks or shared writable files.
        shutil.copytree(original, destination, ignore=shutil.ignore_patterns(
            'build', '__pycache__', 'node_modules', '.waf*', '.lock-waf*'))
        copied_top = Path(subprocess.check_output(['git', '-C', str(destination), 'rev-parse', '--show-toplevel'], text=True).strip()).resolve()
        copied_git = Path(subprocess.check_output(['git', '-C', str(destination), 'rev-parse', '--absolute-git-dir'], text=True).strip()).resolve()
        if copied_top != destination.resolve() or not copied_git.is_relative_to(destination.resolve()):
            raise SystemExit('Copied Git metadata points outside V3; refusing reset.')
        run('git', '-C', destination, 'config', 'core.autocrlf', 'false')
        # Restore upstream tracked files before applying V3; old UserCode is replaced.
        run('git', '-C', destination, 'reset', '--hard', PIN)
    else:
        run('git', 'clone', '--filter=blob:none', URL, destination)
        run('git', '-C', destination, 'config', 'core.autocrlf', 'false')
        run('git', '-C', destination, 'checkout', '--detach', PIN)
        run('git', '-C', destination, 'submodule', 'update', '--init', '--recursive')
    install(destination)
    print('V3 source prepared. Firmware compilation is a separate step.')

if __name__ == '__main__':
    main()
