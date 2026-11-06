"""Install V3 into an explicitly selected, disposable pinned upstream tree."""
from pathlib import Path
import argparse
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PIN = '92b0cd788ec29406f26c6f9c31d5ceedbd1cc538'

def install(source):
    source = Path(source).resolve()
    if not source.is_relative_to(ROOT.resolve()):
        raise SystemExit('Source must be inside this V3 repository; old versions are read-only.')
    top = Path(subprocess.check_output(['git', '-C', str(source), 'rev-parse', '--show-toplevel'], text=True).strip()).resolve()
    gitdir = Path(subprocess.check_output(['git', '-C', str(source), 'rev-parse', '--absolute-git-dir'], text=True).strip()).resolve()
    if top != source or not gitdir.is_relative_to(source):
        raise SystemExit('Refusing external worktree or Git metadata outside this private source tree.')
    actual = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    if actual != PIN:
        raise SystemExit(f'Unexpected upstream commit: {actual}')
    vehicle = source / 'ArduCopter'
    def original(name):
        return subprocess.check_output(['git', '-C', str(source), 'show', f'HEAD:ArduCopter/{name}']).decode('utf-8')
    shutil.copyfile(ROOT / 'ardupilot/UserCode.cpp', vehicle / 'UserCode.cpp')
    shutil.copyfile(ROOT / 'core/RobustWindowEstimator.h', vehicle / 'RobustWindowEstimator.h')
    config = original('APM_Config.h') + '\n' + (ROOT / 'ardupilot/APM_Config.h.fragment').read_text(encoding='utf-8')
    (vehicle / 'APM_Config.h').write_text(config, encoding='utf-8', newline='\n')
    schedule = original('Copter.cpp')
    for before, after in [
        ('SCHED_TASK(userhook_MediumLoop,   10,     75, 159)',
         'SCHED_TASK(userhook_MediumLoop,   10,    500, 159)'),
        ('SCHED_TASK(userhook_50Hz,         50,     75, 156)',
         'SCHED_TASK(userhook_50Hz,         50,    150, 156)'),
    ]:
        if schedule.count(before) != 1:
            raise SystemExit('Unexpected upstream scheduler: refusing a partial adapter install.')
        schedule = schedule.replace(before, after)
    (vehicle / 'Copter.cpp').write_text(schedule, encoding='utf-8', newline='\n')
    return source

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT / 'vendor/ardupilot')
    print(install(parser.parse_args().source))
