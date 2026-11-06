"""Restore executable bits from Git when copying Windows files into Linux."""
from pathlib import Path
import subprocess
import sys

root = Path(sys.argv[1]).resolve()
for record in subprocess.check_output(['git', '-C', str(root), 'ls-files', '-s', '-z']).split(b'\0'):
    if record:
        metadata, name = record.split(b'\t', 1)
        path = root / name.decode('utf-8')
        if metadata.split()[0] == b'100755' and path.is_file():
            data = path.read_bytes()
            if data.startswith(b'#!') and b'\r\n' in data:
                path.write_bytes(data.replace(b'\r\n', b'\n'))
            path.chmod(path.stat().st_mode | 0o111)
