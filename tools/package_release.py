"""Validate the full APJ, then write source/firmware hashes and upstream patch."""
from pathlib import Path
import base64
import argparse
import difflib
import hashlib
import json
import subprocess
import zlib

ROOT = Path(__file__).resolve().parents[1]
PIN = '92b0cd788ec29406f26c6f9c31d5ceedbd1cc538'

def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--built-source', type=Path, default=ROOT / 'vendor/ardupilot')
    args = parser.parse_args()
    release = ROOT / 'release'
    release.mkdir(exist_ok=True)
    apj = release / 'arducopter-4.6.3-fmuv3-staticpos-v3.apj'
    elf = release / 'arducopter-4.6.3-fmuv3-staticpos-v3.elf'
    data = json.loads(apj.read_text(encoding='utf-8'))
    image = zlib.decompress(base64.b64decode(data['image']))
    assert data['board_id'] == 9, 'Incorrect APJ board'
    assert b'STPOS V3 qrw_w90_k3' in image, 'V3 marker missing'
    assert len(image) == data['image_size'], 'APJ size mismatch'
    assert len(image) <= 2080768, 'fmuv3 nominal flash capacity exceeded'
    assert elf.read_bytes()[:4] == b'\x7fELF', 'ELF output missing'
    source = args.built_source.resolve()
    actual = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    assert actual == PIN, 'Build source commit mismatch'
    for local, built in [('core/RobustWindowEstimator.h', 'ArduCopter/RobustWindowEstimator.h'),
                         ('ardupilot/UserCode.cpp', 'ArduCopter/UserCode.cpp')]:
        assert (ROOT / local).read_bytes() == (source / built).read_bytes(), 'Source changed during build: ' + local
    original_config = subprocess.check_output(['git', '-C', str(source), 'show', 'HEAD:ArduCopter/APM_Config.h']).decode('utf-8')
    expected_config = original_config + '\n' + (ROOT / 'ardupilot/APM_Config.h.fragment').read_text(encoding='utf-8')
    assert (source / 'ArduCopter/APM_Config.h').read_text(encoding='utf-8') == expected_config, 'Configuration fragment changed during build'
    files = ['ArduCopter/UserCode.cpp', 'ArduCopter/APM_Config.h', 'ArduCopter/Copter.cpp']
    patch = subprocess.check_output(['git', '-C', str(source), 'diff', '--no-ext-diff', 'HEAD', '--', *files]).decode('utf-8')
    header = ROOT / 'core/RobustWindowEstimator.h'
    patch += 'diff --git a/ArduCopter/RobustWindowEstimator.h b/ArduCopter/RobustWindowEstimator.h\nnew file mode 100644\n'
    patch += ''.join(difflib.unified_diff([], header.read_text(encoding='utf-8').splitlines(keepends=True),
        fromfile='/dev/null', tofile='b/ArduCopter/RobustWindowEstimator.h'))
    patch_file = release / 'ardupilot-4.6.3-staticpos-v3.patch'
    patch_file.write_text(patch, encoding='utf-8', newline='\n')
    manifest = dict(target='ArduCopter 4.6.3 / fmuv3 / APJ board_id 9',
        algorithm='qrw_w90_k3', algorithm_version=3, upstream_commit=PIN,
        compiler=subprocess.check_output(['arm-none-eabi-g++', '--version'], text=True).splitlines()[0],
        firmware_file=apj.name, file_sha256=sha(apj), image_sha256=hashlib.sha256(image).hexdigest(),
        image_bytes=len(image), nominal_flash_capacity_bytes=2080768,
        elf_file=elf.name, elf_sha256=sha(elf), patch_sha256=sha(patch_file),
        firmware_build='PASS', on_hardware_tested=False,
        validation='Full firmware compile/link/package plus APJ board/image/marker checks; target timing not measured.',
        navigation_gps_replaced=False,
        source_files={str(p.relative_to(ROOT)).replace('\\', '/'): sha(p) for p in [header,
            ROOT / 'ardupilot/UserCode.cpp', ROOT / 'ardupilot/APM_Config.h.fragment']})
    manifest['compiled_adapter_files'] = {name: sha(source / name) for name in [*files, 'ArduCopter/RobustWindowEstimator.h']}
    (release / 'firmware_manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps({k:manifest[k] for k in ('firmware_build', 'image_bytes', 'on_hardware_tested')}))

if __name__ == '__main__':
    main()
