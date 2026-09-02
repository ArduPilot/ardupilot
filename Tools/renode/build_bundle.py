#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Build a source-checkout-independent ArduPilot Renode runtime tree."""

import argparse
import hashlib
import json
import os
import shutil
import stat
import sys
import tempfile

from pathlib import Path

RUNTIME_FILES = (
    'README.md',
    'THIRD_PARTY_NOTICES.md',
    'build_bundle.py',
    'chibios_gdb.py',
    'device_emulator.py',
    'dfu.py',
    'driver_catalog.py',
    'extract_logs.py',
    'fat_image.py',
    'gen_board.py',
    'launch.py',
    'physics_protocol.py',
    'renode_data.py',
    'renode_firmware.py',
    'run.py',
    'usbip_attach.py',
)
RUNTIME_TREES = {
    'peripherals': {'.cs'},
    'platforms': {'.repl'},
    'scripts': {'.resc'},
}
HWDEF_SUFFIXES = {'.dat', '.inc', '.parm'}
HWDEF_SCRIPT_SUFFIXES = {'.py', '.h'}
DMAMUX_HEADERS = (
    'modules/ChibiOS/os/hal/ports/STM32/STM32G4xx/stm32_dmamux.h',
    'modules/ChibiOS/os/hal/ports/STM32/STM32H7xx/stm32_dmamux.h',
    'modules/ChibiOS/os/hal/ports/STM32/STM32L4xx+/stm32_dmamux.h',
)


def _copy_file(root, destination, relative):
    source = root / relative
    if not source.is_file():
        raise FileNotFoundError(source)
    target = destination / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)


def _copy_tree(source, target, suffixes=None):
    if not source.is_dir():
        raise FileNotFoundError(source)
    for path in sorted(source.rglob('*')):
        if (path.is_file() and
                (suffixes is None or path.suffix.lower() in suffixes) and
                '__pycache__' not in path.parts):
            destination = target / path.relative_to(source)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, destination)


def _sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _manifest(destination, physics_name):
    files = []
    for path in sorted(destination.rglob('*')):
        if path.is_file() and path.name != 'bundle.json':
            files.append({
                'path': path.relative_to(destination).as_posix(),
                'size': path.stat().st_size,
                'sha256': _sha256(path),
            })
    return {
        'format': 1,
        'launcher': 'Tools/renode/launch.py',
        'physics': 'Tools/renode/bin/%s' % physics_name,
        'files': files,
    }


def _claim_output(destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    lock = destination.with_name('.%s.bundle-lock' % destination.name)
    descriptor = os.open(lock, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        if os.name == 'nt':
            import msvcrt
            if os.fstat(descriptor).st_size == 0:
                os.write(descriptor, b'\0')
            os.lseek(descriptor, 0, os.SEEK_SET)
            msvcrt.locking(descriptor, msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as error:
        os.close(descriptor)
        raise FileExistsError(
            'another bundle build owns %s' % lock) from error
    try:
        if destination.exists():
            raise FileExistsError('%s already exists' % destination)
        os.ftruncate(descriptor, 0)
        os.lseek(descriptor, 0, os.SEEK_SET)
        os.write(descriptor, ('pid=%u\n' % os.getpid()).encode())
    except Exception:
        os.close(descriptor)
        raise
    return descriptor


def create_bundle(root, destination, physics_binary):
    """Create an atomic runtime tree and return its generated manifest."""
    root = Path(root).resolve()
    destination = Path(destination).resolve()
    physics_binary = Path(physics_binary).resolve()
    if not physics_binary.is_file():
        raise FileNotFoundError(physics_binary)
    physics_name = ('renode-physics.exe'
                    if physics_binary.suffix.lower() == '.exe'
                    else 'renode-physics')

    lock_descriptor = _claim_output(destination)
    try:
        temporary = Path(tempfile.mkdtemp(
            prefix='.%s-' % destination.name, dir=destination.parent))
        try:
            renode_source = root / 'Tools' / 'renode'
            for name in RUNTIME_FILES:
                _copy_file(root, temporary, Path('Tools') / 'renode' / name)
            for name, suffixes in RUNTIME_TREES.items():
                _copy_tree(renode_source / name,
                           temporary / 'Tools' / 'renode' / name, suffixes)

            hwdef = root / 'libraries' / 'AP_HAL_ChibiOS' / 'hwdef'
            _copy_tree(hwdef, temporary / hwdef.relative_to(root),
                       HWDEF_SUFFIXES)
            _copy_tree(hwdef / 'scripts',
                       temporary / hwdef.relative_to(root) / 'scripts',
                       HWDEF_SCRIPT_SUFFIXES)
            _copy_tree(hwdef / 'common',
                       temporary / hwdef.relative_to(root) / 'common')
            _copy_file(root, temporary,
                       Path('libraries/AP_HAL/hwdef/scripts/hwdef.py'))
            _copy_file(root, temporary,
                       Path('Tools/AP_Bootloader/board_types.txt'))
            for relative in DMAMUX_HEADERS:
                _copy_file(root, temporary, Path(relative))
            _copy_tree(root / 'Tools' / 'IO_Firmware',
                       temporary / 'Tools' / 'IO_Firmware', {'.bin'})
            _copy_tree(root / 'Tools' / 'bootloaders',
                       temporary / 'Tools' / 'bootloaders', {'.bin'})

            installed_physics = (temporary / 'Tools' / 'renode' / 'bin' /
                                 physics_name)
            installed_physics.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(physics_binary, installed_physics)
            installed_physics.chmod(
                installed_physics.stat().st_mode | stat.S_IXUSR)

            manifest = _manifest(temporary, physics_name)
            (temporary / 'bundle.json').write_text(
                json.dumps(manifest, indent=2, sort_keys=True) + '\n')
            if destination.exists():
                raise FileExistsError('%s was created during the build' %
                                      destination)
            os.replace(temporary, destination)
            return manifest
        except Exception:
            shutil.rmtree(temporary, ignore_errors=True)
            raise
    finally:
        os.close(lock_descriptor)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', help='new bundle directory to create')
    parser.add_argument('--physics-binary', required=True,
                        help='native renode-physics executable to install')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    try:
        manifest = create_bundle(root, args.output, args.physics_binary)
    except (FileExistsError, FileNotFoundError, OSError) as error:
        parser.error(str(error))
    total = sum(entry['size'] for entry in manifest['files'])
    print('created %s with %u files (%u bytes)' %
          (Path(args.output).resolve(), len(manifest['files']), total))
    return 0


if __name__ == '__main__':
    sys.exit(main())
