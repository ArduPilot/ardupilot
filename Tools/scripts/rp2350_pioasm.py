#!/usr/bin/env python3
'''
Assemble the RP2350 PIO programs and install the Raspberry Pi tools for them.

The .pio sources in libraries/AP_HAL_ChibiOS/rp2350/pio are assembled with
pioasm into the .pio.h headers next to them, which are committed so that an
ordinary build never needs pioasm.

    rp2350_pioasm.py            regenerate every .pio.h
    rp2350_pioasm.py --check    fail if any .pio.h does not match its source
    rp2350_pioasm.py --install  install pioasm and picotool for this host

pioasm is found from --pioasm, then $PIOASM, then the --install directory,
then PATH. It must be the pinned version, since its version is written into
every header it generates.

AP_FLAKE8_CLEAN
'''

import argparse
import hashlib
import io
import os
import platform
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.request
import zipfile

TOOLS_VERSION = '2.2.0'
RELEASE_URL = 'https://github.com/raspberrypi/pico-sdk-tools/releases/download/v2.2.0-3/'

# (asset, sha256) per tool and host, from the v2.2.0-3 release
ASSETS = {
    'pioasm': {
        'x86_64-lin': ('pico-sdk-tools-2.2.0-x86_64-lin.tar.gz',
                       'f8c34e99af693fdcf420c2ecdd3f0204d5bf4f6bc774249983413bb1e0934000'),
        'aarch64-lin': ('pico-sdk-tools-2.2.0-aarch64-lin.tar.gz',
                        '4517d78021af73231bbce3ecf725433a4ba61574275bbba780461a2df93d839e'),
        'mac': ('pico-sdk-tools-2.2.0-mac.zip',
                '586ded1697b49e4c23c106f23b77ba43cc24c9d2cd417ce10be884a3047b2b63'),
    },
    'picotool': {
        'x86_64-lin': ('picotool-2.2.0-a4-x86_64-lin.tar.gz',
                       'f4a6784fbb862520b797bfb3302c5b94f47664692d85d03c0a7fbee98065568d'),
        'aarch64-lin': ('picotool-2.2.0-a4-aarch64-lin.tar.gz',
                        'c77292acc4c5b22f6b1b80ee9e4c4c176e98d77f152eaaa5996c0818d6bbfb9f'),
        'mac': ('picotool-2.2.0-a4-mac.zip',
                '06dc51c9a187b1f20c159dec754777416d88b06190d248b7b00e8e17432cd782'),
    },
}

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
PIO_DIR = os.path.join(ROOT, 'libraries', 'AP_HAL_ChibiOS', 'rp2350', 'pio')
DEFAULT_INSTALL_DIR = os.path.join(os.path.expanduser('~'), '.local', 'opt', 'pico-sdk-tools')
VERSION_ARGS = {'pioasm': '--version', 'picotool': 'version'}


def host_key():
    system = platform.system()
    machine = platform.machine().lower()
    if system == 'Darwin':
        # universal binaries
        return 'mac'
    if system == 'Linux' and machine in ('x86_64', 'amd64'):
        return 'x86_64-lin'
    if system == 'Linux' and machine in ('aarch64', 'arm64'):
        return 'aarch64-lin'
    sys.exit('rp2350_pioasm: no prebuilt Raspberry Pi tools for %s %s' % (system, machine))


def tool_version(binary, tool):
    try:
        return subprocess.run([binary, VERSION_ARGS[tool]], capture_output=True, text=True).stdout.strip()
    except OSError:
        return ''


def install(tool, dest):
    '''download, verify and unpack one tool into dest/<tool>/, replacing another version'''
    asset, sha256 = ASSETS[tool][host_key()]
    binary = os.path.join(dest, tool, tool)
    if os.path.exists(binary):
        if TOOLS_VERSION in tool_version(binary, tool):
            print('%s %s already installed at %s' % (tool, TOOLS_VERSION, binary))
            return binary
        shutil.rmtree(os.path.join(dest, tool))
    print('Downloading %s' % asset)
    with urllib.request.urlopen(RELEASE_URL + asset) as response:
        data = response.read()
    if hashlib.sha256(data).hexdigest() != sha256:
        sys.exit('rp2350_pioasm: checksum mismatch for %s' % asset)
    os.makedirs(dest, exist_ok=True)
    if asset.endswith('.zip'):
        with zipfile.ZipFile(io.BytesIO(data)) as z:
            z.extractall(dest)
    else:
        with tarfile.open(fileobj=io.BytesIO(data)) as t:
            for member in t.getmembers():
                if member.name.startswith('/') or '..' in member.name.split('/'):
                    sys.exit('rp2350_pioasm: unexpected %s in %s' % (member.name, asset))
            if hasattr(tarfile, 'data_filter'):
                t.extractall(dest, filter='data')
            else:
                t.extractall(dest)
    # zipfile does not keep the execute bit
    os.chmod(binary, 0o755)
    print('Installed %s at %s' % (tool, binary))
    return binary


def find_pioasm(args):
    candidates = [args.pioasm, os.environ.get('PIOASM'),
                  os.path.join(args.install_dir, 'pioasm', 'pioasm'),
                  shutil.which('pioasm')]
    for c in candidates:
        if c and os.path.exists(c):
            break
    else:
        sys.exit('rp2350_pioasm: pioasm not found; run Tools/scripts/rp2350_pioasm.py --install')
    version = tool_version(c, 'pioasm')
    if TOOLS_VERSION not in version:
        sys.exit('rp2350_pioasm: %s reports "%s", need version %s; run with --install' % (c, version, TOOLS_VERSION))
    return c


def assemble(pioasm, source):
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'out.h')
        subprocess.run([pioasm, '-o', 'c-sdk', source, out], check=True)
        with open(out) as f:
            return f.read()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--check', action='store_true', help='fail if a generated header is out of date')
    parser.add_argument('--install', action='store_true', help='install pioasm and picotool for this host')
    parser.add_argument('--install-dir', default=DEFAULT_INSTALL_DIR, help='where --install puts the tools')
    parser.add_argument('--pioasm', help='pioasm binary to use')
    args = parser.parse_args()

    if args.install:
        for tool in ('pioasm', 'picotool'):
            install(tool, args.install_dir)
        print('Add to PATH: %s %s' % (os.path.join(args.install_dir, 'pioasm'),
                                      os.path.join(args.install_dir, 'picotool')))
        return

    pioasm = find_pioasm(args)
    stale = []
    for name in sorted(os.listdir(PIO_DIR)):
        if not name.endswith('.pio'):
            continue
        source = os.path.join(PIO_DIR, name)
        header = source + '.h'
        generated = assemble(pioasm, source)
        current = open(header).read() if os.path.exists(header) else None
        if generated == current:
            continue
        if args.check:
            stale.append(os.path.relpath(header, ROOT))
        else:
            with open(header, 'w') as f:
                f.write(generated)
            print('Updated %s' % os.path.relpath(header, ROOT))
    if stale:
        sys.exit('rp2350_pioasm: out of date, run Tools/scripts/rp2350_pioasm.py:\n  ' + '\n  '.join(stale))


if __name__ == '__main__':
    main()
