#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Install the CI Rust toolchain without requiring curl or wget."""

import argparse
import os
import platform
import shutil
import subprocess
import tempfile
import urllib.request

from pathlib import Path


def host_triple():
    machines = {'x86_64': 'x86_64', 'AMD64': 'x86_64', 'aarch64': 'aarch64', 'arm64': 'aarch64'}
    systems = {'Linux': 'unknown-linux-gnu', 'Darwin': 'apple-darwin'}
    try:
        return f'{machines[platform.machine()]}-{systems[platform.system()]}'
    except KeyError as error:
        raise ValueError('Rust CI installation requires a supported Linux or macOS host') from error


def install(directory):
    directory = directory.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    cargo = directory / 'cargo'
    rustup = directory / 'rustup'
    environment = dict(os.environ, CARGO_HOME=str(cargo), RUSTUP_HOME=str(rustup))
    version = Path(__file__).with_name('rust-toolchain-version').read_text().strip()
    url = f'https://static.rust-lang.org/rustup/dist/{host_triple()}/rustup-init'
    with tempfile.TemporaryDirectory(prefix='bootstrap-', dir=directory) as temporary:
        installer = Path(temporary) / 'rustup-init'
        with urllib.request.urlopen(url, timeout=60) as response, installer.open('wb') as output:
            shutil.copyfileobj(response, output)
        installer.chmod(0o700)
        subprocess.run([str(installer), '-y', '--no-modify-path', '--profile', 'minimal',
                        '--default-toolchain', version], env=environment, check=True)
    return subprocess.check_output([str(cargo / 'bin/rustup'), 'which', '--toolchain', version, 'rustc'],
                                   env=environment, text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, required=True, help='isolated toolchain installation directory')
    args = parser.parse_args()
    compiler = install(args.directory)
    if os.environ.get('GITHUB_ENV'):
        with Path(os.environ['GITHUB_ENV']).open('a') as output:
            output.write(f'RUSTC={compiler}\n')
    print(compiler)


if __name__ == '__main__':
    main()
