# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Rust CI bootstrap uses Python downloading and an isolated native installer."""

import io

from pathlib import Path

import pytest

from rust import install_toolchain


@pytest.mark.parametrize('system,machine,expected', [
    ('Linux', 'x86_64', 'x86_64-unknown-linux-gnu'),
    ('Linux', 'aarch64', 'aarch64-unknown-linux-gnu'),
    ('Darwin', 'arm64', 'aarch64-apple-darwin'),
])
def test_host_triple(monkeypatch, system, machine, expected):
    monkeypatch.setattr(install_toolchain.platform, 'system', lambda: system)
    monkeypatch.setattr(install_toolchain.platform, 'machine', lambda: machine)
    assert install_toolchain.host_triple() == expected


def test_unsupported_host(monkeypatch):
    monkeypatch.setattr(install_toolchain.platform, 'system', lambda: 'Other')
    with pytest.raises(ValueError, match='supported'):
        install_toolchain.host_triple()


def test_native_bootstrap(tmp_path, monkeypatch):
    monkeypatch.setattr(install_toolchain, 'host_triple', lambda: 'x86_64-unknown-linux-gnu')
    requested = []

    def download(url, timeout):
        requested.append((url, timeout))
        return io.BytesIO(b'native installer')

    def execute(command, env, check):
        assert check
        assert Path(command[0]).read_bytes() == b'native installer'
        assert Path(command[0]).stat().st_mode & 0o100
        assert command[1:5] == ['-y', '--no-modify-path', '--profile', 'minimal']
        version = Path(install_toolchain.__file__).with_name('rust-toolchain-version').read_text().strip()
        assert command[5:] == ['--default-toolchain', version]
        assert env['CARGO_HOME'] == str(tmp_path / 'cargo')
        assert env['RUSTUP_HOME'] == str(tmp_path / 'rustup')

    def compiler(command, env, text):
        assert command[0] == str(tmp_path / 'cargo/bin/rustup')
        assert command[1:3] == ['which', '--toolchain']
        assert env['RUSTUP_HOME'] == str(tmp_path / 'rustup')
        assert text
        return '/installed/bin/rustc\n'

    monkeypatch.setattr(install_toolchain.urllib.request, 'urlopen', download)
    monkeypatch.setattr(install_toolchain.subprocess, 'run', execute)
    monkeypatch.setattr(install_toolchain.subprocess, 'check_output', compiler)
    assert install_toolchain.install(tmp_path) == '/installed/bin/rustc'
    assert requested == [('https://static.rust-lang.org/rustup/dist/x86_64-unknown-linux-gnu/rustup-init', 60)]
    assert not list(tmp_path.glob('bootstrap-*'))


def test_main_exports_compiler_for_later_ci_steps(tmp_path, monkeypatch, capsys):
    environment = tmp_path / 'github-env'
    monkeypatch.setenv('GITHUB_ENV', str(environment))
    monkeypatch.setattr('sys.argv', ['install_toolchain.py', '--directory', str(tmp_path)])
    monkeypatch.setattr(install_toolchain, 'install', lambda directory: '/installed/bin/rustc')
    install_toolchain.main()
    assert environment.read_text() == 'RUSTC=/installed/bin/rustc\n'
    assert capsys.readouterr().out == '/installed/bin/rustc\n'
