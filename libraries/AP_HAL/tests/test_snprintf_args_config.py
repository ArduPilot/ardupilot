# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Check the typed formatter's feature gate and unsupported-ABI diagnostic."""

import shutil
import subprocess

from pathlib import Path

import pytest

SOURCE = Path(__file__).resolve().parents[1] / 'utility/snprintf_args.cpp'
ARCHITECTURES = [
    (['__arm__', '__ARM_EABI__'], True),
    (['__x86_64__'], True),
    (['__x86_64__', '__CYGWIN__'], True),
    (['__APPLE__', '__aarch64__'], True),
    (['__aarch64__'], True),
    (['__aarch64__', '__AARCH64EB__'], True),
    (['__XTENSA__'], False),
    (['__hexagon__'], False),
    (['__riscv'], False),
    (['__x86_64__', '_WIN32'], False),
    (['__aarch64__', '_WIN32'], False),
]


@pytest.mark.parametrize('architecture,supported', ARCHITECTURES)
@pytest.mark.parametrize('enabled', [False, True])
def test_formatter_configuration(tmp_path, architecture, supported, enabled):
    compiler = shutil.which('c++')
    if compiler is None:
        pytest.skip('requires a C++ preprocessor')
    # Only preprocess here: real HAL formatting is covered by test_vsnprintf.
    # Isolate the source's guards from the host's headers and architecture.
    headers = tmp_path / 'AP_HAL'
    headers.mkdir()
    (headers / 'AP_HAL.h').write_text('#define CONFIG_HAL_BOARD 1\n#define HAL_BOARD_CHIBIOS 2\n')
    (headers / 'Util.h').write_text('' if enabled else '#error Module-only header included\n')
    (tmp_path / 'string.h').write_text('' if enabled else '#error Module-only header included\n')
    result = subprocess.run([compiler, '-E', '-P', '-undef', '-nostdinc', '-I', str(tmp_path),
                             f'-DAP_MODULE_ENABLED={int(enabled)}',
                             *(f'-D{macro}=1' for macro in architecture), str(SOURCE)],
                            capture_output=True, text=True)
    if enabled and not supported:
        assert result.returncode != 0
        assert 'Unsupported architecture for module support' in result.stderr
    else:
        assert result.returncode == 0, result.stderr
        assert ('AP_HAL::Util::snprintf_args' in result.stdout) == enabled
        if not enabled:
            assert not result.stdout.strip()
