# AP_FLAKE8_CLEAN

"""Behaviour tests for the built-in AM32-style DShot ESC model.

Compiles Tools/renode/peripherals/common/AP_DShotESC.cs with a small
harness under mono; skipped when mcs or mono is not installed.
"""

import shutil
import subprocess

from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
MODEL = ROOT / 'Tools' / 'renode' / 'peripherals' / 'common' / 'AP_DShotESC.cs'
HARNESS = Path(__file__).resolve().parent / 'dshot_esc_harness.cs'
SCENARIOS = ['arming', 'throttle', 'edt', 'commands', 'signal-loss', 'settings',
             'bidirectional-latch', '3d']

pytestmark = pytest.mark.skipif(
    shutil.which('mcs') is None or shutil.which('mono') is None,
    reason='needs mcs and mono')


@pytest.fixture(scope='module')
def harness(tmp_path_factory):
    exe = tmp_path_factory.mktemp('dshot_esc') / 'harness.exe'
    subprocess.run(['mcs', '-nologo', '-out:%s' % exe, str(MODEL), str(HARNESS)],
                   check=True, capture_output=True, text=True)
    return exe


@pytest.mark.parametrize('scenario', SCENARIOS)
def test_model(harness, scenario):
    result = subprocess.run(['mono', str(harness), scenario], capture_output=True,
                            text=True, timeout=60)
    assert result.returncode == 0 and result.stdout.strip() == 'ok', result.stderr
