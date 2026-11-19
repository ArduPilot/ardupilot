# AP_FLAKE8_CLEAN
"""Stack-guard options must reach the ChibiOS C and assembly build too."""

import importlib
import re

from pathlib import Path

import pytest


@pytest.mark.parametrize('value', [None, 0, 1])
@pytest.mark.parametrize('hwdef_value', [None, 0, 1])
def test_stack_guard_hwdef(tmp_path, monkeypatch, value, hwdef_value):
    root = Path(__file__).resolve().parents[3]
    for path in ('modules/waf', 'Tools/ardupilotwaf', 'Tools/scripts',
                 'libraries/AP_HAL_ChibiOS/hwdef/scripts'):
        monkeypatch.syspath_prepend(str(root / path))
    chibios = importlib.import_module('chibios')
    config = importlib.import_module('waflib.ConfigSet')
    env = config.ConfigSet()
    env.SRCROOT = str(root)
    env.BUILDROOT = str(tmp_path)
    env.BOARD = 'CubeOrangePlus'
    env.OPTIONS = {}
    if value is not None:
        env.OPTIONS['enable_STACK_GUARD' if value else 'disable_STACK_GUARD'] = True
    if hwdef_value is not None:
        extra = tmp_path / 'extra.hwdef'
        extra.write_text('define HAL_STACK_GUARD_ENABLED %u\n' % hwdef_value)
        env.HWDEF_EXTRA = str(extra)
    chibios.generate_hwdef_h(env)
    header = (tmp_path / 'hwdef.h').read_text()
    defines = re.findall(r'^#define HAL_STACK_GUARD_ENABLED ([01])$', header, re.M)
    expected = value if value is not None else hwdef_value
    if expected is None:
        assert defines == []
    else:
        assert int(defines[-1]) == expected
