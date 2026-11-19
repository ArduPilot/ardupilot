# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Native modules must select the board's actual instruction set."""

import importlib

from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize('board,cortex,dest_cpu,target,cpu', [
    ('ChibiOS', 'cortex-m4', '', 'thumbv7em-none-eabihf', 'cortex-m4'),
    ('ChibiOS', 'cortex-m7', '', 'thumbv7em-none-eabihf', 'cortex-m7'),
    ('ChibiOS', 'cortex-m3', '', None, None),
    ('ChibiOS', 'cortex-m0', '', None, None),
    ('SITL', '', 'x86_64', 'x86_64-unknown-linux-gnu', None),
    ('LINUX', '', 'amd64', 'x86_64-unknown-linux-gnu', None),
    ('LINUX', '', 'aarch64', 'aarch64-linux-android', None),
    ('SITL', '', 'arm64', 'aarch64-linux-android', None),
    ('LINUX', '', 'armv7l', None, None),
])
def test_module_target_matches_board(monkeypatch, board, cortex, dest_cpu, target, cpu):
    root = Path(__file__).resolve().parents[3]
    for directory in ('modules/waf', 'Tools/ardupilotwaf'):
        monkeypatch.syspath_prepend(str(root / directory))
    ap_module = importlib.import_module('ap_module')
    env = SimpleNamespace(BOARD_CLASS=board, CORTEX=cortex, DEST_CPU=dest_cpu,
                          APJ_BOARD_TYPE='STM32F405xx' if cortex == 'cortex-m4' else 'STM32H743xx')
    assert ap_module.module_target(env) == (target, cpu)


def test_configure_rejects_unsupported_board_before_recursing(monkeypatch):
    root = Path(__file__).resolve().parents[3]
    for directory in ('modules/waf', 'Tools/ardupilotwaf'):
        monkeypatch.syspath_prepend(str(root / directory))
    ap_module = importlib.import_module('ap_module')

    def fatal(message):
        raise ValueError(message)

    cfg = SimpleNamespace(options=SimpleNamespace(module=['applets/example'], disable_modules=False),
                          env=SimpleNamespace(BOARD_CLASS='ChibiOS', CORTEX='cortex-m3',
                                              DEST_CPU='', BOARD='unsupported'), fatal=fatal)
    with pytest.raises(ValueError, match='native modules are not supported on board unsupported'):
        ap_module.configure(cfg)


@pytest.mark.parametrize('mcu,cortex,supported', [
    ('STM32F405xx', 'cortex-m4', True),
    ('STM32H743xx', 'cortex-m7', True),
    ('STM32H757xx', 'cortex-m7', True),
    ('STM32F411xx', 'cortex-m4', False),
    ('STM32F765xx', 'cortex-m7', False),
    ('STM32H753xx', 'cortex-m7', False),
])
def test_chibios_module_target_requires_verified_mcu(monkeypatch, mcu, cortex, supported):
    root = Path(__file__).resolve().parents[3]
    for directory in ('modules/waf', 'Tools/ardupilotwaf'):
        monkeypatch.syspath_prepend(str(root / directory))
    ap_module = importlib.import_module('ap_module')
    env = SimpleNamespace(BOARD_CLASS='ChibiOS', CORTEX=cortex, APJ_BOARD_TYPE=mcu)
    expected = ('thumbv7em-none-eabihf', cortex) if supported else (None, None)
    assert ap_module.module_target(env) == expected
