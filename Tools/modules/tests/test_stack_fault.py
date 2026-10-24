# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Register-level recovery policy, independent of emulator exception fidelity."""

import ctypes
import subprocess

from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]


class Fault(ctypes.Structure):
    _fields_ = [(name, ctypes.c_uint32) for name in (
        'exc_return', 'psp', 'primask', 'basepri', 'faultmask', 'icsr', 'cfsr',
        'hfsr', 'mmfar', 'fpccr', 'fpcar', 'base', 'top')]


@pytest.fixture(scope='module')
def matches(tmp_path_factory):
    library = tmp_path_factory.mktemp('stack-fault') / 'policy.so'
    source = '''#include "stack_guard_fault.h"
bool check(const struct AP_StackFault *f) { return ap_stack_fault_matches(f, 1024); }
'''
    subprocess.run(['cc', '-shared', '-fPIC', '-O2', '-Wall', '-Werror', '-x', 'c', '-',
                    '-I', str(ROOT / 'libraries/AP_HAL_ChibiOS/hwdef/common'), '-o', str(library)],
                   input=source, text=True, check=True)
    check = ctypes.CDLL(str(library)).check
    check.argtypes = [ctypes.POINTER(Fault)]
    check.restype = ctypes.c_bool

    def call(**overrides):
        values = dict(exc_return=0xFFFFFFFD, psp=0x20000500, icsr=1 << 11,
                      cfsr=0x82, mmfar=0x200003FC, base=0x20000000, top=0x20004000)
        values.update(overrides)
        return check(ctypes.byref(Fault(**values)))

    return call


def test_valid_frames(matches):
    assert matches()
    assert matches(hfsr=1 << 30)  # escalated MemManage
    assert matches(exc_return=0xFFFFFFED)
    assert matches(exc_return=0xFFFFFFED, fpccr=0xC0000001, fpcar=0x20000380)


@pytest.mark.parametrize('status', [0x10, 0x12, 0x92])
def test_incomplete_stacking(matches, status):
    assert matches(cfsr=status, psp=0x20000380)
    if not status & 0x80:
        assert not matches(cfsr=status, psp=0x20000400)


@pytest.mark.parametrize('field,value', [
    ('exc_return', 0xFFFFFFF1), ('exc_return', 0xFFFFFFF9), ('exc_return', 0),
    ('primask', 1), ('basepri', 0x10), ('faultmask', 1), ('icsr', 0),
    ('cfsr', 0), ('cfsr', 0x80), ('cfsr', 0x2),
    ('psp', 0x1FFFFFFC), ('psp', 0x20000504),
    ('mmfar', 0x1FFFFFFC), ('mmfar', 0x20000400),
    ('fpccr', 1),  # lazy state with basic EXC_RETURN
])
def test_refuse_unattributable_or_critical(matches, field, value):
    assert not matches(**{field: value})


@pytest.mark.parametrize('bit', [b for b in range(32) if b not in (1, 4, 7)])
def test_refuse_every_other_fault_bit(matches, bit):
    assert not matches(cfsr=0x82 | (1 << bit))


@pytest.mark.parametrize('bit', [b for b in range(32) if b != 30])
def test_refuse_other_hardfault_bits(matches, bit):
    assert not matches(hfsr=1 << bit)


@pytest.mark.parametrize('address', [0, 0x1FFFFFFF, 0x20004000, 0xFFFFFFFF])
def test_refuse_foreign_lazy_state(matches, address):
    assert not matches(exc_return=0xFFFFFFED, fpccr=1, fpcar=address)
