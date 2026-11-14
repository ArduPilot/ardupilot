# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Audited native CPU profiles shared by builders and container tools."""

from dataclasses import dataclass


@dataclass(frozen=True)
class CpuProfile:
    id: int
    target: str
    cpu: str
    machine: int
    fpu: str = ''

    @property
    def default_thread_priority(self):
        return 2 if self.machine == 40 else 1


# IDs are container requirements, not an ordered CPU capability level.
CPU_PROFILES = {
    1: CpuProfile(1, 'thumbv7em-none-eabihf', 'cortex-m4', 40, 'fpv4-sp-d16'),
    2: CpuProfile(2, 'x86_64-unknown-linux-gnu', 'x86-64', 62),
    3: CpuProfile(3, 'thumbv7em-none-eabihf', 'cortex-m7', 40, 'fpv5-d16'),
}
TARGETS = {
    'thumbv7em-none-eabihf': (1, 'cortex-m4'),
    'x86_64-unknown-linux-gnu': (2, 'x86-64'),
}


def select(target, cpu=None):
    if target not in TARGETS:
        raise ValueError('unsupported module target: ' + target)
    cpu = TARGETS[target][1] if cpu is None else cpu
    for profile in CPU_PROFILES.values():
        if profile.target == target and profile.cpu == cpu:
            return profile
    raise ValueError(f'CPU {cpu} is incompatible with module target {target}')
