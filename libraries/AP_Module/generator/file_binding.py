# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Bounded, stateless local-file reads for SD schedules (Lua io equivalent)."""

from native_templates import render

PATH_SIZE = 256
READ_MAX = 256
POLICY = ('v1:local-files-only,reject-leading-slashes-then-@,no-scheduler-lock,blocking-io,'
          'one-open-seek-read-close-per-call,no-retained-handles,not-atomic-between-calls,'
          'open-failure-is-none,seek-read-close-failure-is-host,past-eof-is-empty,zero-pad-output')


def specification(first_id):
    return dict(id=first_id, name='file.read_at',
                arguments=[dict(type='nonempty-nul-free-bytes-max255-zero-pad256', direction='in'),
                           dict(type='u32-le', direction='in', range=[0, 2147483647]),
                           dict(type='u16-le', direction='in', range=[1, READ_MAX])],
                result=dict(presence='bool-u8', length='u16-le', data=f'bytes-max{READ_MAX}-zero-pad{READ_MAX}'),
                policy=POLICY + ',offset-plus-count-le-INT32_MAX')


def rust(method, notice):
    return notice + render(
        'rust/file.rs',
        path_size=PATH_SIZE,
        read_max=READ_MAX,
        path_size_plus_6=PATH_SIZE + 6,
        path_size_plus_4=PATH_SIZE + 4,
        read_max_plus_3=READ_MAX + 3,
        method_id=method['id'],
    )
