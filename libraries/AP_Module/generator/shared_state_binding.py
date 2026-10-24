# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Context-owned references to bounded, versioned shared memory blocks."""

from native_templates import render

TRANSFER_MAX = 2048
POLICY = ('v1:firmware-local-shared-id-u32,size-exact-1-to-16384,zero-initialized-on-first-open,'
          '8-blocks-total65536-bytes,32-handles-global-4-per-context,nonreused-context-owned-tokens,'
          'mutex-held-over-copy-and-revision,offset-length-checked,transfer-max2048-zero-padding,'
          'revision-u64-start1,no-revision-wrap,write-expected0-unconditional-otherwise-cas,'
          'stale-write-returns0-without-copy,last-reference-frees,post-join-cleanup,no-retained-module-pointers')


def argument(name, kind):
    return dict(name=name, type=kind)


def specification(first_id):
    token = dict(wire='u32-le', handle='SharedState')
    methods = [
        dict(name='shared_state.open', arguments=[argument('id', 'u32-le'), argument('size', 'u32-le')], result=token),
        dict(name='shared_state.read',
             arguments=[argument('block', token), argument('offset', 'u32-le'), argument('length', 'u16-le')],
             result=dict(name='SharedStateRead', fields=[['revision', 'u64-le'], ['data', f'bytes{TRANSFER_MAX}']])),
        dict(name='shared_state.write',
             arguments=[argument('block', token), argument('offset', 'u32-le'), argument('length', 'u16-le'),
                        argument('expected_revision', 'u64-le'), argument('data', f'bytes{TRANSFER_MAX}')],
             result='u64-le'),
        dict(name='shared_state.close', arguments=[argument('block', token)], result='void'),
    ]
    for offset, method in enumerate(methods):
        method.update(id=first_id + offset, policy=POLICY)
    return dict(methods=methods)


def rust(spec, notice):
    return notice + render('rust/shared_state.rs', **{
        method['name'].split('.')[1] + '_id': method['id'] for method in spec['methods']})
