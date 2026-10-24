# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Bounded filesystem metadata, read and directory traversal APIs."""

from native_templates import render

OWNER = 'AP_Filesystem'
STAT = 'AP_Filesystem::stat_t'
PATH_SIZE = 256
READ_MAX = 1024
WRITE_MAX = 1024
NAME_MAX = 255
POLICY = ('v1:stateless-open-per-call,path-max255,virtual-and-local-backends,blocking-io,'
          'no-retained-descriptors,read-max1024,append-max1024,dir-index-u16,'
          'all-descriptors-closed-on-all-paths,fixed-zero-padded-wire')


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    if owner is None or owner['rename'] != 'fs' or owner['depends'] is not None:
        raise ValueError('unaudited AP_Filesystem metadata')
    stat = next((item for item in owner['methods'] if item['name'] == 'stat'), None)
    if stat is None or stat['return']['kind'] != 'boolean' or len(stat['arguments']) != 2:
        raise ValueError('unaudited AP_Filesystem.stat')
    if (stat['arguments'][0]['kind'], stat['arguments'][1]['kind'], stat['arguments'][1]['name'],
            stat['arguments'][1]['nullable']) != ('string', 'userdata', STAT, True):
        raise ValueError('unaudited AP_Filesystem.stat arguments')
    format_method = next((item for item in owner['methods'] if item['name'] == 'format'), None)
    if (format_method is None or format_method['return']['kind'] != 'boolean' or
            format_method['arguments'] or format_method['depends'] != 'AP_FILESYSTEM_FORMAT_ENABLED'):
        raise ValueError('unaudited AP_Filesystem.format')
    value = next((item for item in ir['userdata'] if item['name'] == STAT), None)
    if value is None:
        raise ValueError('missing AP_Filesystem::stat_t')
    fields = {(field['name'], field['type']['kind'], field['read'], field['write'])
              for field in value['fields']}
    expected = {('size', 'uint32_t', True, False), ('mtime', 'uint32_t', True, False),
                ('mode', 'int32_t', True, False), ('atime', 'uint32_t', True, False),
                ('ctime', 'uint32_t', True, False)}
    if fields != expected:
        raise ValueError('unaudited AP_Filesystem::stat_t fields')
    return specification(first_id)


def specification(first_id):
    path = dict(type='nonempty-nul-free-bytes-max255-zero-pad256', name='path')
    methods = [
        dict(id=first_id, name='filesystem.stat', arguments=[path],
             result=dict(presence='bool-u8', size='u32-le', mtime='u32-le', directory='bool-u8'),
             policy=POLICY),
        dict(id=first_id + 1, name='filesystem.read_at',
             arguments=[path, dict(type='u32-le', name='offset'),
                        dict(type='u16-le', range=[1, READ_MAX], name='length')],
             result=dict(presence='bool-u8', length='u16-le',
                         data=f'bytes-max{READ_MAX}-zero-pad{READ_MAX}'), policy=POLICY),
        dict(id=first_id + 2, name='filesystem.dir_entry',
             arguments=[path, dict(type='u16-le', name='index')],
             result=dict(presence='bool-u8', length='u16-le',
                         name=f'utf8-max{NAME_MAX}-zero-pad{NAME_MAX}'), policy=POLICY),
        dict(id=first_id + 3, name='filesystem.truncate', arguments=[path], result='bool-u8', policy=POLICY),
        dict(id=first_id + 4, name='filesystem.append',
             arguments=[path, dict(type=f'bytes-max{WRITE_MAX}-zero-pad{WRITE_MAX}', name='data')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 5, name='filesystem.unlink', arguments=[path], result='bool-u8', policy=POLICY),
        dict(id=first_id + 6, name='filesystem.format', arguments=[], result='bool-u8',
             policy=POLICY + ',requires-disarmed,asynchronous-backend-status-not-retained'),
    ]
    return dict(methods=methods, read_max=READ_MAX, write_max=WRITE_MAX, name_max=NAME_MAX)


def rust(spec, notice):
    stat, read, entry, truncate, append, unlink, format_ = spec['methods']
    return notice + render(
        'rust/filesystem.rs',
        read_max=READ_MAX,
        write_max=WRITE_MAX,
        name_max=NAME_MAX,
        path_size=PATH_SIZE,
        stat_id=stat['id'],
        path_size_plus_6=PATH_SIZE + 6,
        path_size_plus_4=PATH_SIZE + 4,
        read_max_plus_3=READ_MAX + 3,
        read_id=read['id'],
        path_size_plus_2=PATH_SIZE + 2,
        name_max_plus_3=NAME_MAX + 3,
        entry_id=entry['id'],
        truncate_id=truncate['id'],
        path_size_plus_2_plus_write_max=PATH_SIZE + 2 + WRITE_MAX,
        append_id=append['id'],
        unlink_id=unlink['id'],
        format_id=format_['id'],
    )
