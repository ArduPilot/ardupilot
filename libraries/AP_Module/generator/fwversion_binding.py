# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Audited read-only firmware-version fields used by scripting applets."""


MAX_STRING = 64
OWNER = 'AP::fwversion()'
FIELDS = {
    'fw_hash_str': ('hash', 'fwversion_hash', 'fwstring'),
    'fw_short_string': ('string', 'fwversion_string', 'fwstring'),
    'major': ('major', 'fwversion_major', 'uint8_t'),
    'minor': ('minor', 'fwversion_minor', 'uint8_t'),
    'patch': ('patch', 'fwversion_patch', 'uint8_t'),
    'vehicle_type': ('type', 'fwversion_type', 'uint8_t'),
}
POLICY = ('v1:read-static-field,copy-under-call,no-lock,'
          f'utf8-length-u8-bytes-max{MAX_STRING}-zero-pad{MAX_STRING},reject-null-or-overlength')


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    if owner is None:
        raise ValueError('missing firmware-version owner')
    if (owner['rename'] != 'FWVersion' or owner['depends'] is not None or owner['get'] is not None or
            not owner['literal'] or not owner['reference'] or owner['methods'] or owner['aliases'] or
            any(owner[key] for key in ('semaphore', 'scheduler_semaphore', 'semaphore_pointer'))):
        raise ValueError('unaudited firmware-version owner')
    fields = {field['name']: field for field in owner['fields']}
    if not set(FIELDS).issubset(fields):
        raise ValueError('missing firmware-version field')
    methods = []
    established = [item for item in FIELDS.items() if item[0] != 'patch']
    ordered_fields = sorted(established, key=lambda item: item[1][0]) + [('patch', FIELDS['patch'])]
    for name, (public, rust_name, kind) in ordered_fields:
        field = fields[name]
        expected_kind = 'string' if kind == 'fwstring' else kind
        if ((field['rename'] or name) != public or field['depends'] is not None or field['array_len'] is not None or
                field['valid_mask_name'] is not None or field['valid_mask_value'] is not None or
                not field['read'] or field['write'] or
                field['type'] != dict(kind=expected_kind, name=None, nullable=False, reference=False,
                                      pointer=False, skip_check=False, range=None)):
            raise ValueError('unaudited firmware-version field: ' + name)
        methods.append(dict(owner=OWNER, method=name, field=name, fwversion=True, rust_name=rust_name,
                            api_name='FWVersion.' + public, api_aliases=[], aliases=[],
                            owner_depends=None, depends=None, lock=None,
                            literal=True, reference=True,
                            result=dict(type=kind, output=False), arguments=[],
                            effects=('Copies a static firmware-version field; strings are bounded UTF-8 values and '
                                     'are rejected rather than truncated.'),
                            id=first_id + len(methods)))
    reports = [dict(owner=OWNER, name=field['rename'] or field['name'], line=field['line'],
                    kind='singleton-field', status='generated' if field['name'] in FIELDS else 'unsupported',
                    **({'api': 'FWVersion.' + FIELDS[field['name']][0]} if field['name'] in FIELDS else
                       {'reason': 'unsupported-field'}))
               for field in owner['fields']]
    return methods, reports


def wire_type():
    return dict(name='FirmwareString', length='u8', data=f'utf8-bytes-max{MAX_STRING}-zero-pad{MAX_STRING}')
