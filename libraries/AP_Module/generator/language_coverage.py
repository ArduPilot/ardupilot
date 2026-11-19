# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Distinguish host calls, local helpers, primitives and replacement-only APIs."""

import re

from collections import Counter
from pathlib import Path

import contracts
import cpp_bindings
import naming

# These adapters implement client-side behaviour, not another firmware call.
# The C++ SDK exposes their building blocks but not the named convenience API.
LOCAL = {
    ('AP_Networking', 'address_to_str'): ('Ipv4Address', ('string.format_printf',)),
    ('AP_RangeFinder_Backend', 'distance'): ('RangeFinderBackend.distance', ('RangeFinderBackend.get_state',)),
    ('AP_RangeFinder_Backend', 'signal_quality_pct'): ('RangeFinderBackend.signal_quality', ('RangeFinderBackend.get_state',)),
    ('AP_Filesystem::stat_t', 'is_directory'): ('FileStat.is_directory', ('filesystem.stat',)),
    ('AP_HAL::CANFrame', 'id_signed'): ('CanFrame.id_signed', ('CanBuffer.read_frame',)),
    ('AP_HAL::CANFrame', 'isErrorFrame'): ('CanFrame.is_error_frame', ('CanBuffer.read_frame',)),
    ('AP_HAL::CANFrame', 'isExtended'): ('CanFrame.is_extended', ('CanBuffer.read_frame',)),
    ('AP_HAL::CANFrame', 'isRemoteTransmissionRequest'): (
        'CanFrame.is_remote_transmission_request', ('CanBuffer.read_frame',)),
    ('AP_Scripting_SerialAccess', 'read'): ('SerialPort.read_byte', ('SerialPort.read',)),
    ('AP_Scripting_SerialAccess', 'write'): ('SerialPort.write_byte', ('SerialPort.write',)),
    ('Location', 'get_vector_from_origin_NEU'): (
        'Location.get_vector_from_origin_neu', ('Location.get_vector_from_origin_NEU_cm',)),
}

REMOVED = {
    ('AP_AHRS', 'get_roll'): ('ahrs.get_roll_rad',),
    ('AP_AHRS', 'get_pitch'): ('ahrs.get_pitch_rad',),
    ('AP_AHRS', 'get_yaw'): ('ahrs.get_yaw_rad',),
    ('AP_AHRS', 'wind_estimate'): ('ahrs.get_wind',),
    ('RangeFinder', 'distance_cm_orient'): ('rangefinder.distance_orient',),
    ('RangeFinder', 'ground_clearance_cm_orient'): ('rangefinder.ground_clearance_orient',),
    ('RangeFinder', 'max_distance_cm_orient'): ('rangefinder.max_distance_orient',),
    ('RangeFinder', 'min_distance_cm_orient'): ('rangefinder.min_distance_orient',),
}


HELPER_FILES = {
    'Ipv4Address': 'sdk/rust/src/network.rs',
    'RangeFinderBackend': 'sdk/rust/src/rangefinder_proximity.rs',
    'CanFrame': 'sdk/rust/src/can.rs',
    'SerialPort': 'sdk/rust/src/serial.rs',
    'Location': 'sdk/rust/src/compatibility.rs',
}


def helper_present(api, sources):
    owner, _, method = api.partition('.')
    source = sources[HELPER_FILES[owner]]
    if owner == 'Ipv4Address':
        return 'impl core::fmt::Display for Ipv4Address {' in source
    # Generated top-level impls and their closing braces are unindented. Scope
    # checks to that owner block, not another type with the same method name.
    blocks = re.findall(r'^impl ' + re.escape(owner) + r' \{\n(.*?)^\}', source, re.M | re.S)
    function = naming.rust_identifier(method)
    return any(re.search(r'pub (?:const )?fn ' + re.escape(function) + r'\(', block) for block in blocks)


def report(spec, outputs):
    catalog = {e['contract']['name'] for e in contracts.describe(spec)}
    _, _, plans = cpp_bindings.describe(spec)
    cpp_calls = {plan[0] for plan in plans}
    if cpp_calls != catalog:
        raise ValueError('C++ coverage and contract catalogue disagree')
    expected_ada = {'ap_module_ada_binding_' + naming.binding_name(name) for name in catalog}
    ada_bridge = set(re.findall(r'extern "C" int32_t (ap_module_ada_binding_\w+)\(', outputs['sdk/ada/bridge.cpp']))
    ada_source = outputs['sdk/ada/ap_module-bindings.adb']
    ada_imports = set(re.findall(r'External_Name => "(ap_module_ada_binding_\w+)"', ada_source))
    if ada_bridge != expected_ada or ada_imports != expected_ada:
        raise ValueError('Ada coverage and generated clients disagree')
    expected_f90 = {'ap_module_f90_binding_' + naming.binding_name(name) for name in catalog}
    f90_bridge = set(re.findall(r'extern "C" int32_t (ap_module_f90_binding_\w+)\(', outputs['sdk/fortran/bridge.cpp']))
    f90_imports = set(re.findall(r'bind\(C, name="(ap_module_f90_binding_\w+)"\)',
                                 outputs['sdk/fortran/ap_module_bindings.f90']))
    if f90_bridge != expected_f90 or f90_imports != expected_f90:
        raise ValueError('Fortran coverage and generated clients disagree')
    lookup = {(m['owner'], m['method']): (m['api_name'],) for m in spec['methods']}
    for method in spec['methods']:
        lookup.update({(method['owner'], alias.rsplit('.', 1)[1]): (alias,) for alias in method['api_aliases']})
    for owner in spec['math']['types']:
        lookup.update({(owner['name'], m['name']): (owner['name'] + '.' + m['name'],) for m in owner['methods']})
    lookup.update({('Parameter', m['name']): ('Parameter.' + m['name'],) for m in spec['parameter']['methods']})
    rust_sources = {name: value for name, value in outputs.items() if name.endswith('.rs')}
    rust_sources['sdk/rust/src/compatibility.rs'] = (
        Path(__file__).resolve().parents[1] / 'sdk/rust/src/compatibility.rs').read_text()
    rows = []
    for entry in spec['coverage']['entries']:
        key = entry['owner'], entry['name']
        row = {k: entry[k] for k in ('owner', 'name', 'kind', 'line') if k in entry}
        required = LOCAL[key][1] if key in LOCAL else REMOVED.get(key, ())
        if required and not set(required) <= catalog:
            row['languages'] = {language: dict(status='unsupported', reason='Required host API not selected.')
                                for language in ('rust', 'cpp', 'ada', 'fortran')}
            rows.append(row)
            continue
        if key == ('AP_Filesystem::stat_t', 'is_directory'):
            if (not re.search(r'struct FileStat \{[^}]*pub is_directory: bool', rust_sources['sdk/rust/src/filesystem.rs']) or
                    not re.search(r'struct filesystem_statOutput \{[^}]*bool directory;', outputs['sdk/cpp/bindings.hpp'])):
                raise ValueError('missing filesystem directory value view')
            languages = dict(rust=dict(status='value-field', api='FileStat.is_directory'),
                             cpp=dict(status='value-field', api='filesystem_statOutput.directory'))
        elif key in LOCAL:
            api, calls = LOCAL[key]
            if not set(calls) <= catalog:
                raise ValueError('stale local API coverage: ' + api)
            if not helper_present(api, rust_sources):
                raise ValueError('local Rust helper missing from generated source: ' + api)
            languages = dict(rust=dict(status='local-helper', api=api),
                             cpp=dict(status='replacement-only', calls=list(calls)))
        elif key in REMOVED:
            calls = REMOVED[key]
            if not set(calls) <= catalog:
                raise ValueError('stale removed API replacement: ' + '.'.join(key))
            languages = {language: dict(status='replacement-only', calls=list(calls),
                                        note='Deprecated wrapper is absent; replacement may use different units.')
                         for language in ('rust', 'cpp')}
        elif entry.get('reason') == 'native-rust-integer-operation':
            languages = {language: dict(status='native-operation', note='Use language integer/formatting operations.')
                         for language in ('rust', 'cpp')}
        else:
            api = entry.get('api')
            calls = (api,) if isinstance(api, str) else tuple(api or ())
            if not calls or not set(calls) <= catalog:
                calls = lookup.get(key, lookup.get((entry['owner'], entry.get('target')), ()))
            if key == ('AP_Logger', 'write'):
                calls = (spec['log']['name'],)
            if calls and set(calls) <= catalog:
                languages = {language: dict(status='host-call', calls=list(calls)) for language in ('rust', 'cpp')}
            elif entry['status'] == 'unsupported':
                languages = {language: dict(status='unsupported', reason=entry['reason']) for language in ('rust', 'cpp')}
            else:
                raise ValueError('unclassified language coverage: ' + '.'.join(key))
        languages['ada'] = dict(languages['cpp'])
        if entry.get('reason') == 'native-rust-integer-operation':
            if entry['name'] == '__tostring':
                languages['ada'] = (dict(status='replacement-only', calls=['string.format_printf'],
                                         note="Ada 'Image may require runtime/secondary-stack support.")
                                    if 'string.format_printf' in catalog else
                                    dict(status='unsupported', reason='Firmware formatter not selected.'))
            else:
                languages['ada']['note'] = 'Use Interfaces modular integer arithmetic, shifts and numeric conversions.'
        if languages['ada']['status'] == 'value-field':
            languages['ada']['api'] = 'AP_Module.Types.' + languages['ada']['api']
        languages['fortran'] = dict(languages['cpp'])
        if entry.get('reason') == 'native-rust-integer-operation':
            if entry['name'] == '__tostring':
                languages['fortran'] = (dict(status='replacement-only', calls=['string.format_printf'],
                                             note='Fortran formatted I/O requires the excluded libgfortran runtime.')
                                        if 'string.format_printf' in catalog else
                                        dict(status='unsupported', reason='Firmware formatter not selected.'))
            else:
                languages['fortran']['note'] = ('Use bit intrinsics and widened conversions: unsigned values are '
                                                'same-width signed integer bit patterns, not signed arithmetic.')
        if languages['fortran']['status'] == 'value-field':
            languages['fortran']['api'] = 'ap_module_bindings.filesystem_statoutput%directory'
        row['languages'] = languages
        rows.append(row)
    return dict(
        schema=2, scope=spec['coverage']['scope'], lifecycle=spec['coverage']['lifecycle'],
        availability='Catalogue support; feature guards and runtime availability still apply.',
        definitions={
            'host-call': 'A generated firmware-call API, possibly renamed or split into typed operations.',
            'local-helper': 'A client-side SDK helper implementing the Lua operation.',
            'value-field': 'The result is exposed as a client value field rather than a method.',
            'native-operation': 'Language primitive operations, not named generated SDK methods.',
            'replacement-only': 'Building blocks exist, but the named convenience/deprecated API does not.',
            'unsupported': 'No audited implementation or replacement.'},
        native_host_calls=dict(rust=len(catalog), cpp=len(cpp_calls), ada=len(ada_bridge), fortran=len(f90_bridge)),
        summary={language: dict(sorted(Counter(row['languages'][language]['status'] for row in rows).items()))
                 for language in ('rust', 'cpp', 'ada', 'fortran')}, entries=rows,
    )
