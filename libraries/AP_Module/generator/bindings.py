# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Generate supported binding categories from the existing Lua generator's IR."""

import json
import re
import subprocess
import sys
import tempfile

from collections import Counter
from pathlib import Path

import airspeed_binding
import analog_pwm_binding
import applet_binding
import arming_binding
import battery_binding
import borrowed_binding
import camera_binding
import can_binding
import command_int_binding
import contracts
import crsf_binding
import custom_mode_binding
import dronecan_binding
import efi_binding
import example_constants_binding
import examples_binding
import file_binding
import filesystem_binding
import fwversion_binding
import i2c_binding
import libm_binding
import log_binding
import math_binding
import mavlink_binding
import motor_config_binding
import naming
import network_binding
import old_parameter_binding
import onvif_binding
import parameter_binding
import periph_binding
import printf_binding
import rangefinder_proximity_binding
import serial_binding
import serialled_binding
import shared_state_binding
import singleton_binding
import strtof_binding
import telemetry_binding
import value_binding

from native_templates import render_lines

ROOT = Path(__file__).resolve().parents[1]
LUA_GENERATOR = ROOT.parent / 'AP_Scripting/generator'
ENCODING = 'AP_Module.bindings.v1:little-endian,IEEE754-f32,bool-u8,optional-presence-u8,exact-lengths'
# Hand-written adapters use the same public namespace proxies as bindings.desc.
# Keep this list explicit so adding an adapter cannot silently add SDK API.
MANUAL_NAMESPACES = (
    'airspeed', 'analog', 'arming', 'battery', 'camera', 'can', 'crsf', 'dronecan', 'efi',
    'esc_telem', 'file', 'fs', 'gcs', 'gps', 'i2c', 'libm', 'logger', 'mavlink', 'mission',
    'motors', 'motors_6dof', 'motors_dynamic', 'motors_matrix', 'network', 'networking', 'onvif',
    'param', 'periph', 'proximity', 'pwm_source', 'rangefinder', 'scripting', 'serial', 'serial_led',
    'servo_telem', 'srv_channels', 'string', 'vehicle',
)
PARAM_ARGUMENTS = {
    'get': [('string', False), ('float', True)],
    'add_table': [('uint8_t', False), ('string', False), ('uint8_t', False)],
    'add_param': [('uint8_t', False), ('uint8_t', False), ('string', False), ('float', False)],
    'set_by_name': [('string', False), ('float', False)],
    'set_and_save_by_name': [('string', False), ('float', False)],
    'set_default_by_name': [('string', False), ('float', False)],
}
PARAM_RANGES = {'add_table': {0: ['0', '200'], 2: ['1', '63']},
                'add_param': {0: ['0', '200'], 1: ['1', '63']}}
PARAM_POLICY = 'v1:nonempty-nul-free-bytes-max16-zero-pad17,finite-inputs,checked-integer-range,persistent-tables'


def read_ir(description=None):
    with tempfile.TemporaryDirectory(prefix='ap-rust-bindings-') as work:
        work = Path(work)
        output = work / 'bindings.json'
        subprocess.run([sys.executable, str(LUA_GENERATOR / 'generator.py'),
                        '-i', str(description or LUA_GENERATOR / 'description/bindings.desc'),
                        '--dump-ir', str(output)], check=True)
        return json.loads(output.read_text())


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z_0-9]*', value):
        raise ValueError(f'invalid identifier: {value}')
    return value


def policies():
    return json.loads((ROOT / 'generator/binding_policies.json').read_text())


def method_contract(owner, method):
    lock_fields = [('semaphore', 'semaphore'), ('scheduler-semaphore', 'scheduler_semaphore'),
                   ('semaphore-pointer', 'semaphore_pointer')]
    locks = [name for name, key in lock_fields if owner[key]]
    if len(locks) > 1:
        raise ValueError('conflicting-singleton-locks')
    lock = locks[0] if locks else None
    if method['deprecate']:
        raise ValueError('deprecated')
    name = identifier(method['name'])
    parameter = owner['name'] == 'AP_Param'
    if parameter and (name not in PARAM_ARGUMENTS or lock is not None or method['return']['kind'] != 'boolean' or
                      [(a['kind'], a['nullable']) for a in method['arguments']] != PARAM_ARGUMENTS[name] or
                      any(a['range'] != PARAM_RANGES.get(name, {}).get(i) for i, a in enumerate(method['arguments']))):
        raise ValueError('unaudited-parameter-adapter')
    result = singleton_binding.field(method['return'])
    args = [singleton_binding.field(a, argument=True, parameter=parameter) for a in method['arguments']]
    outputs = [a for a in args if a['output']]
    if outputs:
        references = [a.get('reference', False) for a in outputs]
        if any(references) and not all(references):
            raise ValueError('mixed-output-directions')
        if (all(references) and result['type'] != 'void') or (not any(references) and result['type'] != 'boolean'):
            raise ValueError('unsupported-output-result')
    public_owner = identifier(owner['rename'] or owner['name'])
    aliases = [a for a in owner['aliases'] if a['name'] == name and not a['manual']]
    if any(a['depends'] for a in aliases):
        raise ValueError('conditional-alias')
    public = public_owner + '.' + identifier(method['rename'] or name)
    item = dict(owner=owner['name'], method=name, rust_name=public.replace('.', '_').lower(), api_name=public,
                api_aliases=sorted(public_owner + '.' + identifier(a['alias']) for a in aliases),
                aliases=sorted((public_owner + '_' + a['alias']).lower() for a in aliases),
                owner_depends=owner['depends'], depends=method['depends'], lock=lock, result=result, arguments=args)
    for key in ('get', 'literal', 'reference'):
        if owner[key]:
            item[key] = owner[key]
    if parameter:
        item['adapter'] = PARAM_POLICY
    return item


def select(ir, selection, only=None):
    """Select by signature. `only` is a fail-closed subset hook for synthetic tests."""
    if ir['schema'] != 1:
        raise ValueError('unsupported Lua binding IR version')
    policy = policies()
    owners = {o['name']: o for o in ir['singletons']}
    if not set(policy['accessors']).issubset(owners):
        raise ValueError('stale accessor policy')
    for name, accessor in policy['accessors'].items():
        if not accessor['reason'] or any(owners[name][key] != accessor[key] for key in ('get', 'literal', 'reference')):
            raise ValueError('changed or unexplained accessor policy: ' + name)
    available = {o['name'] + '.' + m['name'] for o in ir['singletons'] + ir.get('ap_objects', []) for m in o['methods']}
    if not set(policy['excluded']).issubset(available) or not all(policy['excluded'].values()):
        raise ValueError('stale or unexplained binding exclusion')
    if not set(policy['effects']).issubset(available) or not all(policy['effects'].values()):
        raise ValueError('stale or unexplained binding effect')
    includes = {h['name']: h for h in ir['includes']}
    if not set(policy['headers']).issubset(includes):
        raise ValueError('stale header policy')
    if only and not {o + '.' + m for o, c in only.items() for m in c['methods']}.issubset(available):
        raise ValueError('missing requested binding')
    methods, coverage, names = [], [], set()
    for owner in sorted(ir['singletons'], key=lambda o: o['name']):
        owner_reports = {}
        for method in sorted(owner['methods'], key=lambda m: m['name']):
            key = owner['name'] + '.' + method['name']
            if only is not None and method['name'] not in only.get(owner['name'], {}).get('methods', []):
                continue
            report = dict(owner=owner['name'], name=method['name'], line=method['line'], kind='singleton-method')
            try:
                if key in policy['excluded']:
                    raise ValueError(policy['excluded'][key])
                if (owner['get'] or owner['literal'] or owner['reference']) and owner['name'] not in policy['accessors']:
                    raise ValueError('unaudited-accessor')
                item = method_contract(owner, method)
                effects = policy['accessors'].get(owner['name'], {}).get('effects')
                if effects:
                    item['getter_effects'] = effects
                if key in policy['effects']:
                    item['effects'] = policy['effects'][key]
            except ValueError as error:
                if only is not None:
                    raise ValueError(f'bindings.desc:{method["line"]}: {key}: {error}') from error
                report.update(status='unsupported', reason=str(error))
            else:
                for name in [item['rust_name']] + item['aliases']:
                    if name in names:
                        raise ValueError('duplicate Rust binding name: ' + name)
                    names.add(name)
                item['id'] = len(methods) + 1
                if only is not None:
                    header = includes[only[owner['name']]['include']]
                    item.update(include=header['name'], include_depends=header['depends'])
                methods.append(item)
                report.update(status='generated', api=item['api_name'])
            coverage.append(report)
            owner_reports[method['name']] = report
        for alias in sorted(owner['aliases'], key=lambda a: a['alias']):
            if alias['manual']:
                logger = owner['name'] == 'AP_Logger' and alias['alias'] == 'write'
                coverage.append(dict(owner=owner['name'], name=alias['alias'], kind='singleton-manual',
                                     status='generated' if logger else 'unsupported',
                                     reason='manual-logger-adapter' if logger else 'manual-lua'))
            elif alias['name'] in owner_reports:
                target = owner_reports[alias['name']]
                report = dict(owner=owner['name'], name=alias['alias'], kind='singleton-alias',
                              target=alias['name'], status=target['status'])
                if target['status'] != 'generated':
                    report['reason'] = target['reason']
                coverage.append(report)
    fwversion_all, fwversion_reports = (fwversion_binding.select(ir, len(methods) + 1)
                                        if only is None or fwversion_binding.OWNER in only else ([], []))
    fwversion = fwversion_all[:5]
    fwversion_tail = fwversion_all[5:]
    methods += fwversion
    coverage += fwversion_reports
    objects, object_reports = (borrowed_binding.select(ir, len(methods) + 1, method_contract, policy['effects'])
                               if only is None else ([], []))
    methods += objects
    coverage += object_reports
    values, value_reports = (value_binding.select(ir, len(methods) + 1, method_contract)
                             if only is None else ([], []))
    methods += values
    coverage += [r for r in value_reports if r['owner'] in value_binding.TYPES]
    parameter = parameter_binding.select(ir, selection['Parameter'], len(methods) + 1)
    math = math_binding.select(ir, selection, parameter['layout']['id'] + 1)
    libm = libm_binding.specification(max(m['id'] for o in math['types'] for m in o['methods']) + 1)
    file = file_binding.specification(max(m['id'] for m in libm) + 1)
    log = log_binding.specification(file['id'] + 1)
    strtof = strtof_binding.specification(log['id'] + 1)
    battery = battery_binding.specification(strtof['id'] + 1)
    dronecan = (dronecan_binding.select(ir, battery['id'] + 1)
                if only is None or 'DroneCAN_Handle' in only else
                dronecan_binding.specification(battery['id'] + 1))
    # Request support is assigned at the ABI tail with other example adapters.
    dronecan_base_last = dronecan['methods'][5]['id']
    command_int = (command_int_binding.select(ir, dronecan_base_last + 1)
                   if only is None or command_int_binding.OWNER in only else None)
    command_int_last = command_int['id'] if command_int is not None else dronecan_base_last
    serial = (serial_binding.select(ir, command_int_last + 1)
              if only is None or serial_binding.OWNER in only or serial_binding.USERDATA in only else None)
    serial_last = serial['methods'][-1]['id'] if serial is not None else command_int_last
    mavlink = (mavlink_binding.select(ir, serial_last + 1)
               if only is None or mavlink_binding.OWNER in only else None)
    # block_command is assigned at the ABI tail below.
    mavlink_last = mavlink['methods'][3]['id'] if mavlink is not None else serial_last
    arming = (arming_binding.select(ir, mavlink_last + 1)
              if only is None or arming_binding.OWNER in only else None)
    arming_last = arming['methods'][-1]['id'] if arming is not None else mavlink_last
    efi = (efi_binding.select(ir, arming_last + 1)
           if only is None or efi_binding.OWNER in only or efi_binding.BACKEND in only else None)
    # The example-only get_state adapter is assigned at the tail below so adding
    # it does not renumber the established binding ABI.
    efi_last = efi['methods'][1]['id'] if efi is not None else arming_last
    rangefinder_proximity_owners = {
        rangefinder_proximity_binding.RANGEFINDER_OWNER,
        rangefinder_proximity_binding.RANGEFINDER_BACKEND,
        rangefinder_proximity_binding.RANGEFINDER_STATE,
        rangefinder_proximity_binding.PROXIMITY_OWNER,
        rangefinder_proximity_binding.PROXIMITY_BACKEND,
    }
    rangefinder_proximity = (
        rangefinder_proximity_binding.select(ir, efi_last + 1)
        if only is None or rangefinder_proximity_owners.intersection(only) else None)
    # The example-only backend snapshot is assigned at the ABI tail below.
    rangefinder_proximity_last = (rangefinder_proximity['methods'][9]['id']
                                  if rangefinder_proximity is not None else efi_last)
    filesystem = (filesystem_binding.select(ir, rangefinder_proximity_last + 1)
                  if only is None or filesystem_binding.OWNER in only or filesystem_binding.STAT in only else None)
    filesystem_last = filesystem['methods'][5]['id'] if filesystem is not None else rangefinder_proximity_last
    network = (network_binding.select(ir, filesystem_last + 1)
               if only is None or network_binding.OWNER in only or network_binding.NETWORK_OWNER in only else None)
    # Peer-aware receive and pending-connect state are assigned at the ABI tail.
    network_last = network['methods'][12]['id'] if network is not None else filesystem_last
    can_owners = {can_binding.OWNER, can_binding.BUFFER, can_binding.FRAME}
    can = (can_binding.select(ir, network_last + 1)
           if only is None or can_owners.intersection(only) else None)
    can_last = can['methods'][-1]['id'] if can is not None else network_last
    telemetry_owners = {
        telemetry_binding.ESC_OWNER, telemetry_binding.ESC_VALUE,
        telemetry_binding.SERVO_OWNER, telemetry_binding.SERVO_VALUE,
    }
    telemetry = (telemetry_binding.select(ir, can_last + 1)
                 if only is None or telemetry_owners.intersection(only) else None)
    telemetry_last = telemetry['methods'][1]['id'] if telemetry is not None else can_last
    motor_config_owners = {
        motor_config_binding.MATRIX_OWNER, motor_config_binding.DYNAMIC_OWNER,
        motor_config_binding.FACTOR_TABLE, motor_config_binding.MOTORS_OWNER,
        motor_config_binding.SIX_DOF_OWNER,
    }
    motor_config = (motor_config_binding.select(ir, telemetry_last + 1)
                    if only is None or motor_config_owners.intersection(only) else None)
    # Copter 6-DoF configuration is assigned at the ABI tail.
    motor_config_last = motor_config['methods'][6]['id'] if motor_config is not None else telemetry_last
    serialled = (serialled_binding.select(ir, motor_config_last + 1)
                 if only is None or serialled_binding.OWNER in only else None)
    serialled_last = serialled['methods'][-1]['id'] if serialled is not None else motor_config_last
    onvif = (onvif_binding.select(ir, serialled_last + 1)
             if only is None or onvif_binding.OWNER in only else None)
    onvif_last = onvif['id'] if onvif is not None else serialled_last
    crsf_owners = {crsf_binding.OWNER, crsf_binding.MENU, crsf_binding.PARAMETER}
    crsf = (crsf_binding.select(ir, onvif_last + 1)
            if only is None or crsf_owners.intersection(only) else None)
    # Menu metadata and the legacy filtered event call are assigned at the ABI tail.
    crsf_last = crsf['methods'][8]['id'] if crsf is not None else onvif_last
    applet_methods = {
        applet_binding.GPS_OWNER: {'time_epoch_usec', 'inject_data'},
        applet_binding.CAMERA_OWNER: {'set_stream_information'},
        applet_binding.NETWORKING_OWNER: {'add_route'},
        applet_binding.SCRIPTING_OWNER: {'restart_all'},
    }
    applet_requested = only is None or any(
        names.intersection(only.get(owner, {}).get('methods', []))
        for owner, names in applet_methods.items())
    applet = (applet_binding.select(ir, crsf_last + 1)
              if applet_requested else None)
    applet_last = applet['methods'][-1]['id'] if applet is not None else crsf_last
    example_methods = {
        examples_binding.VEHICLE_OWNER: {examples_binding.VELOCITY_ALIAS},
        examples_binding.SRV_OWNER: {examples_binding.SAFETY_ALIAS},
    }
    examples_requested = only is None or any(
        names.intersection(only.get(owner, {}).get('methods', []))
        for owner, names in example_methods.items())
    examples = (examples_binding.select(ir, applet_last + 1) if examples_requested else None)
    # The example globals are assigned at the ABI tail.
    examples_last = examples['methods'][1]['id'] if examples is not None else applet_last
    i2c_owners = {i2c_binding.OWNER, i2c_binding.DEVICE}
    i2c = (i2c_binding.select(ir, examples_last + 1)
           if only is None or i2c_owners.intersection(only) else None)
    i2c_last = i2c['methods'][-1]['id'] if i2c is not None else examples_last
    camera = (camera_binding.select(ir, i2c_last + 1)
              if only is None or camera_binding.OWNER in only else None)
    camera_last = camera['methods'][-1]['id'] if camera is not None else i2c_last
    custom_mode_requested = only is None or 'register_custom_mode' in only.get(
        custom_mode_binding.OWNER, {}).get('methods', [])
    custom_mode = (custom_mode_binding.select(ir, camera_last + 1) if custom_mode_requested else None)
    custom_mode_last = custom_mode['methods'][-1]['id'] if custom_mode is not None else camera_last
    analog_pwm_requested = (only is None or analog_pwm_binding.ANALOG_SOURCE in only or
                            analog_pwm_binding.PWM_SOURCE in only or
                            'channel' in only.get(analog_pwm_binding.ANALOG_OWNER, {}).get('methods', []))
    analog_pwm = (analog_pwm_binding.select(ir, custom_mode_last + 1) if analog_pwm_requested else None)
    tail_last = analog_pwm['methods'][-1]['id'] if analog_pwm is not None else custom_mode_last
    if efi is not None:
        efi['methods'][2]['id'] = tail_last + 1
        tail_last = efi['methods'][2]['id']
    dronecan['methods'][6]['id'] = tail_last + 1
    tail_last = dronecan['methods'][6]['id']
    if mavlink is not None:
        mavlink['methods'][4]['id'] = tail_last + 1
        tail_last = mavlink['methods'][4]['id']
    if rangefinder_proximity is not None:
        rangefinder_proximity['methods'][10]['id'] = tail_last + 1
        tail_last = rangefinder_proximity['methods'][10]['id']
    if network is not None:
        for method in network['methods'][13:15]:
            method['id'] = tail_last + 1
            tail_last = method['id']
    if motor_config is not None:
        for method in motor_config['methods'][7:]:
            method['id'] = tail_last + 1
            tail_last = method['id']
    if examples is not None:
        for method in examples['methods'][2:]:
            method['id'] = tail_last + 1
            tail_last = method['id']
    if network is not None:
        for method in network['methods'][15:]:
            method['id'] = tail_last + 1
            tail_last = method['id']
    if crsf is not None:
        for method in crsf['methods'][9:]:
            method['id'] = tail_last + 1
            tail_last = method['id']
    if rangefinder_proximity is not None:
        for method in rangefinder_proximity['methods'][11:]:
            method['id'] = tail_last + 1
            tail_last = method['id']
    for method in fwversion_tail:
        method['id'] = tail_last + 1
        tail_last = method['id']
        methods.append(method)
    if telemetry is not None:
        for method in telemetry['methods'][2:]:
            method['id'] = tail_last + 1
            tail_last = method['id']
    airspeed_owners = {airspeed_binding.OWNER, airspeed_binding.BACKEND}
    airspeed = (airspeed_binding.select(ir, tail_last + 1)
                if only is None or airspeed_owners.intersection(only) else None)
    if airspeed is not None:
        tail_last = airspeed['methods'][-1]['id']
    if filesystem is not None:
        for method in filesystem['methods'][6:]:
            method['id'] = tail_last + 1
            tail_last = method['id']
    periph = (periph_binding.select(ir, tail_last + 1)
              if only is None or periph_binding.OWNER in only else None)
    if periph is not None:
        tail_last = periph['methods'][-1]['id']
    old_parameter = (old_parameter_binding.select(ir, tail_last + 1)
                     if only is None or old_parameter_binding.OWNER in only else None)
    if old_parameter is not None:
        tail_last = old_parameter['methods'][-1]['id']
    printf = printf_binding.specification(tail_last + 1)
    shared_state = shared_state_binding.specification(printf['id'] + 1)
    example_constants = example_constants_binding.select(ir)
    for owner in sorted(ir['userdata'], key=lambda o: o['name']):
        if only is None and owner['name'] in value_binding.TYPES:
            continue
        selected = next((o for o in math['types'] if o['name'] == owner['name']), None)
        owner_methods = parameter['methods'] if owner['name'] == 'Parameter' else selected['methods'] if selected else []
        generated = {m['name'] for m in owner_methods}
        generated.update(m['method'] for m in values if m['owner'] == owner['name'])
        for method in sorted(owner['methods'], key=lambda m: m['name']):
            included = method['name'] in generated
            coverage.append(dict(owner=owner['name'], name=method['name'], line=method['line'], kind='userdata-method',
                                 status='generated' if included else 'unsupported',
                                 reason='shared-parameter-adapter' if included and owner['name'] == 'Parameter' else
                                 'value-adapter' if included else 'userdata-ownership-or-mutation'))
        for alias in sorted(owner['aliases'], key=lambda a: a['alias']):
            coverage.append(dict(owner=owner['name'], name=alias['alias'], kind='userdata-alias',
                                 status='unsupported', reason='manual-lua' if alias['manual'] else 'userdata-alias'))
    headers = []
    for header in ir['includes']:
        if only is not None and header['name'] not in {c['include'] for c in only.values()}:
            continue
        override = policy['headers'].get(header['name'])
        if override:
            if not override['reason']:
                raise ValueError('unexplained header policy')
            if override['replace'] is None:
                continue
            header = dict(header, name=override['replace'])
        headers.append(header)
    for entry in coverage:
        if entry['owner'] == 'AP_AHRS' and entry['name'] == 'wind_estimate':
            entry.update(status='generated', api='ahrs_wind_estimate',
                         reason='native-rust-deprecated-compatibility-wrapper')
        deprecated_compatibility = {
            ('AP_AHRS', 'get_roll'): 'ahrs_get_roll',
            ('AP_AHRS', 'get_pitch'): 'ahrs_get_pitch',
            ('AP_AHRS', 'get_yaw'): 'ahrs_get_yaw',
            ('RangeFinder', 'distance_cm_orient'): 'rangefinder_distance_cm_orient',
            ('RangeFinder', 'ground_clearance_cm_orient'): 'rangefinder_ground_clearance_cm_orient',
            ('RangeFinder', 'max_distance_cm_orient'): 'rangefinder_max_distance_cm_orient',
            ('RangeFinder', 'min_distance_cm_orient'): 'rangefinder_min_distance_cm_orient',
            ('Location', 'get_vector_from_origin_NEU'): 'Location.get_vector_from_origin_neu',
        }
        compatibility_api = deprecated_compatibility.get((entry['owner'], entry['name']))
        if compatibility_api is not None:
            entry.update(status='generated', api=compatibility_api,
                         reason='native-rust-deprecated-compatibility-wrapper')
        if entry['owner'] in ('uint32_t', 'uint64_t') and entry['status'] == 'unsupported':
            entry.update(status='generated', api='native Rust integer operation',
                         reason='native-rust-integer-operation')
        if entry['owner'] == 'AP_BattMonitor' and entry['name'] == 'handle_scripting':
            entry.update(status='generated', api=battery['name'], reason='owned-battery-state-adapter')
        if entry['owner'] == 'DroneCAN_Handle' and entry['name'] in ('subscribe', 'check_message', '__gc'):
            api = {'subscribe': dronecan['methods'][0]['name'],
                   'check_message': dronecan['methods'][1]['name'],
                   '__gc': dronecan['methods'][2]['name']}[entry['name']]
            entry.update(status='generated', api=api, reason='module-lifecycle-owned-adapter')
        if entry['owner'] == 'DroneCAN_Handle' and entry['name'] == 'broadcast':
            entry.update(status='generated', api=dronecan['methods'][4]['name'],
                         reason='module-lifecycle-owned-adapter')
        if entry['owner'] == 'DroneCAN_Handle' and entry['name'] == 'request':
            entry.update(status='generated', api=dronecan['methods'][6]['name'],
                         reason='module-lifecycle-owned-adapter')
        if (command_int is not None and entry['owner'] == command_int_binding.OWNER and
                entry['name'] == command_int_binding.ALIAS):
            entry.update(status='generated', api=command_int['name'], reason='typed-command-int-adapter')
        if (serial is not None and entry['owner'] == 'serial' and
                entry['name'] in ('find_serial', 'find_simulated_device')):
            api = {'find_serial': serial['methods'][0]['name'],
                   'find_simulated_device': serial['methods'][1]['name']}[entry['name']]
            entry.update(status='generated', api=api, reason='revalidated-serial-selector-adapter')
        if serial is not None and entry['owner'] == 'AP_Scripting_SerialAccess':
            serial_api = {
                'begin': serial['methods'][2]['name'],
                'configure_parity': serial['methods'][3]['name'],
                'set_stop_bits': serial['methods'][4]['name'],
                'set_unbuffered_writes': serial['methods'][5]['name'],
                'set_flow_control': serial['methods'][6]['name'],
                'available': serial['methods'][7]['name'],
                'read': 'SerialPort.read_byte', 'readstring': serial['methods'][8]['name'],
                'write': 'SerialPort.write_byte', 'writestring': serial['methods'][9]['name'],
            }
            if entry['name'] in serial_api:
                entry.update(status='generated', api=serial_api[entry['name']],
                             reason='revalidated-serial-selector-adapter')
        if (mavlink is not None and entry['owner'] == mavlink_binding.OWNER and
                entry['name'] in mavlink_binding.ALIASES):
            method = next(method for method in mavlink['methods']
                          if method['name'] == f'mavlink.{entry["name"]}')
            entry.update(status='generated', api=method['name'], reason='module-lifecycle-owned-adapter')
        if (arming is not None and entry['owner'] == arming_binding.OWNER and
                entry['name'] in ('get_aux_auth_id', 'set_aux_auth_passed', 'set_aux_auth_failed')):
            method = next(method for method in arming['methods']
                          if method['name'] == f'arming.{entry["name"]}')
            entry.update(status='generated', api=method['name'], reason='module-lifecycle-owned-adapter')
        if efi is not None and entry['owner'] == efi_binding.OWNER and entry['name'] == 'get_backend':
            entry.update(status='generated', api=efi['methods'][0]['name'],
                         reason='revalidated-efi-backend-adapter')
        if efi is not None and entry['owner'] == efi_binding.OWNER and entry['name'] == 'get_state':
            entry.update(status='generated', api=efi['methods'][2]['name'],
                         reason='owned-efi-state-adapter')
        if efi is not None and entry['owner'] == efi_binding.BACKEND and entry['name'] == 'handle_scripting':
            entry.update(status='generated', api=efi['methods'][1]['name'],
                         reason='owned-efi-state-adapter')
        if rangefinder_proximity is not None:
            sensor_apis = {
                (rangefinder_proximity_binding.RANGEFINDER_OWNER, 'get_backend'): 0,
                (rangefinder_proximity_binding.RANGEFINDER_BACKEND, 'type'): 1,
                (rangefinder_proximity_binding.RANGEFINDER_BACKEND, 'status'): 2,
                (rangefinder_proximity_binding.PROXIMITY_OWNER, 'get_backend'): 5,
                (rangefinder_proximity_binding.PROXIMITY_BACKEND, 'type'): 6,
                (rangefinder_proximity_binding.PROXIMITY_BACKEND, 'handle_script_distance_msg'): 7,
                (rangefinder_proximity_binding.PROXIMITY_BACKEND, 'set_distance_min_max'): 8,
                (rangefinder_proximity_binding.PROXIMITY_BACKEND, 'update_virtual_boundary'): 9,
                (rangefinder_proximity_binding.RANGEFINDER_BACKEND, 'orientation'): 11,
                (rangefinder_proximity_binding.PROXIMITY_BACKEND, 'handle_script_3d_msg'): 12,
            }
            sensor_index = sensor_apis.get((entry['owner'], entry['name']))
            if sensor_index is not None:
                entry.update(status='generated', api=rangefinder_proximity['methods'][sensor_index]['name'],
                             reason='revalidated-scripting-backend-adapter')
            if (entry['owner'] == rangefinder_proximity_binding.RANGEFINDER_BACKEND and
                    entry['name'] == 'handle_script_msg'):
                entry.update(status='generated',
                             api=[rangefinder_proximity['methods'][3]['name'],
                                  rangefinder_proximity['methods'][4]['name']],
                             reason='typed-rangefinder-script-message-adapter')
            if entry['owner'] == rangefinder_proximity_binding.RANGEFINDER_BACKEND:
                snapshot_apis = {
                    'distance': 'RangeFinderBackend.distance',
                    'signal_quality_pct': 'RangeFinderBackend.signal_quality',
                    'get_state': 'RangeFinderBackend.get_state',
                }
                snapshot_api = snapshot_apis.get(entry['name'])
                if snapshot_api is not None:
                    entry.update(status='generated', api=snapshot_api,
                                 reason='owned-rangefinder-state-snapshot')
        if filesystem is not None:
            if entry['owner'] == filesystem_binding.OWNER and entry['name'] == 'stat':
                entry.update(status='generated', api=filesystem['methods'][0]['name'],
                             reason='owned-filesystem-stat-adapter')
            if entry['owner'] == filesystem_binding.STAT and entry['name'] == 'is_directory':
                entry.update(status='generated', api='FileStat.is_directory',
                             reason='owned-filesystem-stat-adapter')
            if entry['owner'] == filesystem_binding.OWNER and entry['name'] == 'format':
                entry.update(status='generated', api=filesystem['methods'][6]['name'],
                             reason='guarded-filesystem-format-adapter')
        if network is not None and entry['owner'] == network_binding.OWNER:
            network_apis = {
                'bind': 2, 'listen': 3, 'set_blocking': 4, 'is_connected': 5,
                'pollin': 6, 'pollout': 7, 'accept': 8, 'recv': 9,
                'send': 10, 'sendfile': 11, 'connect': 12, 'close': 1, 'is_pending': 14,
                'reuseaddress': 15, 'sendto': 16,
            }
            network_index = network_apis.get(entry['name'])
            if network_index is not None:
                entry.update(status='generated', api=network['methods'][network_index]['name'],
                             reason='module-lifecycle-owned-socket-adapter')
        if (network is not None and entry['owner'] == network_binding.NETWORK_OWNER and
                entry['name'] == 'address_to_str'):
            entry.update(status='generated', api='Ipv4Address', reason='native-rust-ipv4-formatting')
        if can is not None:
            if entry['owner'] == can_binding.OWNER and entry['name'] in ('get_device', 'get_device2'):
                can_index = 0 if entry['name'] == 'get_device' else 1
                entry.update(status='generated', api=can['methods'][can_index]['name'],
                             reason='module-lifecycle-owned-can-adapter')
            can_buffer_apis = {'read_frame': 2, 'write_frame': 3, 'add_filter': 4}
            if entry['owner'] == can_binding.BUFFER and entry['name'] in can_buffer_apis:
                entry.update(status='generated', api=can['methods'][can_buffer_apis[entry['name']]]['name'],
                             reason='module-lifecycle-owned-can-adapter')
            if entry['owner'] == can_binding.FRAME and entry['name'] in {
                    'id_signed', 'isExtended', 'isRemoteTransmissionRequest', 'isErrorFrame'}:
                frame_api = {
                    'id_signed': 'id_signed',
                    'isExtended': 'is_extended',
                    'isRemoteTransmissionRequest': 'is_remote_transmission_request',
                    'isErrorFrame': 'is_error_frame',
                }[entry['name']]
                entry.update(status='generated', api='CanFrame.' + frame_api,
                             reason='owned-can-frame-value')
        if telemetry is not None:
            telemetry_apis = {
                (telemetry_binding.ESC_OWNER, 'update_telem_data'): 0,
                (telemetry_binding.SERVO_OWNER, 'get_telem'): 1,
                (telemetry_binding.SERVO_OWNER, 'update_telem_data'): 2,
            }
            telemetry_index = telemetry_apis.get((entry['owner'], entry['name']))
            if telemetry_index is not None:
                entry.update(status='generated', api=telemetry['methods'][telemetry_index]['name'],
                             reason='owned-telemetry-value-adapter')
        if airspeed is not None:
            airspeed_apis = {
                (airspeed_binding.OWNER, 'get_backend'): 0,
                (airspeed_binding.BACKEND, 'handle_script_airspeed'): 1,
                (airspeed_binding.BACKEND, 'handle_script_differential_pressure'): 2,
                (airspeed_binding.BACKEND, 'handle_script_temperature'): 3,
            }
            airspeed_index = airspeed_apis.get((entry['owner'], entry['name']))
            if airspeed_index is not None:
                entry.update(status='generated', api=airspeed['methods'][airspeed_index]['name'],
                             reason='revalidated-scripting-backend-adapter')
        if (periph is not None and entry['owner'] == periph_binding.OWNER and
                entry['name'] == 'get_vehicle_state'):
            entry.update(status='generated', api=periph['methods'][0]['name'],
                         reason='owned-u64-state-adapter')
        if (old_parameter is not None and entry['owner'] == old_parameter_binding.OWNER and
                entry['name'] == 'init_by_info'):
            entry.update(status='generated', api=old_parameter['methods'][0]['name'],
                         reason='leak-free-owned-old-parameter-value-adapter')
        if motor_config is not None:
            motor_apis = {
                (motor_config_binding.DYNAMIC_OWNER, 'init'): 0,
                (motor_config_binding.DYNAMIC_OWNER, 'add_motor'): 1,
                (motor_config_binding.DYNAMIC_OWNER, 'load_factors'): 2,
                (motor_config_binding.MATRIX_OWNER, 'init'): 3,
                (motor_config_binding.MATRIX_OWNER, 'add_motor_raw'): 4,
                (motor_config_binding.MATRIX_OWNER, 'set_throttle_factor'): 5,
                (motor_config_binding.MOTORS_OWNER, 'set_frame_string'): 6,
                (motor_config_binding.SIX_DOF_OWNER, 'init'): 7,
                (motor_config_binding.SIX_DOF_OWNER, 'add_motor'): 8,
            }
            motor_index = motor_apis.get((entry['owner'], entry['name']))
            if motor_index is not None:
                entry.update(status='generated', api=motor_config['methods'][motor_index]['name'],
                             reason='serialized-persistent-motor-configuration-adapter')
        if serialled is not None and entry['owner'] == serialled_binding.OWNER:
            serialled_names = [method['name'].rsplit('.', 1)[1] for method in serialled['methods']]
            if entry['name'] in serialled_names:
                serialled_index = serialled_names.index(entry['name'])
                entry.update(status='generated', api=serialled['methods'][serialled_index]['name'],
                             reason='module-channel-claimed-persistent-driver-adapter')
        if (onvif is not None and entry['owner'] == onvif_binding.OWNER and
                entry['name'] == onvif_binding.METHOD):
            entry.update(status='generated', api=onvif['name'],
                         reason='single-module-persistent-client-adapter')
        if applet is not None:
            applet_apis = {
                (applet_binding.GPS_OWNER, 'time_epoch_usec'): 0,
                (applet_binding.GPS_OWNER, 'inject_data'): 1,
                (applet_binding.CAMERA_OWNER, 'set_stream_information'): 2,
                (applet_binding.NETWORKING_OWNER, 'add_route'): 3,
                (applet_binding.SCRIPTING_OWNER, 'restart_all'): 4,
            }
            applet_index = applet_apis.get((entry['owner'], entry['name']))
            if applet_index is not None:
                entry.update(status='generated', api=applet['methods'][applet_index]['name'],
                             reason='owned-applet-adapter')
        if crsf is not None:
            crsf_apis = {
                (crsf_binding.OWNER, 'add_menu'): 0,
                (crsf_binding.MENU, 'add_menu'): 1,
                (crsf_binding.MENU, 'add_parameter'): 2,
                (crsf_binding.PARAMETER, 'id'): 3,
                (crsf_binding.PARAMETER, 'data'): 4,
                (crsf_binding.OWNER, 'peek_menu_event'): 5,
                (crsf_binding.OWNER, 'pop_menu_event'): 6,
                (crsf_binding.OWNER, 'send_write_response'): 7,
                (crsf_binding.OWNER, 'send_response'): 8,
                (crsf_binding.OWNER, 'get_menu_event'): 9,
                (crsf_binding.MENU, 'id'): 10,
                (crsf_binding.MENU, 'name'): 11,
                (crsf_binding.MENU, 'num_params'): 12,
            }
            crsf_index = crsf_apis.get((entry['owner'], entry['name']))
            if crsf_index is not None:
                entry.update(status='generated', api=crsf['methods'][crsf_index]['name'],
                             reason='module-lifecycle-owned-crsf-menu-adapter')
        if examples is not None:
            examples_apis = {
                (examples_binding.VEHICLE_OWNER, examples_binding.VELOCITY_ALIAS): 0,
                (examples_binding.SRV_OWNER, examples_binding.SAFETY_ALIAS): 1,
            }
            examples_index = examples_apis.get((entry['owner'], entry['name']))
            if examples_index is not None:
                entry.update(status='generated', api=examples['methods'][examples_index]['name'],
                             reason='typed-examples-adapter')
        if i2c is not None:
            i2c_apis = {
                (i2c_binding.OWNER, 'get_device'): 0,
                (i2c_binding.DEVICE, 'set_retries'): 2,
                (i2c_binding.DEVICE, 'set_address'): 3,
                (i2c_binding.DEVICE, 'write_register'): 4,
                (i2c_binding.DEVICE, 'read_registers'): 5,
                (i2c_binding.DEVICE, 'transfer'): 6,
            }
            i2c_index = i2c_apis.get((entry['owner'], entry['name']))
            if i2c_index is not None:
                entry.update(status='generated', api=i2c['methods'][i2c_index]['name'],
                             reason='module-lifecycle-owned-i2c-adapter')
        if camera is not None:
            camera_apis = {(camera_binding.OWNER, 'get_state'): 0,
                           (camera_binding.OWNER, 'set_camera_information'): 1}
            camera_index = camera_apis.get((entry['owner'], entry['name']))
            if camera_index is not None:
                entry.update(status='generated', api=camera['methods'][camera_index]['name'],
                             reason='owned-camera-value-adapter')
        if (custom_mode is not None and entry['owner'] == custom_mode_binding.OWNER and
                entry['name'] == 'register_custom_mode'):
            entry.update(status='generated', api=custom_mode['methods'][0]['name'],
                         reason='module-lifecycle-owned-custom-mode-adapter')
        if analog_pwm is not None:
            analog_apis = {
                (analog_pwm_binding.ANALOG_OWNER, 'channel'): 'analog.channel',
                (analog_pwm_binding.ANALOG_SOURCE, 'set_pin'): 'AnalogSource.set_pin',
                (analog_pwm_binding.ANALOG_SOURCE, 'voltage_average'): 'AnalogSource.voltage_average',
                (analog_pwm_binding.ANALOG_SOURCE, 'voltage_latest'): 'AnalogSource.voltage_latest',
                (analog_pwm_binding.ANALOG_SOURCE, 'voltage_average_ratiometric'):
                    'AnalogSource.voltage_average_ratiometric',
                (analog_pwm_binding.PWM_SOURCE, 'set_pin'): 'PwmSource.set_pin',
                (analog_pwm_binding.PWM_SOURCE, 'get_pwm_us'): 'PwmSource.get_pwm_us',
                (analog_pwm_binding.PWM_SOURCE, 'get_pwm_avg_us'): 'PwmSource.get_pwm_avg_us',
            }
            api = analog_apis.get((entry['owner'], entry['name']))
            if api is not None:
                entry.update(status='generated', api=api,
                             reason='module-lifecycle-owned-analog-pwm-adapter')
    report = dict(scope='Exported singleton/userdata/AP-object methods and aliases plus audited FWVersion fields; '
                        'other globals, fields, enums, constructors and operators are not counted',
                  lifecycle='Shared firmware state changes are not rolled back when a module exits; '
                            'DroneCAN handles, raw CAN buffers, MAVLink receive state, sockets, serial-LED/ONVIF '
                            'claims, CRSF menus/events, and auxiliary arming IDs are module-owned and cleaned '
                            'after worker join.',
                  summary=dict(Counter(c['kind'] + ':' + c['status'] for c in coverage)), entries=coverage)
    return dict(encoding=ENCODING, methods=methods, headers=headers,
                parameter=parameter, math=math, libm=libm, file=file, log=log, strtof=strtof,
                battery=battery, dronecan=dronecan, command_int=command_int, serial=serial, mavlink=mavlink,
                arming=arming, efi=efi, rangefinder_proximity=rangefinder_proximity,
                filesystem=filesystem, network=network, can=can, telemetry=telemetry,
                airspeed=airspeed,
                periph=periph,
                old_parameter=old_parameter,
                motor_config=motor_config, serialled=serialled, onvif=onvif, applet=applet, crsf=crsf,
                examples=examples, shared_state=shared_state,
                example_constants=example_constants, printf=printf,
                i2c=i2c,
                camera=camera,
                custom_mode=custom_mode,
                analog_pwm=analog_pwm,
                coverage=report)


def specification():
    return select(read_ir(), json.loads((ROOT / 'generator/bindings.json').read_text()))


def emit(contract):
    # All generated adapters, including specialised ones, trust the fixed buffers
    # supplied by generated Rust wrappers and verified by the loader's wire-contract
    # hashes. Repeating size/null checks cannot sandbox trusted native modules and
    # wastes flash; retain argument-value and variable-length payload validation.
    notice = '// SPDX-License-Identifier: GPL-3.0-or-later\n// Generated by generator/generate.py; do not edit.\n'
    cpp = [notice, '#include <AP_Module/Bindings.h>', '#if AP_MODULE_ENABLED',
           '#include <AP_Module/BindingSupport.h>', '#include <AP_Module/HostApi.h>',
           '#include <AP_Module/BindingParam.h>', '#include <AP_Module/BindingFile.h>',
           '#include <AP_Module/BindingLog.h>', '#include <AP_Module/BindingBattery.h>',
           '#include <AP_Module/BindingPrintf.h>', '#include <AP_Module/BindingSharedState.h>',
           '#include <AP_Module/BindingDroneCAN.h>',
           '#include <AP_Scripting/AP_Scripting_config.h>', '#include <AP_Vehicle/AP_Vehicle_Type.h>',
           '#include <AP_Scheduler/AP_Scheduler.h>', '#include <stdlib.h>']
    if contract['mavlink'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingMAVLink.h>')
    if contract['serial'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingSerial.h>')
    if contract['arming'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingArming.h>')
    if contract['efi'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingEFI.h>')
    if contract['rangefinder_proximity'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingRangeFinderProximity.h>')
    if contract['filesystem'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingFilesystem.h>')
    if contract['network'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingNetwork.h>')
    if contract['can'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingCAN.h>')
    if contract['telemetry'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingTelemetry.h>')
    if contract['airspeed'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingAirspeed.h>')
    if contract['periph'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingPeriph.h>')
    if contract['old_parameter'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingOldParameter.h>')
    if contract['motor_config'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingMotorConfig.h>')
    if contract['serialled'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingSerialLED.h>')
    if contract['onvif'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingONVIF.h>')
    if contract['applet'] is not None:
        cpp += applet_binding.cpp_headers()
    if contract['examples'] is not None:
        cpp += examples_binding.cpp_headers()
    if contract['crsf'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingCRSF.h>')
    if contract['i2c'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingI2C.h>')
    if contract['custom_mode'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingCustomMode.h>')
    if contract['analog_pwm'] is not None:
        cpp.insert(8, '#include <AP_Module/BindingAnalogPWM.h>')
    header_lines = []
    for header in contract['headers']:
        if header['depends']:
            header_lines.append('#if ' + header['depends'])
        header_lines.append(f'#include <{header["name"]}>')
        if header['depends']:
            header_lines.append('#endif')
    # Lua emits the list in both its generated header and source. Some guards
    # become defined by headers later in the first pass (for example OSD).
    cpp += ['// Match the Lua generated header/source dependency passes.'] + header_lines + header_lines
    cpp += ['#include <AP_Scripting/Parameter.h>', '#include <new>', '#include <type_traits>',
            '', 'namespace AP_Module', '{',
            f'static_assert(sizeof(Parameter) <= {parameter_binding.MAX_SIZE}, "Parameter layout size cap");',
            f'static_assert(alignof(Parameter) <= {parameter_binding.MAX_ALIGNMENT}, "Parameter layout alignment cap");',
            'static_assert(std::is_trivially_destructible<Parameter>::value, '
            '"Parameter layout-query requires trivial drop");', '']
    if any(m.get('literal') and m['owner'].startswith('hal.') for m in contract['methods']):
        cpp.insert(3, '#include <AP_HAL/AP_HAL.h>\nextern const AP_HAL::HAL &hal;')
    value_types = {f['type'] for m in contract['methods'] for f in [m['result']] + m['arguments']
                   if value_binding.is_value(f['type'])}
    value_types.update(m['value_receiver'] for m in contract['methods'] if m.get('value_receiver'))
    values = sorted(singleton_binding.TYPES[name][0] for name in value_types)
    if any(f['type'] == 'fwstring' for m in contract['methods'] for f in [m['result']] + m['arguments']):
        values.append('FirmwareString')
    imports = ', '.join(['result', 'Context', 'Error', 'Result'] + values)
    rust = [notice, f'use crate::{{{imports}}};', '']
    handles = {m['result']['type'] for m in contract['methods'] if m['result']['type'] in borrowed_binding.TYPES}
    cpp += borrowed_binding.cpp_helpers(handles)
    cpp += value_binding.cpp_helpers(value_types)
    for method in contract['methods']:
        cpp += singleton_binding.cpp(method)
    namespaces = {}
    for method in contract['methods']:
        if method.get('receiver') or method.get('value_receiver'):
            continue
        public_namespace, public_method = method['api_name'].split('.', 1)
        namespace = naming.rust_identifier(public_namespace)
        namespaced = dict(method)
        namespaced['rust_name'] = naming.rust_identifier(public_method)
        namespaced['aliases'] = [naming.rust_identifier(alias.split('.', 1)[1])
                                 for alias in method['api_aliases']]
        namespaces.setdefault(namespace, []).append(namespaced)
    for namespace in sorted(MANUAL_NAMESPACES):
        namespaces.setdefault(namespace, [])
    for namespace, methods in namespaces.items():
        namespace_type = naming.rust_type(namespace)
        method_lines = []
        for method in methods:
            method_lines += singleton_binding.rust(method, 'self.context')
        rust += render_lines(
            'rust/namespace.rs', namespace=namespace, namespace_type=namespace_type,
            methods=''.join(line + '\n' for line in method_lines),
        ) + ['']
    cpp += libm_binding.cpp(contract['libm'])
    cpp += strtof_binding.cpp(contract['strtof'])
    if contract['command_int'] is not None:
        cpp += command_int_binding.cpp(contract['command_int'])
    if contract['applet'] is not None:
        cpp += applet_binding.cpp(contract['applet'])
    if contract['examples'] is not None:
        cpp += examples_binding.cpp(contract['examples'])
    if contract['camera'] is not None:
        cpp += camera_binding.cpp(contract['camera'])
    cpp += ['int32_t binding_dispatch(void *context, BindingId id, const uint8_t *input, uint32_t input_size,',
            '                         uint8_t *output, uint32_t output_size)', '{', '    switch (id) {']
    cpp += [f'    case {contract["printf"]["id"]}:',
            '        return binding_string_format_printf(input, input_size, output, output_size);']
    for method in contract['methods']:
        context = 'context, ' if method['owner'] == 'AP_ONVIF' else ''
        cpp += [f'    case {method["id"]}:',
                f'        return binding_{method["id"]}({context}input, input_size, output, output_size);']
    cpp += parameter_binding.cpp_cases(contract['parameter'])
    for method in contract['libm']:
        cpp += [f'    case {method["id"]}:',
                f'        return binding_libm_{method["name"]}(input, input_size, output, output_size);']
    cpp += [f'    case {contract["file"]["id"]}:',
            '        return binding_file_read_at(input, input_size, output, output_size);']
    cpp += [f'    case {contract["log"]["id"]}:',
            '        return binding_log_write(input, input_size, output, output_size);']
    cpp += [f'    case {contract["strtof"]["id"]}:',
            '        return binding_string_strtof(input, input_size, output, output_size);']
    cpp += [f'    case {contract["battery"]["id"]}:',
            '        return binding_battery_handle_scripting(input, input_size, output, output_size);']
    dronecan = contract['dronecan']['methods']
    cpp += [f'    case {dronecan[0]["id"]}:',
            '        return binding_dronecan_subscribe(context, input, input_size, output, output_size);',
            f'    case {dronecan[1]["id"]}:',
            '        return binding_dronecan_check_message(context, input, input_size, output, output_size);',
            f'    case {dronecan[2]["id"]}:',
            '        return binding_dronecan_close(context, input, input_size, output, output_size);']
    if len(dronecan) > 3:
        cpp += [f'    case {dronecan[3]["id"]}:',
                '        return binding_dronecan_open(context, input, input_size, output, output_size);',
                f'    case {dronecan[4]["id"]}:',
                '        return binding_dronecan_broadcast(context, input, input_size, output, output_size);',
                f'    case {dronecan[5]["id"]}:',
                '        return binding_dronecan_close(context, input, input_size, output, output_size);',
                f'    case {dronecan[6]["id"]}:',
                '        return binding_dronecan_request(context, input, input_size, output, output_size);']
    if contract['command_int'] is not None:
        cpp += [f'    case {contract["command_int"]["id"]}:',
                f'        return binding_{contract["command_int"]["id"]}(input, input_size, output, output_size);']
    if contract['serial'] is not None:
        serial = contract['serial']['methods']
        serial_calls = ['find', 'find_simulated', 'begin', 'configure_parity', 'set_stop_bits',
                        'set_unbuffered_writes', 'set_flow_control', 'available', 'read', 'write']
        for method, call in zip(serial, serial_calls):
            cpp += [f'    case {method["id"]}:',
                    f'        return binding_serial_{call}(input, input_size, output, output_size);']
    if contract['mavlink'] is not None:
        mavlink = contract['mavlink']['methods']
        mavlink_calls = ['init', 'register_rx_msgid', 'receive_chan', 'send_chan', 'block_command']
        for method, call in zip(mavlink, mavlink_calls):
            cpp += [f'    case {method["id"]}:',
                    f'        return binding_mavlink_{call}(context, input, input_size, output, output_size);']
    if contract['arming'] is not None:
        arming = contract['arming']['methods']
        arming_calls = ['get_aux_auth_id', 'set_aux_auth_passed', 'set_aux_auth_failed']
        for method, call in zip(arming, arming_calls):
            cpp += [f'    case {method["id"]}:',
                    f'        return binding_arming_{call}(context, input, input_size, output, output_size);']
    if contract['efi'] is not None:
        efi = contract['efi']['methods']
        cpp += [f'    case {efi[0]["id"]}:',
                '        return binding_efi_get_backend(input, input_size, output, output_size);',
                f'    case {efi[1]["id"]}:',
                '        return binding_efi_handle_scripting(input, input_size, output, output_size);',
                f'    case {efi[2]["id"]}:',
                '        return binding_efi_get_state(input, input_size, output, output_size);']
    if contract['rangefinder_proximity'] is not None:
        sensors = contract['rangefinder_proximity']['methods']
        sensor_calls = ['rangefinder_get_backend', 'rangefinder_type', 'rangefinder_status',
                        'rangefinder_handle_script_distance', 'rangefinder_handle_script_state',
                        'proximity_get_backend', 'proximity_type', 'proximity_handle_script_distance',
                        'proximity_set_distance_min_max', 'proximity_update_virtual_boundary',
                        'rangefinder_get_state', 'rangefinder_orientation', 'proximity_handle_script_3d']
        for method, call in zip(sensors, sensor_calls):
            cpp += [f'    case {method["id"]}:',
                    f'        return binding_{call}(input, input_size, output, output_size);']
    if contract['filesystem'] is not None:
        filesystem = contract['filesystem']['methods']
        filesystem_calls = ['stat', 'read_at', 'dir_entry', 'truncate', 'append', 'unlink', 'format']
        for method, call in zip(filesystem, filesystem_calls):
            cpp += [f'    case {method["id"]}:',
                    f'        return binding_filesystem_{call}(input, input_size, output, output_size);']
    if contract['network'] is not None:
        network = contract['network']['methods']
        network_calls = ['open', 'close', 'bind', 'listen', 'set_blocking', 'is_connected',
                         'pollin', 'pollout', 'accept', 'recv', 'send', 'sendfile', 'connect',
                         'recv_from', 'is_pending', 'reuse_address', 'send_to']
        for method, call in zip(network, network_calls):
            cpp += [f'    case {method["id"]}:',
                    f'        return binding_network_{call}(context, input, input_size, output, output_size);']
    if contract['can'] is not None:
        can = contract['can']['methods']
        can_calls = ['get_device', 'get_device2', 'read_frame', 'write_frame', 'add_filter', 'close']
        for method, call in zip(can, can_calls):
            cpp += [f'    case {method["id"]}:',
                    f'        return binding_can_{call}(context, input, input_size, output, output_size);']
    if contract['telemetry'] is not None:
        telemetry = contract['telemetry']['methods']
        cpp += [f'    case {telemetry[0]["id"]}:',
                '        return binding_esc_telem_update_telem_data(input, input_size, output, output_size);',
                f'    case {telemetry[1]["id"]}:',
                '        return binding_servo_telem_get_telem(input, input_size, output, output_size);',
                f'    case {telemetry[2]["id"]}:',
                '        return binding_servo_telem_update_telem_data(input, input_size, output, output_size);']
    if contract['airspeed'] is not None:
        airspeed = contract['airspeed']['methods']
        cpp += [f'    case {airspeed[0]["id"]}:',
                '        return binding_airspeed_get_backend(input, input_size, output, output_size);',
                f'    case {airspeed[1]["id"]}:',
                '        return binding_airspeed_script_value(input, input_size, output, output_size, 0);',
                f'    case {airspeed[2]["id"]}:',
                '        return binding_airspeed_script_value(input, input_size, output, output_size, 1);',
                f'    case {airspeed[3]["id"]}:',
                '        return binding_airspeed_script_value(input, input_size, output, output_size, 2);']
    if contract['periph'] is not None:
        method = contract['periph']['methods'][0]
        cpp += [f'    case {method["id"]}:',
                '        return binding_periph_get_vehicle_state(input, input_size, output, output_size);']
    if contract['old_parameter'] is not None:
        method = contract['old_parameter']['methods'][0]
        cpp += [f'    case {method["id"]}:',
                '        return binding_parameter_old_value(input, input_size, output, output_size);']
    if contract['motor_config'] is not None:
        motor_config = contract['motor_config']['methods']
        motor_calls = ['motors_dynamic_init', 'motors_dynamic_add_motor', 'motors_dynamic_load_factors',
                       'motors_matrix_init', 'motors_matrix_add_motor_raw',
                       'motors_matrix_set_throttle_factor', 'motors_set_frame_string',
                       'motors_6dof_init', 'motors_6dof_add_motor']
        for method, call in zip(motor_config, motor_calls):
            cpp += [f'    case {method["id"]}:',
                    f'        return binding_{call}(input, input_size, output, output_size);']
    if contract['serialled'] is not None:
        serialled = contract['serialled']['methods']
        serialled_calls = ['set_num_neopixel', 'set_num_neopixel_rgb', 'set_num_profiled']
        for method, call in zip(serialled, serialled_calls):
            cpp += [f'    case {method["id"]}:',
                    f'        return binding_serialled_{call}(context, input, input_size, output, output_size);']
    if contract['onvif'] is not None:
        cpp += [f'    case {contract["onvif"]["id"]}:',
                '        return binding_onvif_start(context, input, input_size, output, output_size);']
    if contract['applet'] is not None:
        applet = contract['applet']['methods']
        applet_calls = ['gps_time_epoch_usec', 'gps_inject_data', 'camera_set_stream_information',
                        'networking_add_route', 'scripting_restart_all']
        for method, call in zip(applet, applet_calls):
            cpp += [f'    case {method["id"]}:',
                    f'        return binding_applet_{call}(input, input_size, output, output_size);']
    if contract['crsf'] is not None:
        crsf = contract['crsf']['methods']
        crsf_calls = ['add_menu', 'menu_add_menu', 'menu_add_parameter', 'parameter_id',
                      'parameter_data', 'peek_menu_event', 'pop_menu_event',
                      'send_write_response', 'send_response', 'get_menu_event',
                      'menu_id', 'menu_name', 'menu_num_params']
        for method, call in zip(crsf, crsf_calls):
            cpp += [f'    case {method["id"]}:',
                    f'        return binding_crsf_{call}(context, input, input_size, output, output_size);']
    if contract['examples'] is not None:
        examples = contract['examples']['methods']
        example_calls = ['vehicle_set_target_velocity_ned', 'srv_channels_get_safety_state',
                         'mission_receive', 'dronecan_get_flex_debug']
        for method, call in zip(examples, example_calls):
            cpp += [f'    case {method["id"]}:',
                    f'        return binding_examples_{call}(input, input_size, output, output_size);']
    if contract['i2c'] is not None:
        i2c = contract['i2c']['methods']
        i2c_calls = ['open', 'close', 'set_retries', 'set_address', 'write_register',
                     'read_registers', 'transfer']
        for method, call in zip(i2c, i2c_calls):
            cpp += [f'    case {method["id"]}:',
                    f'        return binding_i2c_{call}(context, input, input_size, output, output_size);']
    if contract['camera'] is not None:
        camera = contract['camera']['methods']
        camera_calls = ['get_state', 'set_information']
        for method, call in zip(camera, camera_calls):
            cpp += [f'    case {method["id"]}:',
                    f'        return binding_camera_{call}(input, input_size, output, output_size);']
    if contract['custom_mode'] is not None:
        custom_mode = contract['custom_mode']['methods']
        custom_calls = ['register', 'get_allow_entry', 'set_allow_entry']
        for method, call in zip(custom_mode, custom_calls):
            cpp += [f'    case {method["id"]}:',
                    f'        return binding_custom_mode_{call}(context, input, input_size, output, output_size);']
    if contract['analog_pwm'] is not None:
        analog_pwm = contract['analog_pwm']['methods']
        analog_pwm_calls = [
            'analog_open', 'analog_set_pin', 'analog_voltage_average', 'analog_voltage_latest',
            'analog_voltage_average_ratiometric', 'pwm_open', 'pwm_close', 'pwm_set_pin',
            'pwm_get_us', 'pwm_get_average_us',
        ]
        for method, call in zip(analog_pwm, analog_pwm_calls):
            cpp += [f'    case {method["id"]}:',
                    f'        return binding_{call}(context, input, input_size, output, output_size);']
    for method in contract['shared_state']['methods']:
        call = method['name'].split('.')[1]
        cpp += [f'    case {method["id"]}:',
                f'        return binding_shared_state_{call}(context, input, input_size, output, output_size);']
    cpp += ['    default:', '        return binding_math_dispatch(id, input, input_size, output, output_size);',
            '    }', '}', '}', '#endif', '']
    binding_names = contracts.binding_names(contract)
    cpp_output = '\n'.join(cpp)
    cpp_output = re.sub(
        r'\bbinding_(\d+)\b',
        lambda match: 'binding_' + binding_names[int(match.group(1))],
        cpp_output,
    )
    cpp_output = re.sub(
        r'(?m)^(\s*)case (\d+):',
        lambda match: (match.group(1) + 'case BindingId::' +
                       binding_names[int(match.group(2))].upper() + ':'),
        cpp_output,
    )
    rust_output = re.sub(
        r'crate::imports::BINDING_(\d+)',
        lambda match: 'crate::imports::BINDING_' + binding_names[int(match.group(1))].upper(),
        '\n'.join(rust),
    )
    outputs = {
        'Bindings_generated.cpp': cpp_output,
        'sdk/rust/src/bindings.rs': rust_output,
        'sdk/rust/src/enums.rs': singleton_binding.rust_enums(contract['methods'], notice),
        'sdk/rust/src/format.rs': printf_binding.rust(contract['printf'], notice),
        'sdk/rust/src/shared_state.rs': shared_state_binding.rust(contract['shared_state'], notice),
        'sdk/rust/src/borrowed.rs': borrowed_binding.rust(
            [m for m in contract['methods'] if m.get('receiver')], notice, singleton_binding.rust),
        'sdk/rust/src/values.rs': value_binding.rust(contract['methods'], notice, singleton_binding.rust),
        'sdk/rust/src/libm.rs': libm_binding.rust(contract['libm'], notice),
        'sdk/rust/src/strtof.rs': strtof_binding.rust(contract['strtof'], notice),
        'sdk/rust/src/file.rs': file_binding.rust(contract['file'], notice),
        'sdk/rust/src/log.rs': log_binding.rust(contract['log'], notice),
        'sdk/rust/src/battery.rs': battery_binding.rust(contract['battery'], notice),
        'sdk/rust/src/dronecan.rs': dronecan_binding.rust(contract['dronecan'], notice),
        'BindingLog_generated.h': log_binding.header(notice),
        'sdk/rust/src/parameter.rs': parameter_binding.rust(contract['parameter'], notice),
        'sdk/rust/src/math.rs': math_binding.rust(contract['math'], notice),
        'BindingMath_generated.cpp': math_binding.cpp(contract['math'], notice),
        'BindingConfig.h': notice + '#pragma once\n\n' +
        f'#define AP_MODULE_FILE_PATH_SIZE {file_binding.PATH_SIZE}U\n' +
        f'#define AP_MODULE_FILE_READ_MAX {file_binding.READ_MAX}U\n' +
        f'#define AP_MODULE_SERIAL_TRANSFER_MAX {serial_binding.TRANSFER_MAX}U\n' +
        f'#define AP_MODULE_BINDING_PARAMETER_LAYOUT {contract["parameter"]["layout"]["id"]}U\n' +
        ''.join(f'#define AP_MODULE_BINDING_PARAMETER_{m["name"].upper()} {m["id"]}U\n'
                for m in contract['parameter']['methods']) +
        ''.join(f'#define AP_MODULE_BINDING_{o["name"].upper()}_{m["name"].upper()} {m["id"]}U\n'
                for o in contract['math']['types'] for m in o['methods']),
        'generator/bindings_contract.json':
            json.dumps({k: v for k, v in contract.items() if k != 'coverage'}, indent=2) + '\n',
        'generator/coverage.json': json.dumps(contract['coverage'], indent=2) + '\n',
    }
    if contract['command_int'] is not None:
        outputs['sdk/rust/src/command_int.rs'] = command_int_binding.rust(contract['command_int'], notice)
    if contract['serial'] is not None:
        outputs['sdk/rust/src/serial.rs'] = serial_binding.rust(contract['serial'], notice)
    if contract['mavlink'] is not None:
        outputs['sdk/rust/src/mavlink.rs'] = mavlink_binding.rust(contract['mavlink'], notice)
    if contract['arming'] is not None:
        outputs['sdk/rust/src/arming.rs'] = arming_binding.rust(contract['arming'], notice)
    if contract['efi'] is not None:
        outputs['sdk/rust/src/efi.rs'] = efi_binding.rust(contract['efi'], notice)
    if contract['rangefinder_proximity'] is not None:
        outputs['sdk/rust/src/rangefinder_proximity.rs'] = rangefinder_proximity_binding.rust(
            contract['rangefinder_proximity'], notice)
    if contract['filesystem'] is not None:
        outputs['sdk/rust/src/filesystem.rs'] = filesystem_binding.rust(contract['filesystem'], notice)
    if contract['network'] is not None:
        outputs['sdk/rust/src/network.rs'] = network_binding.rust(contract['network'], notice)
    if contract['can'] is not None:
        outputs['sdk/rust/src/can.rs'] = can_binding.rust(contract['can'], notice)
    if contract['telemetry'] is not None:
        outputs['sdk/rust/src/telemetry.rs'] = telemetry_binding.rust(contract['telemetry'], notice)
    if contract['airspeed'] is not None:
        outputs['sdk/rust/src/airspeed.rs'] = airspeed_binding.rust(contract['airspeed'], notice)
    if contract['periph'] is not None:
        outputs['sdk/rust/src/periph.rs'] = periph_binding.rust(contract['periph'], notice)
    if contract['old_parameter'] is not None:
        outputs['sdk/rust/src/old_parameter.rs'] = old_parameter_binding.rust(contract['old_parameter'], notice)
    if contract['motor_config'] is not None:
        outputs['sdk/rust/src/motor_config.rs'] = motor_config_binding.rust(contract['motor_config'], notice)
    if contract['serialled'] is not None:
        outputs['sdk/rust/src/serialled.rs'] = serialled_binding.rust(contract['serialled'], notice)
    if contract['onvif'] is not None:
        outputs['sdk/rust/src/onvif.rs'] = onvif_binding.rust(contract['onvif'], notice)
    if contract['applet'] is not None:
        outputs['sdk/rust/src/applet.rs'] = applet_binding.rust(contract['applet'], notice)
    if contract['crsf'] is not None:
        outputs['sdk/rust/src/crsf.rs'] = crsf_binding.rust(contract['crsf'], notice)
    if contract['examples'] is not None:
        outputs['sdk/rust/src/examples.rs'] = examples_binding.rust(contract['examples'], notice)
    outputs['sdk/rust/src/example_constants.rs'] = example_constants_binding.rust(
        contract['example_constants'], notice)
    if contract['i2c'] is not None:
        outputs['sdk/rust/src/i2c.rs'] = i2c_binding.rust(contract['i2c'], notice)
    if contract['camera'] is not None:
        outputs['sdk/rust/src/camera.rs'] = camera_binding.rust(contract['camera'], notice)
    if contract['custom_mode'] is not None:
        outputs['sdk/rust/src/custom_mode.rs'] = custom_mode_binding.rust(contract['custom_mode'], notice)
    if contract['analog_pwm'] is not None:
        outputs['sdk/rust/src/analog_pwm.rs'] = analog_pwm_binding.rust(contract['analog_pwm'], notice)
    return outputs
