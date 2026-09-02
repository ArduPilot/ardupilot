#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Inventory ArduPilot I2C/UART drivers and validate Renode coverage."""

import argparse
import json
import re
import string
import sys

from pathlib import Path

from driver_catalog import ATTACHABLE_DEVICES
from driver_catalog import DRIVER_PROBE_PROFILES
from driver_catalog import I2C_HWDEF_CLASSIFICATIONS
from driver_catalog import I2C_SOURCE_BOUNDARIES
from driver_catalog import I2C_SOURCE_CLASSIFICATIONS
from driver_catalog import SERIAL_PROTOCOL_CLASSIFICATIONS
from driver_catalog import SERIAL_SOURCE_CLASSIFICATIONS
from driver_catalog import i2c_endpoints

TRANSPORT_INTERFACES = {
    'i2c': frozenset(('II2CPeripheral',)),
    'uart': frozenset(('IUART', 'IUARTWithBufferState')),
}
VALID_BUSES = frozenset(('can', 'i2c', 'uart'))
VALID_COVERAGE = frozenset(('detection', 'dynamic'))
VALID_HOTPLUG = frozenset((
    'boot-probe', 'parameter-reinit', 'runtime',
))
VALID_SELECTION = frozenset(('auto-probe', 'hwdef-probe'))
VALID_RECIPE_FIELDS = frozenset((
    'airspeed_prefix', 'battery_prefix', 'i2c_index', 'instance', 'serial',
))
VALID_SOURCE_ROLES = frozenset((
    'base', 'device', 'frontend', 'internal', 'link', 'output', 'service',
))
VALID_I2C_BOUNDARY_SCOPES = frozenset((
    'ap-periph', 'linux-example', 'linux-hal',
))
VALID_PROBE_ASSERTIONS = frozenset((
    'adc-recovery', 'capacity-scaler', 'checksum-recovery',
    'configuration', 'corrupt-read-recovery',
    'data-ready-recovery',
    'data-invalid-recovery', 'detection', 'device-id', 'stable-values',
    'stepped-values',
    'output-corruption-recovery',
    'output-suppression-recovery', 'stuck-sample-recovery',
))
SOURCE_EXCLUDED_PARTS = frozenset((
    'examples', 'tests', 'AP_HAL', 'AP_HAL_ChibiOS', 'AP_HAL_Empty',
    'AP_HAL_ESP32', 'AP_HAL_Linux', 'AP_HAL_QURT', 'AP_HAL_SITL', 'SITL',
))


def _source_files(root):
    libraries = Path(root) / 'libraries'
    for path in sorted(libraries.rglob('*.cpp')):
        relative = path.relative_to(libraries)
        if SOURCE_EXCLUDED_PARTS.intersection(relative.parts):
            continue
        yield path, relative


def direct_i2c_sources(root):
    """Return production sources which directly acquire an I2C device."""
    patterns = (
        'I2CDevice',
        'i2c_mgr->get_device',
        'GET_I2C_DEVICE',
    )
    found = []
    for path, relative in _source_files(root):
        text = path.read_text(errors='replace')
        if any(pattern in text for pattern in patterns):
            found.append(relative.as_posix())
    return found


def serial_driver_sources(root):
    """Return production sources which directly acquire a serial port."""
    patterns = (
        'AP_HAL::UARTDriver',
        'find_serial(',
    )
    found = []
    for path, relative in _source_files(root):
        text = path.read_text(errors='replace')
        if any(pattern in text for pattern in patterns):
            found.append(relative.as_posix())
    return found


def serial_protocols(root):
    """Return the named AP_SerialManager protocols in declaration order."""
    header = (Path(root) / 'libraries' / 'AP_SerialManager' /
              'AP_SerialManager.h')
    text = header.read_text()
    match = re.search(
        r'enum SerialProtocol\s*\{(?P<body>.*?)\n\s*\};',
        text,
        re.DOTALL,
    )
    if match is None:
        raise ValueError('cannot find AP_SerialManager::SerialProtocol')
    protocols = []
    for raw_line in match.group('body').splitlines():
        line = raw_line.split('//', 1)[0]
        entry = re.match(r'\s*SerialProtocol_([A-Za-z0-9_]+)', line)
        if entry is not None and entry.group(1) != 'NumProtocols':
            protocols.append(entry.group(1))
    return protocols


def hwdef_i2c_probes(root):
    """Return named IMU, barometer and compass I2C probes from all hwdefs."""
    hwdef_root = (Path(root) / 'libraries' / 'AP_HAL_ChibiOS' / 'hwdef')
    probes = {'barometer': set(), 'compass': set(), 'imu': set()}
    kinds = {'BARO': 'barometer', 'COMPASS': 'compass', 'IMU': 'imu'}
    for path in sorted(hwdef_root.rglob('hwdef*.dat')):
        for raw_line in path.read_text(errors='replace').splitlines():
            line = raw_line.split('#', 1)[0].strip()
            fields = line.split()
            if (len(fields) < 3 or fields[0] not in kinds or
                    not any(field.startswith('I2C:') for field in fields[2:])):
                continue
            model = fields[1].split(':', 1)[0]
            probes[kinds[fields[0]]].add(model)
    return {kind: sorted(models) for kind, models in probes.items()}


def renode_transport_models(root):
    """Return C# peripheral classes grouped by direct or inherited transport."""
    peripheral_root = Path(root) / 'Tools' / 'renode' / 'peripherals'
    models = {transport: {} for transport in TRANSPORT_INTERFACES}
    declarations = {}
    declaration = re.compile(
        r'public\s+(?P<abstract>abstract\s+)?class\s+'
        r'(?P<name>[A-Za-z0-9_]+)\s*'
        r':\s*(?P<bases>[^\n{]+)')
    for path in sorted(peripheral_root.rglob('*.cs')):
        text = path.read_text(errors='replace')
        for match in declaration.finditer(text):
            declarations[match.group('name')] = {
                'abstract': match.group('abstract') is not None,
                'bases': set(re.findall(r'[A-Za-z_][A-Za-z0-9_]*',
                                        match.group('bases'))),
                'path': path.relative_to(Path(root)).as_posix(),
            }
    for transport, interfaces in TRANSPORT_INTERFACES.items():
        resolved = set()
        changed = True
        while changed:
            changed = False
            for name, entry in declarations.items():
                if name in resolved:
                    continue
                if (interfaces.intersection(entry['bases']) or
                        entry['bases'].intersection(resolved)):
                    resolved.add(name)
                    changed = True
        models[transport] = {
            name: declarations[name]['path'] for name in resolved
            if not declarations[name]['abstract']
        }
    return models


def validate_catalog(root):
    """Return catalog errors without aborting at the first malformed entry."""
    root = Path(root)
    models = renode_transport_models(root)
    errors = []
    required = (
        'name', 'category', 'bus', 'driver', 'feature', 'coverage', 'hotplug',
    )
    for device_id, device in sorted(ATTACHABLE_DEVICES.items()):
        missing = [field for field in required if field not in device]
        if missing:
            errors.append('%s lacks %s' % (device_id, ', '.join(missing)))
            continue
        bus = device['bus']
        if bus not in VALID_BUSES:
            errors.append('%s has invalid bus %s' % (device_id, bus))
        if device['coverage'] not in VALID_COVERAGE:
            errors.append('%s has invalid coverage %s' %
                          (device_id, device['coverage']))
        if device['hotplug'] not in VALID_HOTPLUG:
            errors.append('%s has invalid hotplug policy %s' %
                          (device_id, device['hotplug']))
        driver = root / 'libraries' / device['driver']
        if not driver.is_file():
            errors.append('%s driver does not exist: %s' %
                          (device_id, device['driver']))
        else:
            feature_found = False
            for source in driver.parent.iterdir():
                if source.suffix not in ('.cpp', '.h'):
                    continue
                if device['feature'] in source.read_text(errors='replace'):
                    feature_found = True
                    break
            if not feature_found:
                errors.append('%s feature does not exist beside driver: %s' %
                              (device_id, device['feature']))
        if bus == 'i2c':
            endpoints = i2c_endpoints(device)
            if (not isinstance(endpoints, tuple) or not endpoints or
                    ('i2c_endpoints' in device and
                     ('model' in device or 'address' in device))):
                errors.append('%s has invalid I2C endpoints' % device_id)
                endpoints = ()
            addresses = set()
            for endpoint in endpoints:
                if (not isinstance(endpoint, dict) or
                        set(endpoint) != {'address', 'model'}):
                    errors.append('%s has malformed I2C endpoint %r' %
                                  (device_id, endpoint))
                    continue
                address = endpoint['address']
                if not isinstance(address, int) or not 0 <= address <= 0x7F:
                    errors.append('%s has invalid I2C address %r' %
                                  (device_id, address))
                elif address in addresses:
                    errors.append('%s repeats I2C address 0x%02X' %
                                  (device_id, address))
                addresses.add(address)
        if bus in TRANSPORT_INTERFACES:
            transport_models = (
                [endpoint['model'] for endpoint in i2c_endpoints(device)]
                if bus == 'i2c' else [device.get('model', '')])
            for transport_model in transport_models:
                model = (transport_model.rsplit('.', 1)[-1]
                         if isinstance(transport_model, str) else '')
                if model not in models[bus]:
                    errors.append('%s model %s does not implement %s' %
                                  (device_id, model, '/'.join(sorted(
                                      TRANSPORT_INTERFACES[bus]))))
        physics = device.get('physics')
        if physics is not None:
            if (not isinstance(physics, dict) or
                    set(physics) != {'count', 'property', 'source'} or
                    not isinstance(physics.get('source'), str) or
                    not physics.get('source') or
                    not isinstance(physics.get('property'), str) or
                    not physics.get('property') or
                    isinstance(physics.get('count'), bool) or
                    not isinstance(physics.get('count'), int) or
                    physics.get('count') < 1):
                errors.append('%s has invalid physics channel metadata' %
                              device_id)
        recipe = device.get('parameters')
        selection = device.get('selection')
        if recipe is None and selection not in VALID_SELECTION:
            errors.append('%s has no parameter recipe or valid selection' %
                          device_id)
        if recipe is not None and selection is not None:
            errors.append('%s has both parameter recipe and selection' %
                          device_id)
        parameter_names = set()
        for parameter in recipe or ():
            if (not isinstance(parameter, tuple) or len(parameter) != 2 or
                    not isinstance(parameter[0], str) or
                    not isinstance(parameter[1], (int, float, str))):
                errors.append('%s has invalid parameter recipe entry %r' %
                              (device_id, parameter))
                continue
            fields = set()
            for template in parameter:
                if not isinstance(template, str):
                    continue
                try:
                    fields.update(field for _, field, _, _ in
                                  string.Formatter().parse(template) if field)
                except ValueError:
                    errors.append('%s has malformed parameter template %r' %
                                  (device_id, template))
            unknown_fields = fields - VALID_RECIPE_FIELDS
            if unknown_fields:
                errors.append('%s has unknown parameter fields %s' %
                              (device_id, ', '.join(sorted(unknown_fields))))
            if 'serial' in fields and bus != 'uart':
                errors.append('%s uses a serial field on %s' %
                              (device_id, bus))
            if 'i2c_index' in fields and bus != 'i2c':
                errors.append('%s uses an I2C field on %s' %
                              (device_id, bus))
            if parameter[0] in parameter_names:
                errors.append('%s repeats parameter %s' %
                              (device_id, parameter[0]))
            parameter_names.add(parameter[0])
    return errors


def _default_parameters(path):
    parameters = {}
    for raw_line in path.read_text(errors='replace').splitlines():
        fields = raw_line.split('#', 1)[0].split()
        if len(fields) >= 2:
            parameters[fields[0]] = fields[1]
    return parameters


def validate_probe_profiles(root):
    """Return errors for malformed or recipe-inconsistent probe profiles."""
    root = Path(root)
    errors = []
    required = ('board', 'build_target', 'defaults', 'devices', 'vehicle')
    for profile_id, profile in sorted(DRIVER_PROBE_PROFILES.items()):
        missing = [field for field in required if field not in profile]
        if missing:
            errors.append('%s probe profile lacks %s' %
                          (profile_id, ', '.join(missing)))
            continue
        defaults_path = root / profile['defaults']
        if not defaults_path.is_file():
            errors.append('%s defaults do not exist: %s' %
                          (profile_id, profile['defaults']))
            defaults = {}
        else:
            defaults = _default_parameters(defaults_path)
        extra_hwdef = profile.get('extra_hwdef')
        if extra_hwdef is not None and not (root / extra_hwdef).is_file():
            errors.append('%s extra hwdef does not exist: %s' %
                          (profile_id, extra_hwdef))
        seen_devices = set()
        for attachment in profile['devices']:
            if not isinstance(attachment, dict):
                errors.append('%s has a non-object attachment' % profile_id)
                continue
            device_id = attachment.get('device')
            device = ATTACHABLE_DEVICES.get(device_id)
            if device is None:
                errors.append('%s uses unknown device %s' %
                              (profile_id, device_id))
                continue
            if device_id in seen_devices:
                errors.append('%s repeats device %s' %
                              (profile_id, device_id))
            seen_devices.add(device_id)
            port_id = attachment.get('port', '')
            bus_prefix = {'can': 'CAN', 'i2c': 'I2C', 'uart': 'SERIAL'}[
                device['bus']]
            if not port_id.startswith(bus_prefix):
                errors.append('%s attaches %s to incompatible port %s' %
                              (profile_id, device_id, port_id))
            assertions = attachment.get('assertions')
            if not assertions:
                errors.append('%s has no assertions for %s' %
                              (profile_id, device_id))
            else:
                unknown = set(assertions) - VALID_PROBE_ASSERTIONS
                if unknown:
                    errors.append('%s has unknown assertions for %s: %s' %
                                  (profile_id, device_id,
                                   ', '.join(sorted(unknown))))
                if (device['coverage'] == 'dynamic' and
                        'stepped-values' not in assertions):
                    errors.append('%s does not dynamically test %s' %
                                  (profile_id, device_id))
            instance = attachment.get('instance', 1)
            battery_instances = attachment.get('battery_instances')
            recipe_instances = (instance,)
            if battery_instances is not None:
                if (device['category'] != 'Power' or
                        not isinstance(battery_instances, tuple) or
                        not battery_instances or
                        any(isinstance(value, bool) or
                            not isinstance(value, int) or value < 1
                            for value in battery_instances) or
                        len(set(battery_instances)) != len(battery_instances)):
                    errors.append('%s has invalid battery instances for %s' %
                                  (profile_id, device_id))
                else:
                    recipe_instances = battery_instances
            contexts = []
            for recipe_instance in recipe_instances:
                contexts.append({
                    'instance': recipe_instance,
                    'airspeed_prefix': (
                        'ARSPD' if recipe_instance == 1 else
                        'ARSPD%u' % recipe_instance),
                    'battery_prefix': (
                        'BATT' if recipe_instance == 1 else
                        'BATT%u' % recipe_instance),
                })
            if device['bus'] == 'uart':
                match = re.fullmatch(r'SERIAL([0-9]+)', port_id)
                if match is None:
                    continue
                for context in contexts:
                    context['serial'] = int(match.group(1))
            elif device['bus'] == 'i2c':
                match = re.fullmatch(r'I2C([0-9]+)', port_id)
                if match is None:
                    continue
                for context in contexts:
                    context['i2c_index'] = int(match.group(1))
            else:
                continue
            for context in contexts:
                for name, value in device.get('parameters', ()):
                    try:
                        name = name.format(**context)
                        value = (value.format(**context)
                                 if isinstance(value, str) else value)
                    except (KeyError, TypeError, ValueError):
                        continue
                    if defaults.get(name) != str(value):
                        errors.append('%s defaults require %s %s for %s' %
                                      (profile_id, name, value, device_id))
    return errors


def validate_classifications(root):
    """Return errors for unclassified or stale production inventory names."""
    errors = []
    protocols = set(serial_protocols(root))
    classified_protocols = set(SERIAL_PROTOCOL_CLASSIFICATIONS)
    for protocol in sorted(protocols - classified_protocols):
        errors.append('unclassified serial protocol %s' % protocol)
    for protocol in sorted(classified_protocols - protocols):
        errors.append('stale serial protocol classification %s' % protocol)

    probes = hwdef_i2c_probes(root)
    for kind, discovered in probes.items():
        classified = set(I2C_HWDEF_CLASSIFICATIONS.get(kind, {}))
        discovered = set(discovered)
        for model in sorted(discovered - classified):
            errors.append('unclassified %s I2C probe %s' % (kind, model))
        for model in sorted(classified - discovered):
            errors.append('stale %s I2C probe classification %s' %
                          (kind, model))

    sources = set(direct_i2c_sources(root))
    classified_sources = set(I2C_SOURCE_CLASSIFICATIONS)
    for source in sorted(sources - classified_sources):
        errors.append('unclassified direct I2C source %s' % source)
    for source in sorted(classified_sources - sources):
        errors.append('stale direct I2C source classification %s' % source)
    catalog_sources = {
        device['driver'] for device in ATTACHABLE_DEVICES.values()
        if device['bus'] == 'i2c'
    }
    concrete_sources = {
        source for source, role in I2C_SOURCE_CLASSIFICATIONS.items()
        if role == 'device'
    }
    boundary_sources = set(I2C_SOURCE_BOUNDARIES)
    for source in sorted(concrete_sources - catalog_sources - boundary_sources):
        errors.append('uncovered direct I2C device source %s' % source)
    for source in sorted(boundary_sources - sources):
        errors.append('stale direct I2C source boundary %s' % source)
    for source in sorted(boundary_sources & catalog_sources):
        errors.append('catalogued direct I2C source has boundary %s' % source)
    for source, boundary in I2C_SOURCE_BOUNDARIES.items():
        if (I2C_SOURCE_CLASSIFICATIONS.get(source) != 'device' or
                not isinstance(boundary, tuple) or len(boundary) != 2 or
                boundary[0] not in VALID_I2C_BOUNDARY_SCOPES or
                not isinstance(boundary[1], str) or not boundary[1]):
            errors.append('invalid direct I2C source boundary for %s' % source)

    sources = set(serial_driver_sources(root))
    classified_sources = set(SERIAL_SOURCE_CLASSIFICATIONS)
    for source in sorted(sources - classified_sources):
        errors.append('unclassified direct UART source %s' % source)
    for source in sorted(classified_sources - sources):
        errors.append('stale direct UART source classification %s' % source)
    for source, classification in SERIAL_SOURCE_CLASSIFICATIONS.items():
        if (not isinstance(classification, tuple) or
                len(classification) != 2 or
                classification[0] not in VALID_SOURCE_ROLES or
                not isinstance(classification[1], str) or
                not classification[1]):
            errors.append('invalid direct UART classification for %s' %
                          source)
    return errors


def inventory(root):
    root = Path(root).resolve()
    probes = hwdef_i2c_probes(root)
    models = renode_transport_models(root)
    catalog = {
        device_id: {
            field: value for field, value in device.items()
            if field in ('address', 'bus', 'category', 'coverage', 'driver',
                         'feature', 'hotplug', 'model', 'name', 'parameters',
                         'physics', 'selection', 'sidecar', 'i2c_endpoints')
        }
        for device_id, device in sorted(ATTACHABLE_DEVICES.items())
    }
    return {
        'catalog': catalog,
        'catalog_errors': validate_catalog(root),
        'classification_errors': validate_classifications(root),
        'probe_profile_errors': validate_probe_profiles(root),
        'probe_profiles': DRIVER_PROBE_PROFILES,
        'direct_i2c_sources': direct_i2c_sources(root),
        'hwdef_i2c_probes': probes,
        'i2c_source_classifications': dict(sorted(
            I2C_SOURCE_CLASSIFICATIONS.items())),
        'i2c_source_boundaries': {
            source: {'scope': boundary[0], 'reason': boundary[1]}
            for source, boundary in sorted(I2C_SOURCE_BOUNDARIES.items())
        },
        'renode_models': {
            transport: sorted(entries)
            for transport, entries in models.items()
        },
        'serial_driver_sources': serial_driver_sources(root),
        'serial_source_classifications': {
            source: {'role': classification[0], 'family': classification[1]}
            for source, classification in sorted(
                SERIAL_SOURCE_CLASSIFICATIONS.items())
        },
        'serial_protocols': serial_protocols(root),
        'serial_protocol_classifications': {
            name: {'kind': classification[0], 'status': classification[1]}
            for name, classification in
            SERIAL_PROTOCOL_CLASSIFICATIONS.items()
        },
    }


def print_report(result):
    probes = result['hwdef_i2c_probes']
    print('ArduPilot Renode peripheral-driver inventory')
    print('  attachable catalog: %u devices' % len(result['catalog']))
    print('  direct I2C source candidates: %u' %
          len(result['direct_i2c_sources']))
    print('  hwdef I2C probe families: %u barometer, %u compass, %u IMU' % (
        len(probes['barometer']), len(probes['compass']), len(probes['imu'])))
    print('  serial protocols: %u' % len(result['serial_protocols']))
    print('  direct serial source candidates: %u' %
          len(result['serial_driver_sources']))
    print('  Renode transport models: %u I2C, %u UART' % (
        len(result['renode_models']['i2c']),
        len(result['renode_models']['uart'])))
    coverage = {}
    for device in result['catalog'].values():
        key = (device['bus'], device['coverage'])
        coverage[key] = coverage.get(key, 0) + 1
    for (bus, level), count in sorted(coverage.items()):
        print('    %s/%s: %u' % (bus, level, count))
    if result['catalog_errors']:
        print('catalog errors:')
        for error in result['catalog_errors']:
            print('  %s' % error)
    if result['classification_errors']:
        print('classification errors:')
        for error in result['classification_errors']:
            print('  %s' % error)
    if result['probe_profile_errors']:
        print('probe profile errors:')
        for error in result['probe_profile_errors']:
            print('  %s' % error)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--root', type=Path,
        default=Path(__file__).resolve().parents[2],
        help='ArduPilot repository root')
    parser.add_argument('--json', action='store_true',
                        help='write the complete inventory as JSON')
    parser.add_argument('--check', action='store_true',
                        help='fail if the attachable catalog is inconsistent')
    args = parser.parse_args(argv)
    result = inventory(args.root)
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print_report(result)
    if args.check and (result['catalog_errors'] or
                       result['classification_errors'] or
                       result['probe_profile_errors']):
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
