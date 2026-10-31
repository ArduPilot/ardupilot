# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Parsed method IR must describe the same input as the Lua backend."""

import json
import subprocess
import sys

from pathlib import Path

import pytest

GENERATOR = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def generator(tmp_path_factory):
    del tmp_path_factory
    return [sys.executable, str(GENERATOR / 'generator.py')]


def dump(generator, tmp_path, description):
    source = tmp_path / 'bindings.desc'
    source.write_text(description)
    output = tmp_path / 'bindings.json'
    subprocess.run(generator + ['-i', str(source), '--dump-ir', str(output)], check=True)
    return json.loads(output.read_text())


def test_repository_ir_and_lua_output_unchanged(generator, tmp_path):
    source = GENERATOR / 'description/bindings.desc'
    output = tmp_path / 'bindings.json'
    first, second = tmp_path / 'first', tmp_path / 'second'
    subprocess.run(generator + ['-i', str(source), '-o', str(first)], check=True)
    subprocess.run(generator + ['-i', str(source), '-o', str(second), '--dump-ir', str(output)], check=True)
    for suffix in ('.cpp', '.h'):
        assert first.with_suffix(suffix).read_bytes() == second.with_suffix(suffix).read_bytes()
    ir = json.loads(output.read_text())
    assert ir['schema'] == 1
    owners = {item['name']: item for item in ir['singletons']}
    ahrs = owners['AP_AHRS']
    assert ahrs['semaphore'] and not ahrs['scheduler_semaphore']
    assert ahrs['depends'] == 'AP_AHRS_ENABLED'
    methods = {item['name']: item for item in ahrs['methods']}
    assert methods['airspeed_EAS']['arguments'][0]['nullable']
    assert methods['airspeed_EAS']['arguments'][0]['kind'] == 'float'
    assert methods['get_roll']['deprecate'] == 'Use get_roll_rad'
    assert any(item['name'] == 'airspeed_EAS' and item['alias'] == 'airspeed_estimate' for item in ahrs['aliases'])
    assert owners['AP_Vehicle']['scheduler_semaphore']
    version = owners['AP::fwversion()']
    version_fields = {field['name']: field for field in version['fields']}
    assert set(version_fields) == {'fw_short_string', 'vehicle_type', 'major', 'minor', 'patch', 'fw_hash_str'}
    assert version_fields['fw_short_string']['rename'] == 'string'
    assert version_fields['fw_short_string']['type']['kind'] == 'string'
    assert version_fields['vehicle_type']['rename'] == 'type'
    assert version_fields['vehicle_type']['type']['kind'] == 'uint8_t'
    assert all(field['read'] and not field['write'] for field in version_fields.values())
    pwm = next(item for item in owners['RC_Channels']['methods'] if item['name'] == 'get_pwm')
    assert pwm['arguments'][0]['range'] == ['1', 'NUM_RC_CHANNELS']
    assert [item['kind'] for item in pwm['arguments']] == ['uint8_t', 'uint16_t']
    parameter = next(item for item in ir['userdata'] if item['name'] == 'Parameter')
    assert parameter['creation'] == 'lua_new_Parameter'
    assert parameter['creation_manual']
    assert parameter['creation_args'] == 0
    assert not parameter['has_fields'] and not parameter['has_enums']
    assert parameter['operations'] == 0 and parameter['operator_getter'] is None
    assert parameter['fields'] == [] and parameter['operator_names'] == []
    assert not parameter['operators_manual']
    methods = {item['name']: item for item in parameter['methods']}
    assert set(methods) == {'init', 'init_by_info', 'get', 'set', 'set_and_save', 'configured', 'set_default'}
    assert methods['init']['arguments'][0]['kind'] == 'string'
    assert methods['get']['arguments'][0]['nullable']
    assert methods['set']['arguments'][0]['skip_check']
    assert methods['init_by_info']['arguments'][2]['name'] == 'ap_var_type'
    userdata = {item['name']: item for item in ir['userdata']}
    for name, components, operators in [('Vector3f', ['x', 'y', 'z'], ['+', '-']),
                                        ('Quaternion', ['q1', 'q2', 'q3', 'q4'], ['*'])]:
        value = userdata[name]
        assert sorted(field['name'] for field in value['fields']) == components
        assert value['operator_names'] == operators and not value['operators_manual']
        for field in value['fields']:
            assert field['read'] and field['write'] and field['line'] > 0
            assert all(field[key] is None for key in ('rename', 'depends', 'array_len',
                                                      'valid_mask_name', 'valid_mask_value'))
            assert field['type'] == dict(kind='float', name=None, nullable=False, reference=False,
                                         pointer=False, skip_check=True, range=None)
    assert userdata['Quaternion']['depends'] == 'AP_AHRS_ENABLED'
    objects = {item['name']: item for item in ir['ap_objects']}
    rc = objects['RC_Channel']
    assert rc['depends'] == 'AP_RC_CHANNEL_ENABLED'
    assert {method['name'] for method in rc['methods']} == {
        'norm_input', 'norm_input_dz', 'get_aux_switch_pos', 'norm_input_ignore_trim', 'set_override',
    }
    assert objects['AP_HAL::I2CDevice']['semaphore_pointer']
    custom_mode = objects['AP_Vehicle::custom_mode_state']
    assert custom_mode['fields'][0]['name'] == 'allow_entry'
    assert userdata['uint32_t']['operators_manual']
    telemetry = userdata['AP_Servo_Telem::TelemetryData']
    field = next(field for field in telemetry['fields'] if field['name'] == 'command_position')
    assert field['valid_mask_name'] == 'present_types'
    assert field['valid_mask_value'] == 'AP_Servo_Telem::TelemetryData::Types::COMMANDED_POSITION'
    first_ir = output.read_bytes()
    subprocess.run(generator + ['-i', str(source), '--dump-ir', str(output)], check=True)
    assert output.read_bytes() == first_ir


def test_types_metadata_and_escaping(generator, tmp_path):
    ir = dump(generator, tmp_path, '''include Probe/Probe.h depends PROBE_ENABLED
userdata Value field x float'skip_check read write
ap_object Borrowed method healthy boolean
singleton Probe rename probe
singleton Probe depends PROBE_ENABLED && (MODE == 1)
singleton Probe get acquire_probe()
singleton Probe semaphore
singleton Probe field version uint8_t read
singleton Probe field version rename api_version
singleton Probe method read boolean uint8_t 1 (MAX-1) float'Null
singleton Probe method read rename read_value
singleton Probe method read alias old_read
singleton Probe method read depends READ_ENABLED
singleton Probe method read deprecate Use "new" \\ API
singleton Probe method mutate void float'Ref &uint16_t'Ref
singleton Probe method mixed uint32_t int8_t'skip_check int16_t'skip_check int32_t'skip_check uint32_t'skip_check
singleton Probe method object Value Borrowed string
singleton Probe method choice Mode'enum Mode'enum Mode::LOW Mode::HIGH 7'literal
singleton Probe manual custom lua_custom 1 1
''')
    assert ir['includes'] == [{'name': 'Probe/Probe.h', 'depends': 'PROBE_ENABLED'}]
    owner = ir['singletons'][0]
    assert owner['get'] == 'acquire_probe()'
    assert owner['depends'] == 'PROBE_ENABLED && (MODE == 1)'
    assert owner['fields'][0]['name'] == 'version'
    assert owner['fields'][0]['rename'] == 'api_version'
    assert owner['fields'][0]['read'] and not owner['fields'][0]['write']
    methods = {method['name']: method for method in owner['methods']}
    read = methods['read']
    assert read['rename'] == 'read_value'
    assert read['depends'] == 'READ_ENABLED'
    assert read['deprecate'] == 'Use "new" \\ API'
    assert read['line'] > 0
    assert read['arguments'][0]['range'] == ['1', '(MAX-1)']
    mutate = methods['mutate']
    assert mutate['return']['kind'] == 'void'
    assert mutate['arguments'][0]['reference'] and not mutate['arguments'][0]['pointer']
    assert mutate['arguments'][1]['reference'] and mutate['arguments'][1]['pointer']
    mixed = methods['mixed']
    assert mixed['return']['kind'] == 'uint32_t'
    assert [arg['kind'] for arg in mixed['arguments']] == ['int8_t', 'int16_t', 'int32_t', 'uint32_t']
    assert all(arg['skip_check'] and arg['range'] is None for arg in mixed['arguments'])
    assert methods['object']['return']['name'] == 'Value'
    assert methods['object']['arguments'][0]['kind'] == 'ap_object'
    assert methods['object']['arguments'][1]['kind'] == 'string'
    assert methods['choice']['return']['name'] == 'Mode'
    assert methods['choice']['arguments'][1]['kind'] == 'literal'
    assert methods['choice']['arguments'][1]['name'] == '7'
    assert next(item for item in owner['aliases'] if item['alias'] == 'custom')['manual']
    borrowed = ir['ap_objects'][0]
    assert borrowed['name'] == 'Borrowed' and borrowed['methods'][0]['name'] == 'healthy'
    assert borrowed['methods'][0]['return']['kind'] == 'boolean'


def test_userdata_contract_metadata(generator, tmp_path):
    ir = dump(generator, tmp_path, '''userdata Value field x float'skip_check read write
userdata Value enum MODE
userdata Value operator +
userdata Value operator_getter get_value
userdata Value creation lua_value -1
userdata Value rename value
userdata Value depends VALUE_ENABLED
userdata Value method read boolean float'Null
userdata Value method read rename read_value
userdata Value method read alias old_read
userdata Value method read depends READ_ENABLED
userdata Value method read deprecate Use new_read
userdata Value method mutate void float'Ref &uint16_t'Ref
userdata Value manual custom lua_custom 1 1
userdata Plain method valid boolean
''')
    assert ir['singletons'] == []
    owners = {item['name']: item for item in ir['userdata']}
    owner = owners['Value']
    assert owner['creation'] == 'lua_value' and owner['creation_args'] == -1
    assert owner['creation_manual']
    assert owner['rename'] == 'value' and owner['depends'] == 'VALUE_ENABLED'
    assert owner['has_fields'] and owner['has_enums'] and owner['operations'] != 0
    assert owner['operator_getter'] == 'get_value'
    methods = {item['name']: item for item in owner['methods']}
    assert methods['read']['rename'] == 'read_value'
    assert methods['read']['depends'] == 'READ_ENABLED'
    assert methods['read']['deprecate'] == 'Use new_read'
    assert methods['read']['arguments'][0]['nullable']
    assert methods['mutate']['arguments'][0]['reference']
    assert methods['mutate']['arguments'][1]['pointer']
    assert any(item['alias'] == 'old_read' and not item['manual'] for item in owner['aliases'])
    assert any(item['alias'] == 'custom' and item['manual'] for item in owner['aliases'])
    assert owners['Plain']['creation'] is None
    assert not owners['Plain']['creation_manual']
    assert not owners['Plain']['has_fields'] and not owners['Plain']['has_enums']


def test_userdata_field_contracts(generator, tmp_path):
    ir = dump(generator, tmp_path, '''userdata Value field samples'array NUM_SAMPLES int16_t read write -100 (MAX-1)
userdata Value field samples rename values
userdata Value field samples depends SAMPLES_ENABLED && (MODE == 1)
userdata Value field status boolean read
userdata Value field command float'skip_check write valid_mask flags VALID_COMMAND
''')
    fields = {field['name']: field for field in ir['userdata'][0]['fields']}
    assert fields['samples'] == dict(
        name='samples', rename='values', depends='SAMPLES_ENABLED && (MODE == 1)', array_len='NUM_SAMPLES',
        line=1, read=True, write=True, valid_mask_name=None, valid_mask_value=None,
        type=dict(kind='int16_t', name=None, nullable=False, reference=False, pointer=False,
                  skip_check=False, range=['-100', '(MAX-1)']))
    assert fields['status']['read'] and not fields['status']['write']
    assert fields['status']['type']['kind'] == 'boolean'
    assert not fields['command']['read'] and fields['command']['write']
    assert fields['command']['type']['skip_check']
    assert fields['command']['valid_mask_name'] == 'flags'
    assert fields['command']['valid_mask_value'] == 'VALID_COMMAND'


def test_userdata_operator_tokens(generator, tmp_path):
    tokens = ['+', '-', '*', '/', '%', '&', '|', '^', '<<', '>>', '==', '<', '<=', '~']
    field = "userdata Value field x float'skip_check read write\n"
    description = field + '\n'.join(f'userdata Value operator {token}' for token in reversed(tokens)) + '\n'
    ir = dump(generator, tmp_path, description)
    assert ir['userdata'][0]['operator_names'] == tokens
    assert not ir['userdata'][0]['operators_manual']
    ir = dump(generator, tmp_path, description + 'userdata Value manual_operator __tostring value_tostring\n')
    assert ir['userdata'][0]['operator_names'] == tokens
    assert ir['userdata'][0]['operators_manual']
    ir = dump(generator, tmp_path, field + 'userdata Value manual_operator __tostring value_tostring\n')
    assert ir['userdata'][0]['operator_names'] == []
    assert ir['userdata'][0]['operators_manual']


@pytest.mark.parametrize('declaration', ["field x float write 1", 'field x boolean', 'operator ??'])
def test_invalid_userdata_metadata(generator, tmp_path, declaration):
    source = tmp_path / 'bad.desc'
    source.write_text(f'userdata Value {declaration}\n')
    result = subprocess.run(generator + ['-i', str(source), '--dump-ir', str(tmp_path / 'bad.json')],
                            capture_output=True, text=True)
    assert result.returncode > 0
    assert 'Error' in result.stderr


@pytest.mark.parametrize('arguments', [[], ['--unknown'], ['--dump-ir', 'output.json']])
def test_bad_options_fail_cleanly(generator, tmp_path, arguments):
    result = subprocess.run(generator + arguments, cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode > 0
    assert 'error' in result.stderr.lower()


@pytest.mark.parametrize('owner', ['singleton', 'userdata'])
def test_ir_uses_existing_validation(generator, tmp_path, owner):
    source = tmp_path / 'bad.desc'
    source.write_text(f"{owner} Probe method bad float float'Null\n")
    result = subprocess.run(generator + ['-i', str(source), '--dump-ir', str(tmp_path / 'bad.json')],
                            capture_output=True, text=True)
    assert result.returncode > 0
    assert 'Nullable arguments are only available on a boolean method' in result.stderr
