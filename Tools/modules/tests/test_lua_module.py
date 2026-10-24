# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Execute the loadable C Lua interpreter with built-in Lua disabled."""

import importlib.util
import os
import re
import shutil
import subprocess

from pathlib import Path

import pack
import pytest

from test_runtime import ROOT
from test_runtime import Vehicle
from test_threads import gen_board  # noqa: F401
from test_threads import renode  # noqa: F401
from test_threads import renode_data  # noqa: F401
from test_threads import run  # noqa: F401

spec = importlib.util.spec_from_file_location('lua_module_build', ROOT / 'Tools/modules/lua/build.py')
lua_build = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lua_build)


@pytest.fixture(scope='module')
def lua_image(tmp_path_factory):
    output = tmp_path_factory.mktemp('lua-module') / 'lua.apmod'
    lua_build.build(output, 'x86_64-unknown-linux-gnu')
    return output.read_bytes()


def vehicle(tmp_path, image, scripts):
    result = Vehicle(tmp_path / 'vehicle', {'lua.apmod': image},
                     {'MOD_HEAP_MAX': 131072, 'MOD_CODE_MAXSIZE': 262144})
    directory = result.directory / 'scripts'
    directory.mkdir()
    for name, source in scripts.items():
        (directory / name).write_text(source)
    return result


def test_unchanged_examples(tmp_path, lua_image):
    names = ['hello_world.lua', 'simple_loop.lua', 'ahrs-print-angle-and-rates.lua', 'param_add.lua',
             'protected_call.lua']
    scripts = {name: (ROOT / 'libraries/AP_Scripting/examples' / name).read_text() for name in names}
    instance = vehicle(tmp_path, lua_image, scripts)
    try:
        instance.start()
        for restart in range(1, 4):
            before = len(instance.texts)
            instance.parameter('MOD_RESTART', restart)
            instance.text('Module: loaded 1', after=before)
            instance.text('hello, world', after=before)
            instance.text('Ang R:', after=before)
            instance.text('param1=3.14', after=before)
            instance.text('TICK ', after=before)
        for name, expected in [('MY_TEST', 3.14), ('MY_TEST2', 5.7)]:
            instance.link.mav.param_request_read_send(1, 1, name.encode(), -1)
            message = instance.until(lambda m: m.get_type() == 'PARAM_VALUE' and m.param_id == name)
            assert message.param_value == pytest.approx(expected)
        assert not any('returned -' in text or 'panic' in text for text in instance.texts), instance.diagnostic()
    finally:
        instance.stop()


def test_directory_with_lua_suffix_is_ignored(tmp_path, lua_image):
    instance = vehicle(tmp_path, lua_image, {'hello.lua': 'gcs:send_text(6,"Lua directory test OK")'})
    (instance.directory / 'scripts/not-a-script.lua').mkdir()
    try:
        instance.start()
        instance.text('Lua directory test OK')
        assert not any('returned -' in text for text in instance.texts), instance.diagnostic()
    finally:
        instance.stop()


@pytest.mark.parametrize('source', [
    'assert(uint32_t(1000) > 1000/3); assert(uint32_t(3.75):toint() == 3)',
    'assert(not param:set("LOG_DISARMED", 200)); assert(param:set("LOG_DISARMED", 1))',
    'assert(string.match(string.rep("a", 24), string.rep("(a*)", 12)) ~= nil)',
])
def test_firmware_compatibility(tmp_path, lua_image, source):
    instance = vehicle(tmp_path, lua_image, {'compat.lua': source + '; gcs:send_text(6,"Lua compatibility OK")'})
    try:
        instance.start()
        instance.text('Lua compatibility OK')
        assert not any('returned -' in text for text in instance.texts), instance.diagnostic()
    finally:
        instance.stop()


def test_finalizer_gets_a_fresh_instruction_budget(tmp_path, lua_image):
    instance = vehicle(tmp_path, lua_image, {'gc.lua': '''local finalized = 0
local function work()
    local total = 0
    for i = 1, 2750 do total = total + i end
    return total
end
local function collect() work(); finalized = finalized + 1 end
local function update()
    local garbage = setmetatable({}, {__gc = collect})
    assert(work() > 0)
    if finalized > 0 then gcs:send_text(6, "Lua finalizer OK") end
    return update, 100
end
return update()
'''})
    try:
        instance.start()
        instance.text('Lua finalizer OK')
        assert not any('returned -' in text for text in instance.texts), instance.diagnostic()
    finally:
        instance.stop()


def test_oversized_script_reports_path_and_limit(tmp_path, lua_image):
    instance = vehicle(tmp_path, lua_image, {'too-big.lua': '--' + 'x' * 65535})
    try:
        instance.start()
        instance.text('Lua script size 65537 exceeds 65536 byte limit: ./scripts/too-big.lua')
        instance.text('returned -')
    finally:
        instance.stop()


@pytest.mark.parametrize('source,error', [
    ('this is not Lua', 'syntax'),
    ('while true do end', 'Exceeded CPU time'),
    ('while true do pcall(function() while true do end end) end', 'Exceeded CPU time'),
    ('local function f() return 1+f() end; return f()', 'not enough memory'),
    ('local function f() return string.gsub("x", ".", f) end; f()', 'C stack overflow'),
    ('local s = string.rep("x", 1000000)', 'memory'),
    ('return function() end, -1', 'invalid callback delay'),
    ('return function() end, "100"', 'expected callback and delay'),
    # Binary loading is compiled out, so the text lexer rejects the signature.
    ('\x1bLua', 'unexpected symbol'),
    ('error(123)', 'Lua module error'),
    ('error({})', 'Lua module error'),
])
def test_errors_and_recovery(tmp_path, lua_image, source, error):
    instance = vehicle(tmp_path, lua_image, {'fault.lua': source})
    try:
        instance.start()
        instance.text('returned -')
        if error == 'syntax':
            assert any('syntax' in text or 'near' in text for text in instance.texts), instance.diagnostic()
        else:
            assert any(error in text for text in instance.texts), instance.diagnostic()
        (instance.directory / 'scripts/fault.lua').write_text('gcs:send_text(6,"Lua recovered")')
        before = len(instance.texts)
        instance.parameter('MOD_RESTART', 1)
        instance.text('Lua recovered', after=before)
        instance.check_alive()
    finally:
        instance.stop()


def test_environments_time_and_stop(tmp_path, lua_image):
    scripts = {
        'a.lua': '''private_value = 123
local function tick() assert(private_value == 123); return tick, 0 end
return tick, 0
''',
        'b.lua': '''assert(private_value == nil)
private_value = 456
assert((uint32_t(-1) + uint32_t(1)):toint() == 0)
assert((uint32_t(0) - uint32_t(1)):tofloat() > 4000000000)
local bad, err = pcall(Parameter, "MISSING_LUA_PARAM")
assert(not bad and type(err) == "string")
local start = millis()
local function tick()
  assert(private_value == 456)
  assert(millis() >= start)
  gcs:send_text(6, "Lua environments and time OK")
  return tick, 200
end
return tick, 0
''',
    }
    instance = vehicle(tmp_path, lua_image, scripts)
    try:
        instance.start()
        instance.text('Lua environments and time OK')
        for restart in range(1, 6):
            before = len(instance.texts)
            instance.parameter('MOD_RESTART', restart)
            instance.text('Lua environments and time OK', after=before)
        assert not any('returned -' in text or 'stop timeout' in text for text in instance.texts), instance.diagnostic()
    finally:
        instance.stop()


@pytest.mark.parametrize('at_close', [False, True])
def test_finalizer_is_bounded(tmp_path, lua_image, at_close):
    source = 'kept = setmetatable({}, {__gc=function() while true do end end})'
    if at_close:
        source += '''
local function tick() assert(kept); return tick,1000000 end
return tick,1000000
'''
    instance = vehicle(tmp_path, lua_image, {'finalizer.lua': source})
    try:
        instance.start()
        before = len(instance.texts)
        instance.parameter('MOD_RESTART', 1)
        instance.text('Module: loaded 1', after=before)
        instance.pump(1)
        assert not any('stop timeout' in text or 'panic' in text for text in instance.texts), instance.diagnostic()
    finally:
        instance.stop()


def test_parameter_set_does_not_save(tmp_path, lua_image):
    register = '''assert(param:add_table(73, "LTEST_", 1))
assert(param:add_param(73, 1, "VALUE", 3))
local p = Parameter("LTEST_VALUE")
'''
    source = register + '''assert(p:set_and_save(11))
local function change()
  assert(p:set(22))
  assert(p:get() == 22)
  gcs:send_text(6,"Lua unsaved parameter OK")
end
-- Let the asynchronous parameter save finish before changing RAM again.
return change,2000
'''
    instance = vehicle(tmp_path, lua_image, {'parameter.lua': source})
    try:
        instance.start()
        instance.text('Lua unsaved parameter OK')
        instance.pump(1)
        instance.stop()
        (instance.directory / 'scripts/parameter.lua').write_text(register + '''
assert(p:get() == 11, tostring(p:get()))
gcs:send_text(6,"Lua saved parameter restored")
''')
        before = len(instance.texts)
        instance.start()
        instance.text('Lua saved parameter restored', after=before)
    finally:
        instance.stop()


def test_prefixes_nesting_and_formatting(tmp_path, lua_image):
    nested = 'local x=1\n' + 'if x==1 then\n' * 9 + 'assert(x==1)\n' + 'end\n' * 9
    source = '\ufeff#!' + 'x' * 2000 + '\n' + nested + '''
assert(tonumber(string.rep("x",300)) == nil)
assert(tonumber(string.rep(" ",300).."1.25") == 1.25)
assert(tonumber("1.25"..string.rep(" ",300)) == 1.25)
assert(string.format("%#x",0) == "0")
assert(string.format("%#o",0) == "0")
assert(string.format("%#.3o",8) == "010")
assert(string.format("%.0d",0) == "")
assert(string.format("%c",0) == string.char(0))
assert(not pcall(string.format,"%a",1.25))
assert(not pcall(string.format,"%.61f",1.25))
gcs:send_text(6,"Lua prefixes, nesting and formatting OK")
'''
    instance = vehicle(tmp_path, lua_image, {'prefix.lua': source})
    try:
        instance.start()
        instance.text('Lua prefixes, nesting and formatting OK')
    finally:
        instance.stop()


def test_matches_normal_lua(tmp_path, lua_image, monkeypatch):
    reference = Path(os.environ.get('AP_MODULE_LUA_REFERENCE_BINARY', ROOT / 'build/lua-reference/sitl/bin/arduplane'))
    if not reference.is_file():
        pytest.skip('Build a normal Lua SITL binary or set AP_MODULE_LUA_REFERENCE_BINARY')
    source = '''local values = {
  string.format("%d:%#x:%#o:%#.3o:%+06d",-123,0,0,8,-123),
  string.format("%.1f:%.3f:%g",1.24,3.125,0.5),
  tostring(tonumber("0x10")),
  tostring(tonumber(string.rep("x",300))),
  tostring((uint32_t(0)-uint32_t(1)):toint()),
}
local function tick()
  for i,v in ipairs(values) do gcs:send_text(6,"Lua compare "..i..":"..v) end
  return tick,1000
end
return tick,0
'''
    module = vehicle(tmp_path, lua_image, {'compare.lua': source})
    stock = Vehicle(tmp_path / 'stock', {}, {'MOD_ENABLE': 0, 'SCR_ENABLE': 1, 'SCR_HEAP_SIZE': 131072})
    (stock.directory / 'scripts').mkdir()
    (stock.directory / 'scripts/compare.lua').write_text(source)
    try:
        module.start()
        module.text('Lua compare 5:')
        monkeypatch.setenv('AP_MODULE_SITL_BINARY', str(reference))
        stock.start()
        stock.text('Lua compare 5:')
        prefix = 'Lua compare '
        actual = {text[text.index(prefix):] for text in module.texts if prefix in text}
        expected = {text[text.index(prefix):] for text in stock.texts if prefix in text}
        assert len(actual) == 5 and actual == expected, (actual, expected)
    finally:
        module.stop()
        stock.stop()


def test_arm_packaging_and_rebasing(tmp_path):
    first = lua_build.build(tmp_path / 'first.apmod', 'thumbv7em-none-eabihf', cpu='cortex-m7')
    second = lua_build.build(tmp_path / 'second.apmod', 'thumbv7em-none-eabihf', cpu='cortex-m7', base=0x24123000)
    assert first.read_bytes() == second.read_bytes()
    info = pack.unpack(first.read_bytes())
    assert info['contract']['cpu_profile'] == 3
    assert info['contract']['heap_size'] == 131072
    pack.relocate(first.read_bytes(), 0x24000000, 0x30000000)
    pack.relocate(first.read_bytes(), 0x08020000, 0x30000000)


def test_h7_execution_and_reclamation(tmp_path, renode):  # noqa: F811
    from elftools.elf.elffile import ELFFile

    board = os.environ.get('OWNED_THREAD_BOARD', 'CubeOrangePlus')
    build_root = Path(os.environ.get('AP_MODULE_BUILD_ROOT', ROOT / 'build'))
    executable = build_root / board / 'examples/ModuleLua'
    if not executable.is_file():
        pytest.skip('Build examples/ModuleLua for the selected H7 board')
    if not shutil.which('mcopy') or not shutil.which('mmd'):
        pytest.skip('Requires mtools')
    names = ['result', 'cycles', 'before', 'after', 'heap_peak', 'stack_used', 'stack_total']
    with executable.open('rb') as stream:
        symbols = ELFFile(stream).get_section_by_name('.symtab')
        addresses = [symbols.get_symbol_by_name('lua_module_test_' + name)[0]['st_value'] for name in names]
        error = symbols.get_symbol_by_name('lua_module_test_error')[0]['st_value']
    image = lua_build.build(tmp_path / 'lua.apmod', 'thumbv7em-none-eabihf', cpu='cortex-m7')
    sdcard = run.make_fat_image(tmp_path / 'sdcard.img', 16 * 1024 * 1024, fat_bits=16)
    subprocess.run(['mmd', '-i', str(sdcard), '::/APM', '::/APM/modules', '::/APM/scripts'], check=True)
    source = tmp_path / 'test.lua'
    source.write_text('''local formatted = string.format("%.1f %u", 1.24, 123)
assert(formatted == "1.2 123", formatted)
assert(tonumber("1.25") == 1.25)
assert(math.abs(math.sin(1) - 0.84147098) < 0.00001)
assert((uint32_t(-1) + uint32_t(1)):toint() == 0)
local fn, parse_error = load(string.rep("if true then ",100).."return "..string.rep("end ",100))
assert(fn == nil and string.find(parse_error,"too many C levels"))
local function recursive() return string.gsub("x", ".", recursive) end
local ok, err = pcall(recursive)
assert(not ok and string.find(err, "C stack overflow"))
for i=1,30 do
  local ok, err = pcall(function() error("caught") end)
  assert(not ok and string.find(err, "caught"))
end
local closed = false
do
  local object = setmetatable({}, {__gc=function() closed = true end})
end
local function callback()
  assert(closed)
  gcs:send_text(6,"Lua native test OK")
end
return callback, 5
''')
    for path, destination in [(image, '::/APM/modules/lua.apmod'), (source, '::/APM/scripts/test.lua')]:
        subprocess.run(['mcopy', '-i', str(sdcard), str(path), destination], check=True)
    generated = gen_board.generate(ROOT, board, tmp_path / 'generated', uart_port=0, state_dir=tmp_path)
    data = renode_data.ensure_mcu_data(generated['mcu_type'])
    output = renode('''
$elf=@%s
$mcu_svd=@%s
$sdcard=@%s
$vector_base=0x%X
include @%s
emulation SetGlobalAdvanceImmediately true
emulation RunFor "30"
%s
''' % (executable.resolve(), data['svd'], sdcard, generated['app_base'], generated['resc'],
       '\n'.join('sysbus ReadDoubleWord 0x%X' % address for address in addresses + list(range(error, error + 256, 4)))),
       timeout=60)
    (tmp_path / 'renode.log').write_text(output)
    values = [int(value, 16) for value in re.findall(r'^0x[0-9A-Fa-f]{8}$', output, re.M)]
    assert len(values) == len(names) + 64, output
    result = dict(zip(names, values))
    message = b''.join(value.to_bytes(4, 'little') for value in values[len(names):]).split(b'\0')[0]
    assert result['result'] == 0x600df00d and result['cycles'] == 10, (result, message, output)
    assert result['before'] == result['after'], result
    assert 0 < result['heap_peak'] < 131072, result
    assert 0 < result['stack_used'] < result['stack_total'] - 4096, result
    print(result)
