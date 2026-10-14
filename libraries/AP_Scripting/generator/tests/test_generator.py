#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Behavioral regression checks and description diagnostics."""

import subprocess
import sys
import tempfile
import unittest

from pathlib import Path

GENERATOR = Path(__file__).resolve().parents[1] / 'generator.py'


class GeneratorTests(unittest.TestCase):
    def run_generator(self, description, docs=True):
        temporary = tempfile.TemporaryDirectory(prefix='scripting-generator-')
        self.addCleanup(temporary.cleanup)
        directory = Path(temporary.name)
        source = directory / 'input with spaces.desc'
        source.write_text(description, encoding='utf-8')
        output = directory / 'output with spaces'
        command = [sys.executable, str(GENERATOR), '-i', str(source), '-o', str(output)]
        if docs:
            command += ['-d', str(directory / 'docs.lua')]
        process = subprocess.run(command, capture_output=True, text=True)
        files = {}
        for suffix in ('.cpp', '.h'):
            path = Path(str(output) + suffix)
            if path.exists():
                files[suffix] = path.read_text(encoding='utf-8')
        if (directory / 'docs.lua').exists():
            files['.lua'] = (directory / 'docs.lua').read_text(encoding='utf-8')
        return process, files

    def test_crlf_tabs_and_comments(self):
        plain = 'userdata Value field x float read write LOW HIGH\n'
        _, expected = self.run_generator(plain)
        process, actual = self.run_generator('-- comment\r\n\r\n' + plain.replace(' ', '\t').replace('\n', ' -- inline\r\n'))
        self.assertEqual(process.returncode, 0, process.stderr)
        self.assertEqual(actual, expected)

    def test_no_documentation_requested(self):
        process, files = self.run_generator('userdata Value field x float read\n', docs=False)
        self.assertEqual(process.returncode, 0, process.stderr)
        self.assertEqual(set(files), {'.cpp', '.h'})
        self.assertNotIn('lua_new_Value', files['.cpp'])

    def test_stdlib_parser_does_not_hide_binding_parser(self):
        # Some Python 3.9 builds provide parser as a built-in module. Simulate
        # that module being selected even when the script directory comes first.
        script = (
            "import runpy, sys, types\n"
            "sys.modules['parser'] = types.ModuleType('parser')\n"
            "sys.path.insert(0, sys.argv[1])\n"
            "sys.argv = sys.argv[2:]\n"
            "runpy.run_path(sys.argv[0], run_name='__main__')\n"
        )
        process = subprocess.run(
            [sys.executable, '-c', script, str(GENERATOR.parent), str(GENERATOR), '--help'],
            capture_output=True, text=True,
        )
        self.assertEqual(process.returncode, 0, process.stderr)
        self.assertIn('binding description file', process.stdout)

    def test_large_nullable_result_checks_stack(self):
        arguments = ' '.join(["float'Null"] * 23)
        process, files = self.run_generator(f'userdata Value method results boolean {arguments}\n')
        self.assertEqual(process.returncode, 0, process.stderr)
        self.assertIn('#if 24 > LUA_MINSTACK\n        luaL_checkstack(L, 24, nullptr);', files['.cpp'])
        self.assertIn('lua_pushnumber(L, data_5024);', files['.cpp'])
        self.assertIn('return 23;', files['.cpp'])
        self.assertEqual(files['.lua'].count('---@return number|nil'), 23)

    def test_forward_userdata_reference_and_docs_rename(self):
        description = ('singleton Source method value Later\n'
                       'userdata Later rename renamed\n'
                       'userdata Later field x float read\n')
        process, files = self.run_generator(description)
        self.assertEqual(process.returncode, 0, process.stderr)
        self.assertIn('---@return renamed_ud\nfunction Source:value()', files['.lua'])

    def test_invalid_description_does_not_write_outputs(self):
        cases = [
            ('unknown Value', 'Expected a keyword', 3),
            ('include', 'Missing header name', 2),
            ('include file.h\ninclude file.h', 'already included', 2),
            ('include file.h invalid', 'Unsupported keyword', 2),
            ('include file.h depends', 'Missing dependency', 2),
            ('userdata Value', 'Missing access type', 4),
            ('userdata Value method f', 'Missing data type', 4),
            ('userdata Value method f void int8_t', 'Missing low value', 4),
            ('userdata Value method f void int8_t 0', 'Missing high value', 4),
            ("userdata Value method f void void", "empty argument", 4),
            ("userdata Value method f void float'unknown", 'Unknown attribute', 4),
            ("userdata Value method f void float'Null", 'only available on a boolean', 4),
            ("userdata Value method f boolean float'Ref", 'Use Nullable arguments', 4),
            ("userdata Value method f void &float", 'Only support references', 4),
            ("userdata Value method f void string'skip_check", 'cannot skip range check', 4),
            ("ap_object Pointer method f void\nuserdata Value method f void Pointer'Null", 'cannot be nullable', 4),
            ('userdata Value field x float', 'access specifier', 4),
            ('userdata Value field x float execute', 'Unknown access', 3),
            ("userdata Value field x'unknown float read", 'Unknown field attribute', 4),
            ('userdata Value field x void read', "Can't access", 5),
            ("userdata Value field x CONSTANT'literal read", "Can't access", 5),
            ('userdata Value field x float read\nuserdata Value field x float read', 'already exists', 4),
            ('userdata Value method f void\nuserdata Value method f void', 'already exists', 4),
            ('userdata Value method f void\nuserdata Value method f alias g\nuserdata Value method g void', 'alias g', 4),
            ('userdata Value operator unknown', 'Unknown operation', 4),
            ('userdata Value operator +\nuserdata Value operator +', 'already defined', 4),
            ('userdata Value operator + extra', 'Extra token', 4),
            ('userdata Value operator +', 'no fields or methods', 4),
            ('userdata Value rename', 'Missing renamed object', 7),
            ('userdata Value depends', 'Missing depends', 8),
            ('userdata Value depends A\nuserdata Value depends B', 'single depends', 7),
            ('userdata Value creation', 'Missing creation', 4),
            ('userdata Value creation cpp', 'Missing number', 7),
            ('userdata Value creation cpp 0\nuserdata Value creation cpp 0', 'single creation', 7),
            ('userdata Value operator_getter', 'Missing operator_getter', 4),
            ('userdata Value operator_getter cpp\nuserdata Value operator_getter cpp', 'single operator_getter', 4),
            ('singleton', 'Missing name', 4),
            ('singleton Value', 'Missing access type', 7),
            ('singleton Value operator +', 'unsupported directive', 7),
            ('singleton Value rename', 'Missing renamed object', 7),
            ('singleton Value rename a\nsingleton Value rename b', 'already declared', 7),
            ('singleton Value depends', 'Missing depends', 8),
            ('singleton Value depends A\nsingleton Value depends B', 'single depends', 7),
            ('singleton Value get', 'Missing get', 8),
            ('singleton Value get cpp\nsingleton Value get cpp', 'single get', 7),
            ('singleton Value method', 'Missing method name', 4),
            ('singleton Value method f', 'Missing data type', 4),
            ('ap_object', 'Missing name', 4),
            ('ap_object Value', 'Missing access type', 7),
            ('ap_object Value unknown', 'unsupported directive', 7),
            ('ap_object Value rename', 'Missing renamed object', 7),
            ('ap_object Value rename a\nap_object Value rename b', 'already declared', 7),
            ('ap_object Value depends', 'Missing depends', 8),
            ('ap_object Value depends A\nap_object Value depends B', 'single depends', 7),
            ('ap_object Value semaphore\nap_object Value semaphore-pointer', 'multiple types', 7),
            ('global', 'Missing global directive', 10),
            ('global method f', 'only support manual', 10),
            ('global manual', 'Missing Lua name', 7),
            ('global manual f', 'Missing C++ name', 7),
            ('global manual f cpp', 'Missing number of arguments', 7),
            ('global manual f cpp 0', 'Missing number of returns', 7),
            ('global manual f cpp 0 1 invalid', 'Expected depends', 7),
            ('global manual f cpp invalid 1', 'Expected an integer', 7),
            ('global manual f cpp 0 1 depends', 'Missing dependency', 4),
            ('userdata Value manual', 'Missing Lua name', 7),
            ('userdata Value manual_operator', 'Missing Lua name', 7),
            ('singleton Value literal extra', 'Extra token', 2),
            ('ap_object Value semaphore extra', 'Extra token', 2),
            ('userdata Value rename A extra', 'Extra token', 3),
            ('userdata Value method f MISSING', 'Could not find userdata', 6),
            ("userdata Value method f CONSTANT'literal", "Can't return a literal", 4),
        ]
        for description, message, code in cases:
            with self.subTest(description=description):
                process, files = self.run_generator(description)
                self.assertEqual(process.returncode, code, process.stderr)
                self.assertIn(message, process.stderr)
                self.assertNotIn('Traceback', process.stderr)
                self.assertEqual(files, {})

    def test_line_numbers(self):
        process, _ = self.run_generator('-- comment\n\nuserdata Value method f void float\n')
        self.assertIn('Error (line 3):', process.stderr)


if __name__ == '__main__':
    unittest.main()
