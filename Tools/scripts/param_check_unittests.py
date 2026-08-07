#!/usr/bin/env python3

"""
Parameter File Checker Unit Tests

AP_FLAKE8_CLEAN
"""

from __future__ import annotations
import time
import unittest
import subprocess
from unittest.mock import MagicMock, patch, mock_open
from param_check import (
    SkippedChecks,
    load_params,
    check_param,
    check_range,
    check_values,
    check_bitmask,
    generate_metadata,
    get_metadata,
    check_file,
    parse_arguments,
    main
)


class TestParamCheck(unittest.TestCase):

    def test_load_params(self):
        # Mock parameter file content
        base_mock_content = '''
        PARAM1, 1
        PARAM2\t0x10 @READONLY # Comment
        # Comment
        @tag
        @include filename.parm
        PARAM3   42.5#comment
        PARAM4, 0x0F #DISABLE_CHECKS for some reason
        '''
        mock_file_content = base_mock_content
        mock_included_content = '''
        INCLUDED1, 123
        INCLUDED2, 456
        '''

        def file_side_effect(file, *args, **kwargs):
            if file == 'fake_file.parm':
                return mock_open(read_data=mock_file_content)()
            if file == 'filename.parm':
                return mock_open(read_data=mock_included_content)()

        with patch('builtins.open', side_effect=file_side_effect):
            params, msgs = load_params('fake_file.parm')

        # Test the correct parsing of parameters and DISABLE_CHECKS flag
        self.assertEqual(params['PARAM1'], (1.0, False))
        self.assertEqual(params['PARAM2'], (16.0, False))
        self.assertEqual(params['PARAM3'], (42.5, False))
        self.assertEqual(params['PARAM4'], (15.0, True))
        self.assertEqual(msgs, [])

        # Bad parameter lines that should fail to parse and exit the program
        bad_lines = [
            'PARAM5 1 2\n',
            'PARAM5\n',
            'P#ARAM5 1\n',
            'PARAM5, 0x10.0\n',
        ]
        for line in bad_lines:
            mock_file_content = base_mock_content + line
            with patch('builtins.open', side_effect=file_side_effect):
                _, msgs = load_params('fake_file.parm')
            self.assertEqual(msgs, [f'Error parsing line: `{line.strip()}`'])

        # Test that the program exits when DISABLE_CHECKS does not have a reason
        mock_file_content = base_mock_content + 'PARAM4, 0x10 #DISABLE_CHECKS: \n'
        with patch('builtins.open', side_effect=file_side_effect):
            _, msgs = load_params('fake_file.parm')
        self.assertEqual(msgs, ['Explanation required for disabled checks: `PARAM4, 0x10 #DISABLE_CHECKS:`'])

        # Test that self-includes raise an error for infinite recursion
        mock_file_content = base_mock_content + '@include fake_file.parm\n'
        with patch('builtins.open', side_effect=file_side_effect):
            with self.assertRaises(ValueError) as cm:
                load_params('fake_file.parm')
            self.assertEqual(str(cm.exception), 'Too many levels of @include')

    def test_check_range(self):
        # Mock metadata and parameters
        metadata = {'low': 0.0, 'high': 100.0}
        self.assertIsNone(check_range('PARAM', 50.0, metadata))
        self.assertEqual(check_range('PARAM', -1.0, metadata), 'PARAM: -1 is below minimum value 0.0')
        self.assertEqual(check_range('PARAM', 101.0, metadata), 'PARAM: 101 is above maximum value 100.0')

    def test_check_values(self):
        # Mock metadata and parameters
        metadata = {'0': 'Off', '1': 'On'}
        self.assertIsNone(check_values('PARAM', 0, metadata))
        self.assertEqual(check_values('PARAM', 2, metadata), 'PARAM: 2 is not a valid value')

    def test_check_bitmask(self):
        # Mock metadata and parameters
        metadata = {'0': 'Bit0', '1': 'Bit1'}
        self.assertIsNone(check_bitmask('PARAM', 3.0, metadata))
        self.assertEqual(check_bitmask('PARAM', 4.0, metadata), 'PARAM: bit 2 is not valid')
        self.assertEqual(check_bitmask('PARAM', 1.5, metadata), 'PARAM: 1.5 is not an integer')

    @patch('subprocess.run')
    @patch('os.remove')
    @patch('os.path.getmtime')
    @patch('os.path.exists', return_value=True)
    @patch('builtins.open', new_callable=mock_open, read_data='{}')
    @patch('json.load', return_value={'GROUP1': {'PARAM1': {}}, 'GROUP2': {'PARAM2': {}}, 'json': {'version': 0}})
    @patch('builtins.print')
    def test_generate_metadata(self, mock_print, mock_json, mock_open, mock_exists, mock_getmtime, mock_remove, mock_run):
        # When the function calls getmtime, it will return the current time
        mock_getmtime.side_effect = lambda path: time.time()

        # Check the arguments passed to param_parse.py
        for vehicle in ['Plane', 'Copter']:
            generate_metadata(vehicle)
            call_args, _ = mock_run.call_args
            arglist = call_args[0]
            self.assertIn(f'--vehicle={vehicle}', arglist)
            self.assertIn('--no-legacy-params', arglist)

        # Test that the metadata was loaded and that the json was flattened
        # (and that the json version was stripped out)
        metadata = generate_metadata('Plane')
        self.assertEqual(metadata, {'PARAM1': {}, 'PARAM2': {}})

        # Test that we raise a runtime error if the metadata file exists but is
        # not updated
        now = time.time()
        mock_getmtime.side_effect = lambda path: now
        with self.assertRaises(RuntimeError) as cm:
            generate_metadata('Plane')
        self.assertEqual('Error: Metadata file "apm.pdef.json" was not updated.', str(cm.exception))

        # Test that we raise a runtime error if the metadata file does not
        # exist after generation
        mock_exists.return_value = False
        with self.assertRaises(RuntimeError) as cm:
            generate_metadata('Plane')
        self.assertEqual('Error: Metadata file "apm.pdef.json" was not created.', str(cm.exception))

        # Test that we raise a runtime error if CalledProcessError is raised
        mock_run.side_effect = subprocess.CalledProcessError(
            returncode=1,
            cmd='python3',
            output='Error running script',
            stderr='Error details'
        )
        with self.assertRaises(RuntimeError) as cm:
            generate_metadata('Plane')

        self.assertIn('Error generating metadata for vehicle Plane', str(cm.exception))

    @patch('param_check.generate_metadata')
    def test_get_metadata_merges(self, mock_generate_metadata):
        mock_generate_metadata.side_effect = [
            {
                'PARAM1': {
                    'Description': 'plane PARAM1',
                    'Bitmask': {'0': 'bit0', '1': 'bit1'},
                },
                'PARAM2': {
                    'Description': 'plane PARAM3',
                    'Range': {'low': '-1', 'high': '200'},
                    'Values': {'0': 'Off', '1': 'On'},
                },
                'PARAM3': {'Description': 'plane PARAM2'},
            },
            {
                'PARAM1': {
                    'Description': 'copter PARAM1',
                    'Bitmask': {'0': 'bit0copter', '2': 'bit2'},
                },
                'PARAM2': {
                    'desc': 'copter3',
                    'Range': {'low': '-100', 'high': '10'},
                    'Values': {'0': 'Off', '1': 'On', '50': 'SoTotallyOn'},
                },
                'PARAM4': {'Description': 'copter PARAM4'},
            },
        ]
        vehicles = ['Plane', 'Copter']
        merged = get_metadata(vehicles)

        # Matching field content gets overwritten by the last vehicle's
        # metadata, but extra fields get added (we need to make sure values and
        # bitmasks cover all valid values for all vehicles; we don't actually
        # care about the content)

        # Check that all keys are present in PARAM1's bitmask
        self.assertIn('0', merged['PARAM1']['Bitmask'])
        self.assertIn('1', merged['PARAM1']['Bitmask'])
        self.assertIn('2', merged['PARAM1']['Bitmask'])

        # Check that all keys are present in PARAM2's values
        self.assertIn('0', merged['PARAM2']['Values'])
        self.assertIn('1', merged['PARAM2']['Values'])
        self.assertIn('50', merged['PARAM2']['Values'])

        # Check that PARAM3 and PARAM4 are present
        self.assertIn('PARAM3', merged)
        self.assertIn('PARAM4', merged)

        # Check that all params still have a non-empty description
        # (just to ensure we're not dropping other fields somehow)
        for param in ['PARAM1', 'PARAM2', 'PARAM3', 'PARAM4']:
            self.assertTrue(merged[param]['Description'])

    def test_check_param(self):
        metadata = {
            'Description': 'This is a test parameter',
            'ReadOnly': True,
            'Bitmask': {'0': 'Bit0', '1': 'Bit1', '8': 'Bit8'},
            'Range': {'low': '0.0', 'high': '100.0'},
            'Values': {'0': 'Off', '1': 'On'}
        }
        skip = SkippedChecks()

        # Test ReadOnly
        self.assertEqual(check_param('PARAM', 0, metadata, skip), 'PARAM is read only')
        skip.no_readonly = True
        self.assertIsNone(check_param('PARAM', 0, metadata, skip))

        # Test Bitmask
        del metadata['ReadOnly']  # Remove ReadOnly to test the next priority
        self.assertIsNone(check_param('PARAM', 256.0, metadata, skip))  # 256 would fail the other checks, but pass bitmask
        self.assertEqual(check_param('PARAM', 1.5, metadata, skip), 'PARAM: 1.5 is not an integer')
        self.assertEqual(check_param('PARAM', -1, metadata, skip), 'PARAM: -1 is negative')
        self.assertEqual(
            check_param('PARAM', 18446744073709551616, metadata, skip),
            'PARAM: 18446744073709551616 is larger than 64 bits'
        )
        self.assertEqual(check_param('PARAM', 4, metadata, skip), 'PARAM: bit 2 is not valid')
        skip.no_bitmask = True
        self.assertIsNone(check_param('PARAM', 4, metadata, skip))

        # Test Range
        del metadata['Bitmask']  # Remove Bitmask to test the next priority
        self.assertIsNone(check_param('PARAM', 50, metadata, skip))  # 50 will fail the values check, but pass the range check
        self.assertEqual(check_param('PARAM', 101, metadata, skip), 'PARAM: 101 is above maximum value 100.0')
        self.assertEqual(check_param('PARAM', -1, metadata, skip), 'PARAM: -1 is below minimum value 0.0')
        skip.no_range = True
        self.assertIsNone(check_param('PARAM', -1, metadata, skip))

        # Test Values
        del metadata['Range']  # Remove Range to test the next priority
        self.assertIsNone(check_param('PARAM', 0, metadata, skip))
        self.assertEqual(check_param('PARAM', 2, metadata, skip), 'PARAM: 2 is not a valid value')
        skip.no_values = True
        self.assertIsNone(check_param('PARAM', 2, metadata, skip))

        # Test parameter with no range, bitmask, or value restrictions
        del metadata['Values']
        self.assertIsNone(check_param('PARAM', 0, metadata, skip))  # Should pass no matter what

    @patch('param_check.check_param')
    def test_check_file(self, mock_check_param):
        mock_skip = SkippedChecks()

        # Case 1: All parameters pass their checks
        mock_file_content = "PARAM1, 10\nPARAM2, 20\n"
        mock_metadata = {'PARAM1': {}, 'PARAM2': {}}
        # Mock check_param to return None for all params, indicating they pass validation
        mock_check_param.side_effect = lambda name, value, metadata, skip: None
        with patch('builtins.open', mock_open(read_data=mock_file_content)):
            msgs = check_file('fake_file.parm', mock_metadata, mock_skip)
        # Check that no error messages were returned
        self.assertEqual(msgs, [])
        # Check that check_param was called for each parameter
        self.assertEqual(mock_check_param.call_count, 2)

        # Case 2: Missing parameter (PARAM3 not in metadata)
        mock_file_content += "PARAM3, 30\n"
        with patch('builtins.open', mock_open(read_data=mock_file_content)):
            msgs = check_file('fake_file.parm', mock_metadata, mock_skip)
        # Check that a missing parameter error is reported
        self.assertEqual(msgs, ['PARAM3 not found in metadata'])

        # Case 3: Missing parameter but with no-missing flag
        mock_skip.no_missing = True
        with patch('builtins.open', mock_open(read_data=mock_file_content)):
            msgs = check_file('fake_file.parm', mock_metadata, mock_skip)
        # Check that no error messages are returned when no-missing is enabled
        self.assertEqual(msgs, [])

        # Case 4: Valid parameter with DISABLE_CHECKS flag, and invalid parameter
        # (should report both errors)
        mock_file_content = "PARAM1, 50.0 # DISABLE_CHECKS: reason\nPARAM2, 0\n"
        # Mock check_param so PARAM1 is valid, but not PARAM2
        mock_check_param.side_effect = lambda name, value, metadata, skip: (
            None if name in ['PARAM1'] else f'{name}: Error'
        )
        with patch('builtins.open', mock_open(read_data=mock_file_content)):
            msgs = check_file('fake_file.parm', mock_metadata, mock_skip)
        self.assertEqual(
            msgs,
            [
                'PARAM1 does not need DISABLE_CHECKS',
                'PARAM2: Error',
            ],
        )

        # Case 5: Invalid parameter but with DISABLE_CHECKS (should pass)
        mock_file_content = "PARAM1, 150.0\nPARAM2, 200.0 # DISABLE_CHECKS: reason\n"
        with patch('builtins.open', mock_open(read_data=mock_file_content)):
            msgs = check_file('fake_file.parm', mock_metadata, mock_skip)
        # Check that no error messages are returned because DISABLE_CHECKS is valid
        self.assertEqual(msgs, [])

        # Case 6: Redefined parameter
        mock_file_content = "PARAM1, 10\nPARAM1, 20\n"
        with patch('builtins.open', mock_open(read_data=mock_file_content)):
            msgs = check_file('fake_file.parm', mock_metadata, mock_skip)
        # Check that a redefined parameter error is reported
        self.assertEqual(msgs, ['PARAM1 redefined'])

        # Case 7: Redefined parameter but with no-redefinition flag
        mock_skip.no_redefinition = True
        with patch('builtins.open', mock_open(read_data=mock_file_content)):
            msgs = check_file('fake_file.parm', mock_metadata, mock_skip)
        # Check that no error messages are returned when no-redefinition is enabled
        self.assertEqual(msgs, [])

    @patch('param_check.glob.glob')
    @patch('sys.argv', ['param_check.py', 'file1.parm', 'file2.parm'])
    def test_parse_arguments_defaults(self, mock_glob):
        # Simulate glob returning the same file names
        mock_glob.side_effect = lambda pattern, recursive=True: [pattern]
        args = parse_arguments()
        self.assertEqual(args.files, ['file1.parm', 'file2.parm'])
        # Should default to all vehicles
        self.assertEqual(args.vehicle, 'Sub,Plane,Blimp,Copter,Tracker,Rover,AP_Periph')
        self.assertFalse(args.quiet_success)

    @patch('param_check.glob.glob')
    @patch(
        'sys.argv',
        [
            'param_check.py', 'file*.parm',
            '--vehicle=Plane,Copter',
            '--quiet-success', '--no-missing', '--no-redefinition', '--no-readonly',
            '--no-bitmask', '--no-range', '--no-values',
        ],
    )
    def test_parse_arguments_all_flags(self, mock_glob):
        # Simulate glob expanding file*.parm to two files
        mock_glob.side_effect = lambda pattern, recursive=True: ['file1.parm', 'file2.parm'] if pattern == 'file*.parm' else []
        args = parse_arguments()
        self.assertEqual(args.files, ['file1.parm', 'file2.parm'])
        self.assertEqual(args.vehicle, 'Plane,Copter')
        self.assertTrue(args.quiet_success)
        self.assertTrue(args.no_missing)
        self.assertTrue(args.no_redefinition)
        self.assertTrue(args.no_readonly)
        self.assertTrue(args.no_bitmask)
        self.assertTrue(args.no_range)
        self.assertTrue(args.no_values)

    @patch('param_check.parse_arguments')
    @patch('param_check.generate_metadata')
    @patch('param_check.check_file')
    @patch('builtins.print')
    def test_main(self, mock_print, mock_check_file, mock_generate_metadata, mock_parse_arguments):

        # Setup mock for parse_arguments
        mock_args = MagicMock()
        mock_args.vehicle = 'Plane'
        mock_args.files = ['file1.parm', 'file2.parm']
        mock_args.quiet_success = False
        mock_parse_arguments.return_value = mock_args

        # Setup mock for generate_metadata
        mock_generate_metadata.return_value = {'PARAM1': {}, 'PARAM2': {}}

        # Setup mock for check_file
        mock_check_file.side_effect = lambda file, metadata, args: [] if file == 'file1.parm' else ['Error']

        # Call main function
        with patch('sys.argv', ['param_check.py']):
            with self.assertRaises(SystemExit):
                main()

        # Check that generate_metadata was called correctly
        mock_generate_metadata.assert_called_once_with('Plane')

        # Check that check_file was called for each expanded file
        self.assertEqual(mock_check_file.call_count, 2)
        mock_check_file.assert_any_call('file1.parm', {'PARAM1': {}, 'PARAM2': {}}, unittest.mock.ANY)
        mock_check_file.assert_any_call('file2.parm', {'PARAM1': {}, 'PARAM2': {}}, unittest.mock.ANY)

        # Check that print was called correctly
        mock_print.assert_any_call('file1.parm: Passed')
        mock_print.assert_any_call('file2.parm: Failed')
        mock_print.assert_any_call('  Error')

        # Test that the program exits when no files are provided
        mock_print.reset_mock()
        mock_args.files = []
        mock_parse_arguments.return_value = mock_args
        with patch('sys.argv', ['param_check.py']):
            with self.assertRaises(SystemExit):
                main()

        mock_print.assert_called_once_with('Error: No parameter files specified.')


if __name__ == '__main__':
    unittest.main()
