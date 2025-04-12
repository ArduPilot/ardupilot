#!/usr/bin/env python3

'''
Extracts parameter default values from an ArduPilot .bin log file. Unittests.

AP_FLAKE8_CLEAN

Amilcar do Carmo Lucas, IAV GmbH
'''

import unittest
from unittest.mock import patch, MagicMock
from extract_param_defaults import extract_parameter_default_values, missionplanner_sort, \
                                   mavproxy_sort, sort_params, output_params, parse_arguments, \
                                   NO_DEFAULT_VALUES_MESSAGE, MAVLINK_SYSID_MAX, MAVLINK_COMPID_MAX


class TestArgParseParameters(unittest.TestCase):
    def test_command_line_arguments_combinations(self):
        # Check the 'format' and 'sort' default parameters
        args = parse_arguments(['dummy.bin'])
        self.assertEqual(args.format, 'missionplanner')
        self.assertEqual(args.sort, 'missionplanner')

        # Check the 'format' and 'sort' parameters to see if 'sort' can be explicitly overwritten
        args = parse_arguments(['-s', 'none', 'dummy.bin'])
        self.assertEqual(args.format, 'missionplanner')
        self.assertEqual(args.sort, 'none')

        # Check the 'format' and 'sort' parameters to see if 'sort' can be implicitly overwritten (mavproxy)
        args = parse_arguments(['-f', 'mavproxy', 'dummy.bin'])
        self.assertEqual(args.format, 'mavproxy')
        self.assertEqual(args.sort, 'mavproxy')

        # Check the 'format' and 'sort' parameters to see if 'sort' can be implicitly overwritten (qgcs)
        args = parse_arguments(['-f', 'qgcs', 'dummy.bin'])
        self.assertEqual(args.format, 'qgcs')
        self.assertEqual(args.sort, 'qgcs')

        # Check the 'format' and 'sort' parameters
        args = parse_arguments(['-f', 'mavproxy', '-s', 'none', 'dummy.bin'])
        self.assertEqual(args.format, 'mavproxy')
        self.assertEqual(args.sort, 'none')

        # Assert that a SystemExit is raised when --sysid is used without --format set to qgcs
        with self.assertRaises(SystemExit):
            with patch('builtins.print') as mock_print:
                parse_arguments(['-f', 'mavproxy', '-i', '7', 'dummy.bin'])
                mock_print.assert_called_once_with("--sysid parameter is only relevant if --format is qgcs")

        # Assert that a SystemExit is raised when --compid is used without --format set to qgcs
        with self.assertRaises(SystemExit):
            with patch('builtins.print') as mock_print:
                parse_arguments(['-f', 'missionplanner', '-c', '3', 'dummy.bin'])
                mock_print.assert_called_once_with("--compid parameter is only relevant if --format is qgcs")

        # Assert that a valid sysid and compid are parsed correctly
        args = parse_arguments(['-f', 'qgcs', '-i', '7', '-c', '3', 'dummy.bin'])
        self.assertEqual(args.format, 'qgcs')
        self.assertEqual(args.sort, 'qgcs')
        self.assertEqual(args.sysid, 7)
        self.assertEqual(args.compid, 3)


class TestExtractParameterDefaultValues(unittest.TestCase):

    @patch('extract_param_defaults.mavutil.mavlink_connection')
    def test_logfile_does_not_exist(self, mock_mavlink_connection):
        # Mock the mavlink connection to raise an exception
        mock_mavlink_connection.side_effect = Exception("Test exception")

        # Call the function with a dummy logfile path
        with self.assertRaises(SystemExit) as cm:
            extract_parameter_default_values('dummy.bin')

        # Check the error message
        self.assertEqual(str(cm.exception), "Error opening the dummy.bin logfile: Test exception")

    @patch('extract_param_defaults.mavutil.mavlink_connection')
    def test_extract_parameter_default_values(self, mock_mavlink_connection):
        # Mock the mavlink connection and the messages it returns
        mock_mlog = MagicMock()
        mock_mavlink_connection.return_value = mock_mlog
        mock_mlog.recv_match.side_effect = [
            MagicMock(Name='PARAM1', Default=1.1),
            MagicMock(Name='PARAM2', Default=2.0),
            None  # End of messages
        ]

        # Call the function with a dummy logfile path
        defaults = extract_parameter_default_values('dummy.bin')

        # Check if the defaults dictionary contains the correct parameters and values
        self.assertEqual(defaults, {'PARAM1': 1.1, 'PARAM2': 2.0})

    @patch('extract_param_defaults.mavutil.mavlink_connection')
    def test_no_parameters(self, mock_mavlink_connection):
        # Mock the mavlink connection to return no parameter messages
        mock_mlog = MagicMock()
        mock_mavlink_connection.return_value = mock_mlog
        mock_mlog.recv_match.return_value = None  # No PARM messages

        # Call the function with a dummy logfile path and assert SystemExit is raised with the correct message
        with self.assertRaises(SystemExit) as cm:
            extract_parameter_default_values('dummy.bin')
        self.assertEqual(str(cm.exception), NO_DEFAULT_VALUES_MESSAGE)

    @patch('extract_param_defaults.mavutil.mavlink_connection')
    def test_no_parameter_defaults(self, mock_mavlink_connection):
        # Mock the mavlink connection to simulate no parameter default values in the .bin file
        mock_mlog = MagicMock()
        mock_mavlink_connection.return_value = mock_mlog
        mock_mlog.recv_match.return_value = None  # No PARM messages

        # Call the function with a dummy logfile path and assert SystemExit is raised with the correct message
        with self.assertRaises(SystemExit) as cm:
            extract_parameter_default_values('dummy.bin')
        self.assertEqual(str(cm.exception), NO_DEFAULT_VALUES_MESSAGE)

    @patch('extract_param_defaults.mavutil.mavlink_connection')
    def test_invalid_parameter_name(self, mock_mavlink_connection):
        # Mock the mavlink connection to simulate an invalid parameter name
        mock_mlog = MagicMock()
        mock_mavlink_connection.return_value = mock_mlog
        mock_mlog.recv_match.return_value = MagicMock(Name='INVALID_NAME%', Default=1.0)

        # Call the function with a dummy logfile path
        with self.assertRaises(SystemExit):
            extract_parameter_default_values('dummy.bin')

    @patch('extract_param_defaults.mavutil.mavlink_connection')
    def test_long_parameter_name(self, mock_mavlink_connection):
        # Mock the mavlink connection to simulate a too long parameter name
        mock_mlog = MagicMock()
        mock_mavlink_connection.return_value = mock_mlog
        mock_mlog.recv_match.return_value = MagicMock(Name='TOO_LONG_PARAMETER_NAME', Default=1.0)

        # Call the function with a dummy logfile path
        with self.assertRaises(SystemExit):
            extract_parameter_default_values('dummy.bin')


class TestSortFunctions(unittest.TestCase):
    def test_missionplanner_sort(self):
        # Define a list of parameter names
        params = ['PARAM_GROUP1_PARAM1', 'PARAM_GROUP2_PARAM2', 'PARAM_GROUP1_PARAM2']

        # Sort the parameters using the missionplanner_sort function
        sorted_params = sorted(params, key=missionplanner_sort)

        # Check if the parameters were sorted correctly
        self.assertEqual(sorted_params, ['PARAM_GROUP1_PARAM1', 'PARAM_GROUP1_PARAM2', 'PARAM_GROUP2_PARAM2'])

        # Test with a parameter name that doesn't contain an underscore
        params = ['PARAM1', 'PARAM3', 'PARAM2']
        sorted_params = sorted(params, key=missionplanner_sort)
        self.assertEqual(sorted_params, ['PARAM1', 'PARAM2', 'PARAM3'])

    def test_mavproxy_sort(self):
        # Define a list of parameter names
        params = ['PARAM_GROUP1_PARAM1', 'PARAM_GROUP2_PARAM2', 'PARAM_GROUP1_PARAM2']

        # Sort the parameters using the mavproxy_sort function
        sorted_params = sorted(params, key=mavproxy_sort)

        # Check if the parameters were sorted correctly
        self.assertEqual(sorted_params, ['PARAM_GROUP1_PARAM1', 'PARAM_GROUP1_PARAM2', 'PARAM_GROUP2_PARAM2'])

        # Test with a parameter name that doesn't contain an underscore
        params = ['PARAM1', 'PARAM3', 'PARAM2']
        sorted_params = sorted(params, key=mavproxy_sort)
        self.assertEqual(sorted_params, ['PARAM1', 'PARAM2', 'PARAM3'])


class TestOutputParams(unittest.TestCase):

    @patch('extract_param_defaults.print')
    def test_output_params(self, mock_print):
        # Prepare a dummy defaults dictionary
        defaults = {'PARAM2': 1.0, 'PARAM1': 2.0}

        # Call the function with the dummy dictionary, 'missionplanner' format type
        output_params(defaults, 'missionplanner')

        # Check if the print function was called with the correct parameters
        expected_calls = [unittest.mock.call('PARAM2,1'), unittest.mock.call('PARAM1,2')]
        mock_print.assert_has_calls(expected_calls, any_order=False)

    @patch('extract_param_defaults.print')
    def test_output_params_missionplanner_non_numeric(self, mock_print):
        # Prepare a dummy defaults dictionary
        defaults = {'PARAM1': 'non-numeric'}

        # Call the function with the dummy dictionary, 'missionplanner' format type
        output_params(defaults, 'missionplanner')

        # Check if the print function was called with the correct parameters
        expected_calls = [unittest.mock.call('PARAM1,non-numeric')]
        mock_print.assert_has_calls(expected_calls, any_order=False)

    @patch('extract_param_defaults.print')
    def test_output_params_mavproxy(self, mock_print):
        # Prepare a dummy defaults dictionary
        defaults = {'PARAM2': 2.0, 'PARAM1': 1.0}

        # Call the function with the dummy dictionary, 'mavproxy' format type and 'mavproxy' sort type
        defaults = sort_params(defaults, 'mavproxy')
        output_params(defaults, 'mavproxy')

        # Check if the print function was called with the correct parameters
        expected_calls = [unittest.mock.call("%-15s %.6f" % ('PARAM1', 1.0)),
                          unittest.mock.call("%-15s %.6f" % ('PARAM2', 2.0))]
        mock_print.assert_has_calls(expected_calls, any_order=False)

    @patch('extract_param_defaults.print')
    def test_output_params_qgcs(self, mock_print):
        # Prepare a dummy defaults dictionary
        defaults = {'PARAM2': 2.0, 'PARAM1': 1.0}

        # Call the function with the dummy dictionary, 'qgcs' format type and 'qgcs' sort type
        defaults = sort_params(defaults, 'qgcs')
        output_params(defaults, 'qgcs')

        # Check if the print function was called with the correct parameters
        expected_calls = [unittest.mock.call("\n# # Vehicle-Id Component-Id Name Value Type\n"),
                          unittest.mock.call("%u %u %-15s %.6f %u" % (1, 1, 'PARAM1', 1.0, 9)),
                          unittest.mock.call("%u %u %-15s %.6f %u" % (1, 1, 'PARAM2', 2.0, 9))]
        mock_print.assert_has_calls(expected_calls, any_order=False)

    @patch('extract_param_defaults.print')
    def test_output_params_qgcs_2_4(self, mock_print):
        # Prepare a dummy defaults dictionary
        defaults = {'PARAM2': 2.0, 'PARAM1': 1.0}

        # Call the function with the dummy dictionary, 'qgcs' format type and 'qgcs' sort type
        defaults = sort_params(defaults, 'qgcs')
        output_params(defaults, 'qgcs', 2, 4)

        # Check if the print function was called with the correct parameters
        expected_calls = [unittest.mock.call("\n# # Vehicle-Id Component-Id Name Value Type\n"),
                          unittest.mock.call("%u %u %-15s %.6f %u" % (2, 4, 'PARAM1', 1.0, 9)),
                          unittest.mock.call("%u %u %-15s %.6f %u" % (2, 4, 'PARAM2', 2.0, 9))]
        mock_print.assert_has_calls(expected_calls, any_order=False)

    @patch('extract_param_defaults.print')
    def test_output_params_qgcs_MAV_SYSID(self, mock_print):
        # Prepare a dummy defaults dictionary
        defaults = {'PARAM2': 2.0, 'PARAM1': 1.0, 'MAV_SYSID': 3.0}

        # Call the function with the dummy dictionary, 'qgcs' format type and 'qgcs' sort type
        defaults = sort_params(defaults, 'qgcs')
        output_params(defaults, 'qgcs', -1, 7)

        # Check if the print function was called with the correct parameters
        expected_calls = [
            unittest.mock.call("\n# # Vehicle-Id Component-Id Name Value Type\n"),
            unittest.mock.call("%u %u %-15s %.6f %u" % (3, 7, 'MAV_SYSID', 3.0, 9)),
            unittest.mock.call("%u %u %-15s %.6f %u" % (3, 7, 'PARAM1', 1.0, 9)),
            unittest.mock.call("%u %u %-15s %.6f %u" % (3, 7, 'PARAM2', 2.0, 9)),
        ]
        mock_print.assert_has_calls(expected_calls, any_order=False)

    @patch('extract_param_defaults.print')
    def test_output_params_qgcs_SYSID_INVALID(self, mock_print):
        # Prepare a dummy defaults dictionary
        defaults = {'PARAM2': 2.0, 'PARAM1': 1.0, 'MAV_SYSID': -1.0}

        # Assert that a SystemExit is raised with the correct message when an invalid sysid is used
        with self.assertRaises(SystemExit) as cm:
            defaults = sort_params(defaults, 'qgcs')
            output_params(defaults, 'qgcs', -1, 7)
        self.assertEqual(str(cm.exception), "Invalid system ID parameter -1 must not be negative")

        # Assert that a SystemExit is raised with the correct message when an invalid sysid is used
        with self.assertRaises(SystemExit) as cm:
            defaults = sort_params(defaults, 'qgcs')
            output_params(defaults, 'qgcs', MAVLINK_SYSID_MAX+2, 7)
        self.assertEqual(str(cm.exception), f"Invalid system ID parameter 16777218 must be smaller than {MAVLINK_SYSID_MAX}")

    @patch('extract_param_defaults.print')
    def test_output_params_qgcs_COMPID_INVALID(self, mock_print):
        # Prepare a dummy defaults dictionary
        defaults = {'PARAM2': 2.0, 'PARAM1': 1.0}

        # Assert that a SystemExit is raised with the correct message when an invalid compid is used
        with self.assertRaises(SystemExit) as cm:
            defaults = sort_params(defaults, 'qgcs')
            output_params(defaults, 'qgcs', -1, -3)
        self.assertEqual(str(cm.exception), "Invalid component ID parameter -3 must not be negative")

        # Assert that a SystemExit is raised with the correct message when an invalid compid is used
        with self.assertRaises(SystemExit) as cm:
            defaults = sort_params(defaults, 'qgcs')
            output_params(defaults, 'qgcs', 1, MAVLINK_COMPID_MAX+3)
        self.assertEqual(str(cm.exception), f"Invalid component ID parameter 259 must be smaller than {MAVLINK_COMPID_MAX}")

    @patch('extract_param_defaults.print')
    def test_output_params_integer(self, mock_print):
        # Prepare a dummy defaults dictionary with an integer value
        defaults = {'PARAM1': 1.01, 'PARAM2': 2.00}

        # Call the function with the dummy dictionary, 'missionplanner' format type and 'missionplanner' sort type
        defaults = sort_params(defaults, 'missionplanner')
        output_params(defaults, 'missionplanner')

        # Check if the print function was called with the correct parameters
        expected_calls = [unittest.mock.call('PARAM1,1.01'), unittest.mock.call('PARAM2,2')]
        mock_print.assert_has_calls(expected_calls, any_order=False)


if __name__ == '__main__':
    unittest.main()
