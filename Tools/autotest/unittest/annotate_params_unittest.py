#!/usr/bin/env python3

'''
These are the unit tests for the python script that fetches online ArduPilot
parameter documentation (if not cached) and adds it to the specified file or
to all *.param and *.parm files in the specified directory.

AP_FLAKE8_CLEAN

Author: Amilcar do Carmo Lucas, IAV GmbH
'''

import tempfile
from unittest.mock import patch, mock_open
import os
import unittest
import xml.etree.ElementTree as ET
import requests
import mock
from annotate_params import get_xml_data, remove_prefix, split_into_lines, create_doc_dict, \
                            format_columns, update_parameter_documentation, print_read_only_params, \
                            BASE_URL, PARAM_DEFINITION_XML_FILE


class TestParamDocsUpdate(unittest.TestCase):

    def setUp(self):
        # Create a temporary directory
        self.temp_dir = tempfile.mkdtemp()

        # Create a temporary file
        self.temp_file = tempfile.NamedTemporaryFile(delete=False)

        # Create a dictionary of parameter documentation
        self.doc_dict = {
            "PARAM1": {
                "humanName": "Param 1",
                "documentation": ["Documentation for Param 1"],
                "fields": {"Field1": "Value1", "Field2": "Value2"},
                "values": {"Code1": "Value1", "Code2": "Value2"}
            },
            "PARAM2": {
                "humanName": "Param 2",
                "documentation": ["Documentation for Param 2"],
                "fields": {"Field3": "Value3", "Field4": "Value4"},
                "values": {"Code3": "Value3", "Code4": "Value4"}
            },
            "PARAM_1": {
                "humanName": "Param _ 1",
                "documentation": ["Documentation for Param_1"],
                "fields": {"Field_1": "Value_1", "Field_2": "Value_2"},
                "values": {"Code_1": "Value_1", "Code_2": "Value_2"}
            },
        }

    @patch('builtins.open', new_callable=mock_open, read_data='<root></root>')
    @patch('os.path.isfile')
    def test_get_xml_data_local_file(self, mock_isfile, mock_open):
        # Mock the isfile function to return True
        mock_isfile.return_value = True

        # Call the function with a local file
        result = get_xml_data("/path/to/local/file/", ".", "test.xml")

        # Check the result
        self.assertIsInstance(result, ET.Element)

        # Assert that the file was opened correctly
        mock_open.assert_called_once_with('./test.xml', 'r', encoding='utf-8')

    @patch('requests.get')
    def test_get_xml_data_remote_file(self, mock_get):
        # Mock the response
        mock_get.return_value.status_code = 200
        mock_get.return_value.text = "<root></root>"

        # Remove the test.xml file if it exists
        try:
            os.remove("test.xml")
        except FileNotFoundError:
            pass

        # Call the function with a remote file
        result = get_xml_data("http://example.com/", ".", "test.xml")

        # Check the result
        self.assertIsInstance(result, ET.Element)

        # Assert that the requests.get function was called once
        mock_get.assert_called_once_with("http://example.com/test.xml", timeout=5)

    @patch('os.path.isfile')
    def test_get_xml_data_script_dir_file(self, mock_isfile):
        # Mock the isfile function to return False for the current directory and True for the script directory
        def side_effect(filename):
            return True
        mock_isfile.side_effect = side_effect

        # Mock the open function to return a dummy XML string
        mock_open = mock.mock_open(read_data='<root></root>')
        with patch('builtins.open', mock_open):
            # Call the function with a filename that exists in the script directory
            result = get_xml_data(BASE_URL, ".", PARAM_DEFINITION_XML_FILE)

        # Check the result
        self.assertIsInstance(result, ET.Element)

        # Assert that the file was opened correctly
        mock_open.assert_called_once_with(os.path.join('.', PARAM_DEFINITION_XML_FILE), 'r', encoding='utf-8')

    def test_get_xml_data_no_requests_package(self):
        # Temporarily remove the requests module
        with patch.dict('sys.modules', {'requests': None}):

            # Remove the test.xml file if it exists
            try:
                os.remove("test.xml")
            except FileNotFoundError:
                pass

            # Call the function with a remote file
            with self.assertRaises(SystemExit):
                get_xml_data("http://example.com/", ".", "test.xml")

    @patch('requests.get')
    def test_get_xml_data_request_failure(self, mock_get):
        # Mock the response
        mock_get.side_effect = requests.exceptions.RequestException

        # Remove the test.xml file if it exists
        try:
            os.remove("test.xml")
        except FileNotFoundError:
            pass

        # Call the function with a remote file
        with self.assertRaises(SystemExit):
            get_xml_data("http://example.com/", ".", "test.xml")

    @patch('requests.get')
    def test_get_xml_data_valid_xml(self, mock_get):
        # Mock the response
        mock_get.return_value.status_code = 200
        mock_get.return_value.text = "<root></root>"

        # Call the function with a remote file
        result = get_xml_data("http://example.com/", ".", "test.xml")

        # Check the result
        self.assertIsInstance(result, ET.Element)

    @patch('requests.get')
    def test_get_xml_data_invalid_xml(self, mock_get):
        # Mock the response
        mock_get.return_value.status_code = 200
        mock_get.return_value.text = "<root><invalid></root>"

        # Remove the test.xml file if it exists
        try:
            os.remove("test.xml")
        except FileNotFoundError:
            pass

        # Call the function with a remote file
        with self.assertRaises(ET.ParseError):
            get_xml_data("http://example.com/", ".", "test.xml")

    @patch('requests.get')
    @patch('os.path.isfile')
    def test_get_xml_data_missing_file(self, mock_isfile, mock_get):
        # Mock the isfile function to return False
        mock_isfile.return_value = False
        # Mock the requests.get call to raise FileNotFoundError
        mock_get.side_effect = FileNotFoundError

        # Remove the test.xml file if it exists
        try:
            os.remove("test.xml")
        except FileNotFoundError:
            pass

        # Call the function with a local file
        with self.assertRaises(FileNotFoundError):
            get_xml_data("/path/to/local/file/", ".", "test.xml")

    @patch('requests.get')
    def test_get_xml_data_network_issue(self, mock_get):
        # Mock the response
        mock_get.side_effect = requests.exceptions.ConnectionError

        # Call the function with a remote file
        with self.assertRaises(SystemExit):
            get_xml_data("http://example.com/", ".", "test.xml")

    def test_remove_prefix(self):
        # Test case 1: Normal operation
        self.assertEqual(remove_prefix("prefix_test", "prefix_"), "test")

        # Test case 2: Prefix not present
        self.assertEqual(remove_prefix("test", "prefix_"), "test")

        # Test case 3: Empty string
        self.assertEqual(remove_prefix("", "prefix_"), "")

    def test_split_into_lines(self):
        # Test case 1: Normal operation
        string_to_split = "This is a test string. It should be split into several lines."
        maximum_line_length = 12
        expected_output = ["This is a", "test string.", "It should be", "split into", "several",  "lines."]
        self.assertEqual(split_into_lines(string_to_split, maximum_line_length), expected_output)

        # Test case 2: String shorter than maximum line length
        string_to_split = "Short"
        maximum_line_length = 10
        expected_output = ["Short"]
        self.assertEqual(split_into_lines(string_to_split, maximum_line_length), expected_output)

        # Test case 3: Empty string
        string_to_split = ""
        maximum_line_length = 10
        expected_output = []
        self.assertEqual(split_into_lines(string_to_split, maximum_line_length), expected_output)

    def test_create_doc_dict(self):
        # Mock XML data
        xml_data = '''
        <root>
            <param name="PARAM1" humanName="Param 1" documentation="Documentation for Param 1">
                <field name="Field1">Value1</field>
                <field name="Field2">Value2</field>
                <values>
                    <value code="Code1">Value1</value>
                    <value code="Code2">Value2</value>
                </values>
            </param>
            <param name="PARAM2" humanName="Param 2" documentation="Documentation for Param 2">
                <field name="Units">m/s</field>
                <field name="UnitText">meters per second</field>
                <values>
                    <value code="Code3">Value3</value>
                    <value code="Code4">Value4</value>
                </values>
            </param>
        </root>
        '''
        root = ET.fromstring(xml_data)

        # Expected output
        expected_output = {
            "PARAM1": {
                "humanName": "Param 1",
                "documentation": ["Documentation for Param 1"],
                "fields": {"Field1": "Value1", "Field2": "Value2"},
                "values": {"Code1": "Value1", "Code2": "Value2"}
            },
            "PARAM2": {
                "humanName": "Param 2",
                "documentation": ["Documentation for Param 2"],
                "fields": {"Units": "m/s (meters per second)"},
                "values": {"Code3": "Value3", "Code4": "Value4"}
            }
        }

        # Call the function with the mock XML data
        result = create_doc_dict(root, "VehicleType")

        # Check the result
        self.assertEqual(result, expected_output)

    def test_format_columns(self):
        # Define the input
        values = {
            "Key1": "Value1",
            "Key2": "Value2",
            "Key3": "Value3",
            "Key4": "Value4",
            "Key5": "Value5",
            "Key6": "Value6",
            "Key7": "Value7",
            "Key8": "Value8",
            "Key9": "Value9",
            "Key10": "Value10",
            "Key11": "Value11",
            "Key12": "Value12",
        }

        # Define the expected output
        expected_output = [
            'Key1: Value1                                         Key7: Value7',
            'Key2: Value2                                         Key8: Value8',
            'Key3: Value3                                         Key9: Value9',
            'Key4: Value4                                         Key10: Value10',
            'Key5: Value5                                         Key11: Value11',
            'Key6: Value6                                         Key12: Value12',
        ]

        # Call the function with the input
        result = format_columns(values)

        # Check the result
        self.assertEqual(result, expected_output)

        self.assertEqual(format_columns({}), [])

    def test_update_parameter_documentation(self):
        # Write some initial content to the temporary file
        with open(self.temp_file.name, "w", encoding="utf-8") as file:
            file.write("PARAM1 100\n")

        # Call the function with the temporary file
        update_parameter_documentation(self.doc_dict, self.temp_file.name)

        # Read the updated content from the temporary file
        with open(self.temp_file.name, "r", encoding="utf-8") as file:
            updated_content = file.read()

        # Check if the file has been updated correctly
        self.assertIn("Param 1", updated_content)
        self.assertIn("Documentation for Param 1", updated_content)
        self.assertIn("Field1: Value1", updated_content)
        self.assertIn("Field2: Value2", updated_content)
        self.assertIn("Code1: Value1", updated_content)
        self.assertIn("Code2: Value2", updated_content)

    def test_update_parameter_documentation_sorting_none(self):
        # Write some initial content to the temporary file
        # With stray leading and trailing whitespaces
        with open(self.temp_file.name, "w", encoding="utf-8") as file:
            file.write("PARAM2 100\n PARAM_1 100 \nPARAM3 3\nPARAM4 4\nPARAM5 5\nPARAM1 100\n")

        # Call the function with the temporary file
        update_parameter_documentation(self.doc_dict, self.temp_file.name)

        # Read the updated content from the temporary file
        with open(self.temp_file.name, "r", encoding="utf-8") as file:
            updated_content = file.read()

        expected_content = '''# Param 2
# Documentation for Param 2
# Field3: Value3
# Field4: Value4
# Code3: Value3
# Code4: Value4
PARAM2 100

# Param _ 1
# Documentation for Param_1
# Field_1: Value_1
# Field_2: Value_2
# Code_1: Value_1
# Code_2: Value_2
PARAM_1 100
PARAM3 3
PARAM4 4
PARAM5 5

# Param 1
# Documentation for Param 1
# Field1: Value1
# Field2: Value2
# Code1: Value1
# Code2: Value2
PARAM1 100
'''
        self.assertEqual(updated_content, expected_content)

    def test_update_parameter_documentation_sorting_missionplanner(self):
        # Write some initial content to the temporary file
        with open(self.temp_file.name, "w", encoding="utf-8") as file:
            file.write("PARAM2 100 # ignore, me\nPARAM_1\t100\nPARAM1,100\n")

        # Call the function with the temporary file
        update_parameter_documentation(self.doc_dict, self.temp_file.name, "missionplanner")

        # Read the updated content from the temporary file
        with open(self.temp_file.name, "r", encoding="utf-8") as file:
            updated_content = file.read()

        expected_content = '''# Param _ 1
# Documentation for Param_1
# Field_1: Value_1
# Field_2: Value_2
# Code_1: Value_1
# Code_2: Value_2
PARAM_1\t100

# Param 1
# Documentation for Param 1
# Field1: Value1
# Field2: Value2
# Code1: Value1
# Code2: Value2
PARAM1,100

# Param 2
# Documentation for Param 2
# Field3: Value3
# Field4: Value4
# Code3: Value3
# Code4: Value4
PARAM2 100 # ignore, me
'''
        self.assertEqual(updated_content, expected_content)

    def test_update_parameter_documentation_sorting_mavproxy(self):
        # Write some initial content to the temporary file
        with open(self.temp_file.name, "w", encoding="utf-8") as file:
            file.write("PARAM2 100\nPARAM_1\t100\nPARAM1,100\n")

        # Call the function with the temporary file
        update_parameter_documentation(self.doc_dict, self.temp_file.name, "mavproxy")

        # Read the updated content from the temporary file
        with open(self.temp_file.name, "r", encoding="utf-8") as file:
            updated_content = file.read()

        expected_content = '''# Param 1
# Documentation for Param 1
# Field1: Value1
# Field2: Value2
# Code1: Value1
# Code2: Value2
PARAM1,100

# Param 2
# Documentation for Param 2
# Field3: Value3
# Field4: Value4
# Code3: Value3
# Code4: Value4
PARAM2 100

# Param _ 1
# Documentation for Param_1
# Field_1: Value_1
# Field_2: Value_2
# Code_1: Value_1
# Code_2: Value_2
PARAM_1\t100
'''
        self.assertEqual(updated_content, expected_content)

    def test_update_parameter_documentation_invalid_line_format(self):
        # Write some initial content to the temporary file with an invalid line format
        with open(self.temp_file.name, "w", encoding="utf-8") as file:
            file.write("%INVALID_LINE_FORMAT\n")

        # Call the function with the temporary file
        with self.assertRaises(SystemExit) as cm:
            update_parameter_documentation(self.doc_dict, self.temp_file.name)

        # Check if the SystemExit exception contains the expected message
        self.assertEqual(cm.exception.code, "Invalid line in input file")

    @patch('logging.Logger.info')
    def test_print_read_only_params(self, mock_info):
        # Mock XML data
        xml_data = '''
        <root>
            <param name="PARAM1" humanName="Param 1" documentation="Documentation for Param 1">
                <field name="ReadOnly">True</field>
                <field name="Field1">Value1</field>
                <field name="Field2">Value2</field>
                <values>
                    <value code="Code1">Value1</value>
                    <value code="Code2">Value2</value>
                </values>
            </param>
            <param name="PARAM2" humanName="Param 2" documentation="Documentation for Param 2">
                <field name="Field3">Value3</field>
                <field name="Field4">Value4</field>
                <values>
                    <value code="Code3">Value3</value>
                    <value code="Code4">Value4</value>
                </values>
            </param>
        </root>
        '''
        root = ET.fromstring(xml_data)
        doc_dict = create_doc_dict(root, "VehicleType")

        # Call the function with the mock XML data
        print_read_only_params(doc_dict)

        # Check if the parameter name was logged
        mock_info.assert_has_calls([mock.call('ReadOnly parameters:'), mock.call('PARAM1')])

    def test_update_parameter_documentation_invalid_target(self):
        # Call the function with an invalid target
        with self.assertRaises(ValueError):
            update_parameter_documentation(self.doc_dict, "invalid_target")

    def test_invalid_parameter_name(self):
        # Write some initial content to the temporary file
        with open(self.temp_file.name, "w", encoding="utf-8") as file:
            file.write("INVALID_$PARAM 100\n")

        # Call the function with the temporary file
        with self.assertRaises(SystemExit):
            update_parameter_documentation(self.doc_dict, self.temp_file.name)

    def test_update_parameter_documentation_too_long_parameter_name(self):
        # Write some initial content to the temporary file
        with open(self.temp_file.name, "w", encoding="utf-8") as file:
            file.write("TOO_LONG_PARAMETER_NAME 100\n")

        # Call the function with the temporary file
        with self.assertRaises(SystemExit):
            update_parameter_documentation(self.doc_dict, self.temp_file.name)

    @patch('logging.Logger.warning')
    def test_missing_parameter_documentation(self, mock_warning):
        # Write some initial content to the temporary file
        with open(self.temp_file.name, "w", encoding="utf-8") as file:
            file.write("MISSING_DOC_PARA 100\n")

        # Call the function with the temporary file
        update_parameter_documentation(self.doc_dict, self.temp_file.name)

        # Check if the warnings were logged
        mock_warning.assert_has_calls([
            mock.call('Read file %s with %d parameters, but only %s of which got documented', self.temp_file.name, 1, 0),
            mock.call('No documentation found for: %s', 'MISSING_DOC_PARA')
        ])

    def test_empty_parameter_file(self):
        # Call the function with the temporary file
        update_parameter_documentation(self.doc_dict, self.temp_file.name)

        # Read the updated content from the temporary file
        with open(self.temp_file.name, "r", encoding="utf-8") as file:
            updated_content = file.read()

        # Check if the file is still empty
        self.assertEqual(updated_content, "")


if __name__ == '__main__':
    unittest.main()
