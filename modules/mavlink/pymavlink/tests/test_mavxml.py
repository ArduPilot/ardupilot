#!/usr/bin/env python3
"""
Module to test MAVXML
"""

import unittest
try:
    from importlib.resources import files as importlib_files
except ImportError:
    # importlib.resources.files() requires Python 3.9+; use backport for older versions
    from importlib_resources import files as importlib_files

from pymavlink.generator.mavparse import MAVXML
from pymavlink.generator.mavparse import MAVParseError

class MAVXMLTest(unittest.TestCase):
    """
    Class to test MAVXML
    """

    def test_fields_number(self):
        """Test that a message can have at most 64 fields"""
        test_filename = "64-fields.xml"
        test_filepath = importlib_files(__spec__.parent).joinpath(test_filename)
        xml = MAVXML(test_filepath)
        count = len(xml.message[0].fields)
        self.assertEqual(count, 64)

        test_filename = "65-fields.xml"
        test_filepath = importlib_files(__spec__.parent).joinpath(test_filename)
        with self.assertRaises(MAVParseError):
            _ = MAVXML(test_filepath)


    def test_wire_protocol_version(self):
        """Test that an unknown MAVLink wire protocol version raises an exception"""
        with self.assertRaises(MAVParseError):
            _ = MAVXML(filename="", wire_protocol_version=42)


if __name__ == '__main__':
    unittest.main()
