"""
Validate compinfo-parameter.json output against the local extended MAVLink schema.

AP_FLAKE8_CLEAN
"""

import json
import os
import sys
import unittest

try:
    import jsonschema
except ImportError:
    print("jsonschema package required: pip install jsonschema")
    sys.exit(1)

SCHEMA_PATH = os.path.join(
    os.path.dirname(__file__),
    '..', 'autotest', 'param_metadata', 'compinfo-parameter.schema.json'
)


class TestCompInfoParameterSchema(unittest.TestCase):
    """Validate compinfo-parameter.json files against the extended MAVLink schema."""

    @classmethod
    def setUpClass(cls):
        with open(SCHEMA_PATH, 'r') as f:
            cls.schema = json.load(f)

    def _validate_file(self, path):
        with open(path, 'r') as f:
            data = json.load(f)
        jsonschema.validate(instance=data, schema=self.schema)

    def test_schema_is_valid_draft07(self):
        """Ensure the schema itself is valid JSON Schema Draft-07."""
        jsonschema.Draft7Validator.check_schema(self.schema)

    def test_minimal_valid_document(self):
        """A minimal document with one parameter should validate."""
        doc = {
            "version": 1,
            "parameters": [
                {"name": "TEST_PARAM", "type": "Float"}
            ],
        }
        jsonschema.validate(instance=doc, schema=self.schema)

    def test_readOnly_extension(self):
        """The readOnly field (ArduPilot extension) should validate."""
        doc = {
            "version": 1,
            "parameters": [
                {"name": "TEST_RO", "type": "Float", "readOnly": True}
            ],
        }
        jsonschema.validate(instance=doc, schema=self.schema)

    def test_unknown_field_rejected(self):
        """Fields not in the schema should be rejected."""
        doc = {
            "version": 1,
            "parameters": [
                {"name": "TEST", "type": "Float", "bogusField": 123}
            ],
        }
        with self.assertRaises(jsonschema.ValidationError):
            jsonschema.validate(instance=doc, schema=self.schema)

    def test_param_name_too_long_rejected(self):
        """Parameter names longer than 16 chars should be rejected."""
        doc = {
            "version": 1,
            "parameters": [
                {"name": "PARAM_1234567890XY", "type": "Float"}
            ],
        }
        with self.assertRaises(jsonschema.ValidationError):
            jsonschema.validate(instance=doc, schema=self.schema)

    def test_missing_required_fields(self):
        """Missing required 'type' should be rejected."""
        doc = {
            "version": 1,
            "parameters": [
                {"name": "NO_TYPE"}
            ],
        }
        with self.assertRaises(jsonschema.ValidationError):
            jsonschema.validate(instance=doc, schema=self.schema)

    def test_generated_file_if_exists(self):
        """If parameter.json exists in cwd, validate it."""
        if not os.path.exists('compinfo-parameter.json'):
            self.skipTest("compinfo-parameter.json not found in cwd")
        self._validate_file('compinfo-parameter.json')


if __name__ == '__main__':
    unittest.main()
