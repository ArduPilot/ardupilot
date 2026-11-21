"""Regression tests for parameter scaling and rename collisions.

AP_FLAKE8_CLEAN
"""

import contextlib
import io
import pathlib
import tempfile
import unittest

from Tools.scripts import convert_param_scale as converter


class TestConvertParamScale(unittest.TestCase):
    def test_same_name_scaling(self):
        for before, after in (
            ('TEST_PARAM,100\n', 'TEST_PARAM,1\n'),
            ('  TEST_PARAM = 100 # keep comment\r\n', '  TEST_PARAM = 1 # keep comment\r\n'),
            ('TEST_PARAM\t100', 'TEST_PARAM\t1'),
            ('1\t1\tTEST_PARAM\t100\t9\n', '1\t1\tTEST_PARAM\t1\t9\n'),
        ):
            with self.subTest(before=before):
                lines = [converter.parse_line(before)]
                with self.assertNoLogs(level='WARNING'):
                    changes = converter.apply_renames(
                        lines, {'TEST_PARAM': converter.Rename('TEST_PARAM', 'TEST_PARAM', 0.01)})
                self.assertEqual(''.join(line.render() for line in lines), after)
                self.assertEqual(len(changes), 1)
                self.assertEqual(changes[0].old_text, before)
                self.assertEqual(changes[0].new_text, after)

    def test_existing_destination_is_preserved(self):
        for content in ('OLD_PARAM,100\nNEW_PARAM,7\n', 'NEW_PARAM,7\nOLD_PARAM,100\n'):
            with self.subTest(content=content):
                lines = [converter.parse_line(text) for text in converter.split_lines(content)]
                with self.assertLogs(level='WARNING') as logs:
                    changes = converter.apply_renames(
                        lines, {'OLD_PARAM': converter.Rename('OLD_PARAM', 'NEW_PARAM', 0.01)})
                self.assertEqual(changes, [])
                self.assertEqual(''.join(line.render() for line in lines), content)
                self.assertIn('NEW_PARAM already present', logs.output[0])

    def test_new_destination_collision_is_preserved(self):
        lines = [converter.parse_line('FIRST,100\n'), converter.parse_line('SECOND,200\n')]
        renames = {name: converter.Rename(name, 'DEST', 0.01) for name in ('FIRST', 'SECOND')}
        with self.assertLogs(level='WARNING'):
            changes = converter.apply_renames(lines, renames)
        self.assertEqual(len(changes), 1)
        self.assertEqual(''.join(line.render() for line in lines), 'DEST,1\nSECOND,200\n')

    def test_command_line_same_name_scaling(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / 'example.param'
            path.write_bytes(b'TEST_PARAM,100\n')
            with contextlib.redirect_stdout(io.StringIO()), self.assertNoLogs(level='WARNING'):
                result = converter.main(['--directory', directory, '--scale', '0.01', 'test_param', 'TEST_PARAM'])
            self.assertEqual(result, 0)
            self.assertEqual(path.read_bytes(), b'TEST_PARAM,1\n')


if __name__ == '__main__':
    unittest.main()
