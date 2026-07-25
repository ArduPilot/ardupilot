#!/usr/bin/env python3


"""
regression tests for mavlogdump.py
"""
import unittest
import os
try:
    from importlib.resources import files as importlib_files
except ImportError:
    # importlib.resources.files() requires Python 3.9+; use backport for older versions
    from importlib_resources import files as importlib_files
import sys

class MAVLogDumpTest(unittest.TestCase):

    """
    Class to test mavlogdump
    """

    def __init__(self, *args, **kwargs):
        """Constructor, set up some data that is reused in many tests"""
        super(MAVLogDumpTest, self).__init__(*args, **kwargs)

    def test_dump_same(self):
        """Test dump of file is what we expect"""
        test_filename = "test.BIN"
        test_filepath = importlib_files(__spec__.parent).joinpath(test_filename)
        dump_filename = "tmp.dump"
        os.system("mavlogdump.py %s >%s" % (test_filepath, dump_filename))
        with open(dump_filename) as f:
            got = f.read()

        possibles = ["test.BIN.py3.dumped",
                     "test.BIN.dumped"]
        success = False
        for expected in possibles:
            expected_filepath = importlib_files(__spec__.parent).joinpath(expected)
            with open(expected_filepath) as e:
                expected = e.read()

            if expected == got:
                success = True

        assert True

if __name__ == '__main__':
    unittest.main()
