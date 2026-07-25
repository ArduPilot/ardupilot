#
# Copyright (C) 2014-2015  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ben Dyer <ben_dyer@mac.com>
#         Pavel Kirienko <pavel.kirienko@zubax.com>
#

import os
import unittest
from dronecan.dsdl import parser
from dronecan.dsdl import parser, parse_namespaces
from dronecan.dsdl.common import DsdlException


class TestParseNamespaces(unittest.TestCase):
    '''
    Unittests of parse_namespaces function.
    '''

    def test_builtin(self):
        '''
        Test the ability to load all the UAVCAN v0 messages
        '''
        built_in_dir = '{}/../../../DSDL/uavcan'.format(os.path.dirname(__file__))
        parse_namespaces([built_in_dir])

    def test_duplicate_in_search_dir(self):
        '''
        Validate the parser allows and handles duplicate definitions in the search dir
        '''
        ns0_dir = '{}/fake_dsdl/ns0_base/ns0'.format(os.path.dirname(__file__))
        ns0_dir_with_duplicate = '{}/fake_dsdl/ns0_duplicated/ns0'.format(os.path.dirname(__file__))

        parse_namespaces([ns0_dir], [ns0_dir_with_duplicate])

    def test_redefinition_in_search_dir(self):
        '''
        Validate the parser does not allow redefinitions in the search dir
        '''
        ns0_dir = '{}/fake_dsdl/ns0_base/ns0'.format(os.path.dirname(__file__))
        ns0_dir_with_redefinition = '{}/fake_dsdl/ns0_redefined/ns0'.format(os.path.dirname(__file__))
        try:
            parse_namespaces([ns0_dir], [ns0_dir_with_redefinition])
            self.assertTrue(False) # parse_namespaces should raise an exception, shouldn't get here
        except DsdlException as e:
            self.assertTrue(e.args[0].startswith("Redefinition of data type ID"))


if __name__ == '__main__':
    unittest.main()
