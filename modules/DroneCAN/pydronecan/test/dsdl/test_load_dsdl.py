#
# Copyright (C) 2014-2015  UAVCAN Development Team  <uavcan.org>
# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ben Dyer <ben_dyer@mac.com>
#         Pavel Kirienko <pavel.kirienko@zubax.com>
#

import unittest
import os
from dronecan.dsdl import signature
from dronecan.dsdl.common import DsdlException
from dronecan import load_dsdl


class TestLoadDsdl(unittest.TestCase):
    '''
    Unittests of the load_dsdl method.
    '''

    def test_reload(self):
        '''
        Test calling load_dsdl again (after it is called by the uavcan module)
        '''
        ns0_dir = '{}/fake_dsdl/ns0_base/ns0'.format(os.path.dirname(__file__))
        load_dsdl(ns0_dir, exclude_dist=True)
        import dronecan
        test_type = dronecan.thirdparty.ns0.Type0()
        self.assertEqual(test_type.field0, 0)

    def test_reload_with_redefinition(self):
        '''
        Test calling load_dsdl with paths that contain two different types
        with the same id.
        '''
        ns0_dir = '{}/fake_dsdl/ns0_base/ns0'.format(os.path.dirname(__file__))
        ns0_dir_with_redefinition = '{}/fake_dsdl/ns0_redefined/ns0'.format(os.path.dirname(__file__))
        try:
            load_dsdl(ns0_dir, ns0_dir_with_redefinition, exclude_dist=True)
        except DsdlException as e:
            self.assertTrue(e.args[0].startswith("Redefinition of data type ID"))

    def test_reload_with_duplication(self):
        '''
        Test calling load_dsdl with paths that contain the same type.
        '''
        ns0_dir = '{}/fake_dsdl/ns0_base/ns0'.format(os.path.dirname(__file__))
        ns0_dir_with_redefinition = '{}/fake_dsdl/ns0_duplicated/ns0'.format(os.path.dirname(__file__))
        load_dsdl(ns0_dir, ns0_dir_with_redefinition, exclude_dist=True)
