#!/usr/bin/env python
#
# Copyright (C) 2014-2015  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ben Dyer <ben_dyer@mac.com>
#         Pavel Kirienko <pavel.kirienko@zubax.com>
#

import os
import sys
from setuptools import setup
from io import open

__version__ = None
VERSION_FILE = os.path.join(os.path.dirname(__file__), 'dronecan', 'version.py')
exec(open(VERSION_FILE).read())         # Adds __version__ to globals

with open("README.md", "r", encoding = "utf-8") as fh:
    long_description = fh.read()

try:
    if not os.path.exists('dronecan/dsdl_specs'):
        os.symlink('../../DSDL', 'dronecan/dsdl_specs')
    args = dict(
        version=__version__,
        package_data={
            'dronecan': [os.path.join(root[len('dronecan/'):], fname)
                    for root, dirs, files in os.walk('dronecan/dsdl_specs', followlinks=True)
                    for fname in files if fname.endswith('.uavcan')]
        },
        scripts = [ 'tools/dronecan_bridge.py' ]
    )
    # ensure dsdl specs are not empty
    if  len(args['package_data']['dronecan']) == 0:
        raise Exception('DSDL specs empty or unavailable, please ensure ../DSDL is present relative to project root')

    if sys.version_info[0] < 3:
        args['install_requires'] = ['monotonic']

    setup(**args)
finally:
    if os.path.islink('dronecan/dsdl_specs'):
        os.unlink('dronecan/dsdl_specs')
