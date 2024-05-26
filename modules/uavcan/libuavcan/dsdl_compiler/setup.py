#!/usr/bin/env python

from distutils.core import setup

args = dict(
    name='libuavcan_dsdl_compiler',
    version='0.1',
    description='UAVCAN DSDL compiler for libuavcan',
    packages=['libuavcan_dsdl_compiler'],
    package_data={'libuavcan_dsdl_compiler': ['data_type_template.tmpl']},
    scripts=['libuavcan_dsdlc'],
    requires=['uavcan'],
    author='Pavel Kirienko',
    author_email='pavel.kirienko@gmail.com',
    url='http://uavcan.org',
    license='MIT'
)

setup(**args)
