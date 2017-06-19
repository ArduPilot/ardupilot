#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['gencpp'],
    package_dir={'': 'src'},
    requires=['genmsg']
)

setup(**d)
