#! /usr/bin/env python
# encoding: utf-8
# Calle Rosenquist, 2016 (xbreak)

import os

# Import the foo extension shared object
from foo import foo_ext

def sum(a, b):
	return a + b

def ping():
	return foo_ext.ping()

