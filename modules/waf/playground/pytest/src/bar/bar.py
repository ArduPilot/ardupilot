#! /usr/bin/env python
# encoding: utf-8
# Calle Rosenquist, 2017 (xbreak)

import os

def read_resource():
	filename = os.path.join(os.path.dirname(__file__), 'resource.txt')
	with open(filename, 'r') as f:
		return f.readline()

