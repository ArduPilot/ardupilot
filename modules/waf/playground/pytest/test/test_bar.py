#! /usr/bin/env python
# encoding: utf-8
# Calle Rosenquist, 2016 (xbreak)

import unittest
from bar import bar

class test_bar(unittest.TestCase):
	def test_read_resource(self):
		self.assertEqual('resource!\n', bar.read_resource())
