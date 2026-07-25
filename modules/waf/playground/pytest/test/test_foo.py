#! /usr/bin/env python
# encoding: utf-8
# Calle Rosenquist, 2016 (xbreak)

import unittest
from foo import foo

class test_foo(unittest.TestCase):
	def test_add_integers(self):
		self.assertEqual(7, foo.sum(4, 3))

	def test_add_strings(self):
		self.assertEqual('foobar', foo.sum('foo', 'bar'))

	def test_foo(self):
		self.assertEqual('pong', foo.ping())
