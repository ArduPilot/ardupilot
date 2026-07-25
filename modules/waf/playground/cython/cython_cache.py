#! /usr/bin/env python
# encoding: utf-8
# Thomas Nagy, 2012

"""
A simple cache layer to enable the redistribution of precompiled cython files
"""

from waflib.Task import ASK_LATER
from waflib.extras.cython import cython as cython_base

class cython(cython_base):

	def runnable_status(self):
		ret = cython_base.runnable_status(self)
		if ret != ASK_LATER:
			# we can create Node objects since we are in the main thread
			bld = self.generator.bld
			cache = bld.srcnode.make_node('cython_cache')
			if self.env.CYTHON: # write to the cache directory
				self.cython_cache_outputs = [cache.make_node(x.path_from(bld.bldnode)) for x in self.outputs]
			else: # use the files in the cache directory
				self.cython_cache_outputs = [cache.find_node(x.path_from(bld.bldnode)) for x in self.outputs]
		return ret

	def run(self):	
		if self.env.CYTHON:
			ret = cython_base.run(self)
			if not ret:
				for (x, y) in zip(self.outputs, self.cython_cache_outputs):
					y.parent.mkdir()
					y.write(x.read('rb'), 'wb')
			return ret
		else:
			for (x, y) in zip(self.outputs, self.cython_cache_outputs):
				x.write(y.read('rb'), 'wb')

