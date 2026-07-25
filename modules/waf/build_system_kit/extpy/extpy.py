#! /usr/bin/env python
# encoding: utf-8

import os
from waflib import Errors, Utils
from waflib import Context as mod

class Context(mod.Context):
	cmd = 'all'
	def recurse(self, dirs, name=None, mandatory=True, once=True):
		try:
			cache = self.recurse_cache
		except:
			cache = self.recurse_cache = {}

		for d in Utils.to_list(dirs):

			if not os.path.isabs(d):
				# absolute paths only
				d = os.path.join(self.path.abspath(), d)

			WSCRIPT     = os.path.join(d, 'wscript.py')
			WSCRIPT_FUN = 'wscript_' + (name or self.fun) + '.py'

			node = self.root.find_node(WSCRIPT_FUN)
			if node and (not once or node not in cache):
				cache[node] = True
				self.pre_recurse(node)
				try:
					function_code = node.read('r')
					exec(compile(function_code, node.abspath(), 'exec'), self.exec_dict)
				finally:
					self.post_recurse(node)
			elif not node:
				node = self.root.find_node(WSCRIPT)
				if node and (not once or node not in cache):
					cache[node] = True
					self.pre_recurse(node)
					try:
						wscript_module = mod.load_module(node.abspath())
						user_function = getattr(wscript_module, (name or self.fun), None)
						if not user_function:
							if not mandatory:
								continue
							raise Errors.WafError('No function %s defined in %s' % (name or self.fun, node.abspath()))
						user_function(self)
					finally:
						self.post_recurse(node)
				elif not node:
					if not mandatory:
						continue
					raise Errors.WafError('No wscript file in directory %s' % d)
mod.Context = Context
mod.WSCRIPT_FILE = 'wscript.py'
