# encoding: utf-8

# flake8: noqa

"""
WAF Tool to force programs to be statically linked
"""

from waflib.TaskGen import after_method, feature

@feature('static_linking')
@after_method('apply_link')
def force_static_linking(self):
	env = self.link_task.env
	env.STLIB += env.LIB
	env.LIB = []
	env.STLIB_MARKER = '-static'
	env.SHLIB_MARKER = ''
