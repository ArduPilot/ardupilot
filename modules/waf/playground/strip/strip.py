#! /usr/bin/env python

"""
Strip a program/library after it is created.

Since creating the file and modifying it occurs in the same
task, there will be no race condition with other tasks dependent
on the output.

For other implementation possibilities, see strip_hack.py and strip_on_install.py
"""

from waflib import Task

def configure(conf):
	conf.find_program('strip')

def wrap_compiled_task(classname):
	# override the class to add a new 'run' method
	# such an implementation guarantees that the absence of race conditions
	#
	cls1 = Task.classes[classname]
	cls2 = type(classname, (cls1,), {'run_str': '${STRIP} ${TGT[0].abspath()}'})
	cls3 = type(classname, (cls2,), {})

	def run_all(self):
		if self.env.NO_STRIPPING:
			return cls1.run(self)
		ret = cls1.run(self)
		if ret:
			return ret
		return cls2.run(self)
	cls3.run = run_all

for k in 'cprogram cshlib cxxprogram cxxshlib fcprogram fcshlib dprogram dshlib'.split():
	if k in Task.classes:
		wrap_compiled_task(k)

