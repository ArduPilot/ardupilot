#! /usr/bin/env python
# encoding: utf-8
# DC 2008
# Thomas Nagy 2010 (ita)

import re

from waflib import Utils, Task, TaskGen, Logs
from TaskGen import feature, before, after, extension
from waflib.Tools import ccroot

IS_MODULE_R = re.compile('module ([a-z]*)')
USE_MODULE_R = re.compile('use ([a-z]*)')

@extension('.a')
def hook(self, node):
	self.create_compiled_task('fakecc', node)

def ismodule(node):
	deps = []
	for l in node.read().splitlines():
		m = IS_MODULE_R.match(l)
		if m:
			deps.append(m.group(1) + '.mod')
	return deps

def usemodule(node):
	deps = []
	for l in node.read().splitlines():
		m = USE_MODULE_R.match(l)
		if m:
			deps.append(m.group(1) + '.mod')
	return deps

def compile(tsk):
	tsk.outputs[0].write('compiled')
	m = ismodule(tsk.inputs[0])
	if m:
		print("%s declares module %s" % (tsk.inputs[0], m[0]))
		t2 = open(m[0], 'w')
		try:
			t2.write('module compiled')
		finally:
			t2.close()

class fakecc(Task.Task):
	color = 'YELLOW'
	def run(self):
		cmd = []
		if not len(self.outputs) == len(self.inputs) == 1:
			pass

		bnodes = self.outputs
		m = usemodule(self.inputs[0])
		if m:
			print("%s requires module %s" % (self.inputs[0].abspath(), m[0]))
			#bnodes.append(self.generator.bld.bldnode.exclusive_build_node(m[0]))

		compile(self)

