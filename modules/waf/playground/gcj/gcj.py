#!/usr/bin/env python
# encoding: utf-8
# Thomas Nagy, 2006-2008 (ita)

"""
Native compilation using gcj

highly experimental, and gcj sucks anyway
"""

import os, re
from waflib.Configure import conf
from waflib import TaskGen, Task, Utils, Node
from waflib.TaskGen import feature, before, after
from waflib.Tools import ccroot

def configure(conf):
	conf.find_program('gcj', var='GCJ')
	conf.env.GCJLINK = conf.env.GCJ
	conf.env.GCJLINKFLAGS_gcj_shlib = ['-shared']
	conf.env.GCJFLAGS_gcj_shlib = ['-fPIC']

class gcj(Task.Task):
	run_str = '${GCJ} ${GCJFLAGS} -classpath ${CLASSPATH} -c -o ${TGT} ${SRC}'

class gcj_program(ccroot.link_task):
	run_str = '${GCJLINK} ${GCJLINKFLAGS} ${SRC} -o ${TGT}'
	color   = 'YELLOW'

class gcj_shlib(gcj_program):
	pass

ccroot.USELIB_VARS['gcj'] = set(['CLASSPATH', 'JAVACFLAGS', 'GCJFLAGS'])
ccroot.USELIB_VARS['gcj_program'] = set(['CLASSPATH', 'JAVACFLAGS', 'GCJLINKFLAGS'])
ccroot.USELIB_VARS['gcj_shlib'] = set(['CLASSPATH', 'JAVACFLAGS', 'GCJLINKFLAGS'])
feature('gcj_program', 'gcj_shlib')(ccroot.apply_link)
feature('gcj_program', 'gcj_shlib')(ccroot.propagate_uselib_vars)

@feature('gcj')
@after('propagate_uselib_vars', 'apply_gcj')
def set_gcj_classpath(self):
	lst = [isinstance(x, str) and x or x.abspath() for x in self.env.CLASSPATH]
	self.env.CLASSPATH = os.pathsep.join(lst) + os.pathsep

@feature('gcj')
@before('apply_java')
def apply_gcj(self):
	if 'javac' in self.features:
		self.bld.fatal('feature gcj_native is not compatible with javac %r' % self)

	srcdir = getattr(self, 'srcdir', '')
	if isinstance(srcdir, Node.Node):
		srcdir = [srcdir]

	tmp = []
	for x in Utils.to_list(srcdir):
		if isinstance(x, Node.Node):
			y = x
		else:
			y = self.path.find_dir(x)
			if not y:
				self.bld.fatal('Could not find the folder %s from %s' % (x, self.path))
		tmp.append(y)

	nodes = []
	for x in tmp:
		nodes.extend(x.ant_glob('**/*.java'))

	if not getattr(self, 'gcjonce', None):
		for x in nodes:
			self.create_compiled_task('gcj', x)

#############################################################
# gcj is still beta software
# and this workaround cannot work for shared object (-fPIC)

class fix_dummy(Task.Task):
	run_str = 'objcopy -L _ZGr8_$$_dummy ${SRC}'
	before  = ['gcj_program', 'gcj_shlib']

@feature('gcj')
@after('apply_gcj')
def gcj_developers_like_duplicate_dummy_symbols(self):
	if self.env.FIX_DUMMY:
		for tsk in self.compiled_tasks:
			if isinstance(tsk, gcj):
				self.create_task('fix_dummy', tsk.outputs[0])

