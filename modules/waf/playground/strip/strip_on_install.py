#! /usr/bin/env python

"""
Strip executables upon installation
"""

import shutil, os
from waflib import Build, Utils, Context

def copy_fun(self, src, tgt):
	if Utils.is_win32 and len(tgt) > 259 and not tgt.startswith('\\\\?\\'):
		tgt = '\\\\?\\' + tgt
	shutil.copy2(src, tgt)
	os.chmod(tgt, self.chmod)

	if getattr(self.generator, 'link_task', None):
		if self.generator.link_task.outputs[0] in self.inputs:
			self.generator.bld.cmd_and_log('strip %s' % tgt, quiet=Context.BOTH)
Build.inst.copy_fun = copy_fun

