#! /usr/bin/env python
# encoding: utf-8

"""
Modification to handle folders as if they were files.

Usually, the target folders are created automatically (Node.find_or_declare)
for files that need them so this is not really necessary.

This modification incurs a performance penalty (computing hashes,
creating additional tasks, checking if the folders are there
vs just creating the folders if missing), and can conceal serious
errors (confusing files and folders for example).

The build order will not look at the parent folder relationships,
we will need a testcase for this (overriding the function
Task.set_file_constraints is trivial)
"""

import stat, os
from waflib import Utils, Task
from waflib.TaskGen import feature

def h_file(filename):
	"""now folders can have a signature too"""
	st = os.stat(filename)
	if stat.S_ISDIR(st[stat.ST_MODE]):
		return Utils.md5(filename).digest()
	m = Utils.md5()
	m.update(str(st.st_mtime))
	m.update(str(st.st_size))
	m.update(filename)
	return m.digest()
Utils.h_file = h_file

@feature('mkdir')
def make_target_folder(self):
	"""code provided as an example"""
	try:
		node = self.target
	except AttributeError:
		raise self.bld.errors.WafError('Missing target attribute on task generator %r' % self)
	self.create_task('mkdir', [], node)

class mkdir(Task.Task):
	"""calling node.mkdir() will be more efficient than creating folders"""
	def run(self):
		self.outputs[0].mkdir()

