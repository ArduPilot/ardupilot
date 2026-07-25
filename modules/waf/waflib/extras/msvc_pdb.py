#!/usr/bin/env python
# encoding: utf-8
# Rafaël Kooi 2019

from waflib import TaskGen

@TaskGen.feature('c', 'cxx', 'fc')
@TaskGen.after_method('propagate_uselib_vars')
def add_pdb_per_object(self):
	"""For msvc/fortran, specify a unique compile pdb per object, to work
	around LNK4099. Flags are updated with a unique /Fd flag based on the
	task output name. This is separate from the link pdb.
	"""
	if not hasattr(self, 'compiled_tasks'):
		return

	link_task = getattr(self, 'link_task', None)

	for task in self.compiled_tasks:
		if task.inputs and task.inputs[0].name.lower().endswith('.rc'):
			continue

		add_pdb = False
		for flagname in ('CFLAGS', 'CXXFLAGS', 'FCFLAGS'):
			# several languages may be used at once
			for flag in task.env[flagname]:
				if flag[1:].lower() == 'zi':
					add_pdb = True
					break

		if add_pdb:
			node = task.outputs[0].change_ext('.pdb')
			pdb_flag = '/Fd:' + node.abspath()

			for flagname in ('CFLAGS', 'CXXFLAGS', 'FCFLAGS'):
				buf = [pdb_flag]
				for flag in task.env[flagname]:
					if flag[1:3] == 'Fd' or flag[1:].lower() == 'fs' or flag[1:].lower() == 'mp':
						continue
					buf.append(flag)
				task.env[flagname] = buf

			if link_task and not node in link_task.dep_nodes:
				link_task.dep_nodes.append(node)
			if not node in task.outputs:
				task.outputs.append(node)
