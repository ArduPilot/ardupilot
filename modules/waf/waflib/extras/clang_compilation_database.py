#!/usr/bin/env python
# encoding: utf-8
# Christoph Koke, 2013
# Alibek Omarov, 2019

"""
Writes the c and cpp compile commands into build/compile_commands.json
see http://clang.llvm.org/docs/JSONCompilationDatabase.html

Usage:

	Load this tool in `options` to be able to generate database
	by request in command-line and before build:

	$ waf clangdb

	def options(opt):
		opt.load('clang_compilation_database')

	Otherwise, load only in `configure` to generate it always before build.

	def configure(conf):
		conf.load('compiler_cxx')
		...
		conf.load('clang_compilation_database')
"""

from waflib import Logs, TaskGen, Task, Build, Scripting

Task.Task.keep_last_cmd = True

class ClangDbContext(Build.BuildContext):
	'''generates compile_commands.json by request'''
	cmd = 'clangdb'

	def write_compilation_database(self):
		"""
		Write the clang compilation database as JSON
		"""
		database_file = self.bldnode.make_node('compile_commands.json')
		Logs.info('Build commands will be stored in %s', database_file.path_from(self.path))
		try:
			root = database_file.read_json()
		except IOError:
			root = []
		clang_db = dict((x['file'], x) for x in root)
		for task in self.clang_compilation_database_tasks:
			try:
				cmd = task.last_cmd
			except AttributeError:
				continue
			f_node = task.inputs[0]
			filename = f_node.path_from(task.get_cwd())
			entry = {
				"directory": task.get_cwd().abspath(),
				"arguments": cmd,
				"file": filename,
			}
			clang_db[filename] = entry
		root = list(clang_db.values())
		database_file.write_json(root)

	def execute(self):
		"""
		Build dry run
		"""
		self.restore()
		self.cur_tasks = []
		self.clang_compilation_database_tasks = []

		if not self.all_envs:
			self.load_envs()

		self.recurse([self.run_dir])
		self.pre_build()

		# we need only to generate last_cmd, so override
		# exec_command temporarily
		def exec_command(self, *k, **kw):
			return 0

		for g in self.groups:
			for tg in g:
				try:
					f = tg.post
				except AttributeError:
					pass
				else:
					f()

				if isinstance(tg, Task.Task):
					lst = [tg]
				else: lst = tg.tasks
				for tsk in lst:
					if tsk.__class__.__name__ == "swig":
						tsk.runnable_status()
						if hasattr(tsk, 'more_tasks'):
							lst.extend(tsk.more_tasks)
					# Not all dynamic tasks can be processed, in some cases
					# one may have to call the method "run()" like this:
					#elif tsk.__class__.__name__ == 'src2c':
					#	tsk.run()
					#	if hasattr(tsk, 'more_tasks'):
					#		lst.extend(tsk.more_tasks)

					tup = tuple(y for y in [Task.classes.get(x) for x in ('c', 'cxx')] if y)
					if isinstance(tsk, tup):
						self.clang_compilation_database_tasks.append(tsk)
						tsk.nocache = True
						old_exec = tsk.exec_command
						tsk.exec_command = exec_command
						tsk.run()
						tsk.exec_command = old_exec

		self.write_compilation_database()

EXECUTE_PATCHED = False
def patch_execute():
	global EXECUTE_PATCHED

	if EXECUTE_PATCHED:
		return

	def new_execute_build(self):
		"""
		Invoke clangdb command before build
		"""
		if self.cmd.startswith('build'):
			Scripting.run_command(self.cmd.replace('build','clangdb'))

		old_execute_build(self)

	old_execute_build = getattr(Build.BuildContext, 'execute_build', None)
	setattr(Build.BuildContext, 'execute_build', new_execute_build)
	EXECUTE_PATCHED = True

patch_execute()
