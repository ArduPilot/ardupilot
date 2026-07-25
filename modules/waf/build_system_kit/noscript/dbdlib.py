#! /usr/bin/env python

import os, sys, imp
from waflib import Context, Options, Configure, Utils, Logs, TaskGen, Task
import waflib.Tools.c

"""
Compile main.c and dependent object files into a single target (program/shlib/stlib or just object files)

- no build directory and no script files
- just a c4che directory for the configuration files
- configure, clean or build

Uses the task signatures and the dependency calculation results to avoid
rescanning/rebuilding the files all the time
"""

def options(opt):
	opt.add_option('--type', action='store', default='program', help='type: program, shlib, stlib, objects', dest='progtype')
	opt.add_option('--source', action='store', default='main.c', help='space-separated list of source files', dest='source')
	opt.add_option('--app', action='store', default='app', help='name of the binary file to create', dest='app')
	opt.load('compiler_c')

def configure(conf):
	conf.options = Options.options
	conf.load('compiler_c')

def build(bld):
	tp = Options.options.progtype
	features = 'c cprogram'
	if tp == 'shlib':
		features = 'c cshlib'
	elif tp == 'stlib':
		features = 'c cstlib'
	elif tp == 'objects':
		features = 'c'

	app = Options.options.app
	bld(features=features, source=Options.options.source, target=app)

def recurse_rep(x, y):
	f = getattr(Context.g_module, x.cmd or x.fun, Utils.nada)
	return f(x)

def start(cwd, version, wafdir):
	# this is the entry point of our small build system
	# no script file here
	Logs.init_log()
	Context.waf_dir = wafdir
	Context.out_dir = Context.top_dir = Context.run_dir = cwd
	Context.g_module = imp.new_module('wscript')
	Context.g_module.root_path = cwd
	Context.Context.recurse = recurse_rep

	Context.g_module.configure = configure
	Context.g_module.build = build
	Context.g_module.options = options
	Context.g_module.top = Context.g_module.out = '.'

	Options.OptionsContext().execute()

	do_config = 'configure' in sys.argv
	try:
		os.stat(cwd + os.sep + 'c4che')
	except:
		do_config = True
	if do_config:
		Context.create_context('configure').execute()

	if 'clean' in sys.argv:
		Context.create_context('clean').execute()

	if 'build' in sys.argv:
		Context.create_context('build').execute()


class c2(waflib.Tools.c.c):
	# Make a subclass of the default c task, and bind the .c extension to it

	def runnable_status(self):
		ret = super(waflib.Tools.c.c, self).runnable_status()
		self.more_tasks = []

		# use a cache to avoid creating the same tasks
		# for example, truc.cpp might be compiled twice
		try:
			shared = self.generator.bld.shared_tasks
		except AttributeError:
			shared = self.generator.bld.shared_tasks = {}

		if ret != Task.ASK_LATER:
			for x in self.generator.bld.node_deps[self.uid()]:
				node = x.parent.get_src().find_resource(x.name.replace('.h', '.c'))
				if node:
					try:
						tsk = shared[node]
					except:
						tsk = shared[node] = self.generator.c_hook(node)

						self.more_tasks.append(tsk)

					# add the node created to the link task outputs
					try:
						link = self.generator.link_task
					except AttributeError:
						pass
					else:
						if not tsk.outputs[0] in link.inputs:
							link.inputs.append(tsk.outputs[0])
							link.set_run_after(tsk)

							# any change in the order of the input nodes may cause a recompilation
							link.inputs.sort(key=lambda x: x.abspath())

			# if you want to modify some flags
			# you *must* have the task recompute the signature
			self.env.append_value('CXXFLAGS', '-O2')
			delattr(self, 'cache_sig')
			return super(waflib.Tools.c.c, self).runnable_status()

		return ret

@TaskGen.extension('.c')
def c_hook(self, node):
	# re-bind the extension to this new class
	return self.create_compiled_task('c2', node)

