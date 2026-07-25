#! /usr/bin/env python

import os, sys, imp
from waflib import Context, Options, Configure, Utils, Logs

def start(cwd, version, wafdir):
	# simple example, the file main.c is hard-coded
	try:
		os.stat(cwd + os.sep + 'bbit')
	except:
		print('call from a folder containing a file named "bbit"')
		sys.exit(1)

	Logs.init_log()
	Context.waf_dir = wafdir
	Context.top_dir = Context.run_dir = cwd
	Context.out_dir = os.path.join(cwd, 'build')
	Context.g_module = imp.new_module('wscript')
	Context.g_module.root_path = os.path.join(cwd, 'bbit')
	Context.Context.recurse = \
		lambda x, y: getattr(Context.g_module, x.cmd or x.fun, Utils.nada)(x)

	Context.g_module.configure = lambda ctx: ctx.load('g++')
	Context.g_module.build = lambda bld: bld.objects(source='main.c')

	Options.OptionsContext().execute()

	do_config = 'configure' in sys.argv
	try:
		os.stat(cwd + os.sep + 'build')
	except:
		do_config = True
	if do_config:
		Context.create_context('configure').execute()

	if 'clean' in sys.argv:
		Context.create_context('clean').execute()
	if 'build' in sys.argv:
		Context.create_context('build').execute()
