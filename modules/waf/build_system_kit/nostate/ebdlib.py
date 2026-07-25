#! /usr/bin/env python

import os, sys, imp, time
from waflib import Context, Options, Configure, Utils, Logs, TaskGen, Task, Build, ConfigSet
import waflib.Tools.c

"""
Create a modified waf file in which tasks use timestamps only
see README.txt
"""

# we hard-code a configuration for c but it could be left in the script file too
def configure(conf):
	conf.load('gcc')

def recurse_rep(x, y):
	f = getattr(Context.g_module, x.cmd or x.fun, Utils.nada)
	return f(x)

def start(cwd, version, wafdir):
	# this is the entry point of our small build system

	Logs.init_log()
	Context.waf_dir = wafdir
	Context.out_dir = Context.top_dir = Context.run_dir = cwd
	Context.g_module = Context.load_module(cwd + os.sep + 'wscript')
	Context.g_module.configure = configure
	Context.g_module.root_path = cwd
	Context.Context.recurse = recurse_rep

	Context.g_module.top = Context.g_module.out = '.' # no build directory

	# just parse the options and execute a build
	Options.OptionsContext().execute()

	conf = Context.create_context('configure')
	conf.options = Options.options
	conf.execute()

	bld = Context.create_context('build')
	bld.env = conf.env
	bld.options = Options.options
	bld.environ = os.environ
	bld.execute()

# change the build context so it does not need to write any file
class StatelessBuild(Build.BuildContext):
	def load_envs(self):
		self.env = ConfigSet.ConfigSet()
	def store(self):
		pass
	def restore(self):
		self.init_dirs()
	def execute_build(self):
		# we override this method to hide the messages "leaving directory" (just because)
		self.recurse([self.run_dir])
		self.pre_build()

		self.timer = Utils.Timer()

		if Options.options.progress_bar:
			sys.stderr.write(Logs.colors.cursor_off)
		try:
			self.compile()
		finally:
			if Options.options.progress_bar:
				sys.stderr.write(Logs.colors.cursor_on)
				print('')
		self.post_build()

class SilentConf(Configure.ConfigurationContext):
	# silent configuration
	def __init__(self, **kw):
		# disable the configuration messages from Context.start_msg/end_msg
		self.in_msg = 1
		super(SilentConf, self).__init__(**kw)

	def execute(self):

		# copy-paste from the original method, but without the cache file creation
		self.init_dirs()

		path = os.path.join(self.bldnode.abspath(), 'config.log')
		self.logger = Logs.make_logger(path, 'cfg')

		app = getattr(Context.g_module, 'APPNAME', '')
		if app:
			ver = getattr(Context.g_module, 'VERSION', '')
			if ver:
				app = "%s (%s)" % (app, ver)

		now = time.ctime()
		pyver = sys.hexversion
		systype = sys.platform
		args = " ".join(sys.argv)
		wafver = Context.WAFVERSION
		abi = Context.ABI
		self.to_log(Configure.conf_template % vars())

		super(Configure.ConfigurationContext, self).execute()


# change the superclass of existing tasks to force timestamps (the build has no state)
def status(self):
	for t in self.run_after:
		if not t.hasrun:
			return Task.ASK_LATER

	implicit_deps = []
	try:
		implicit_deps, _ = self.scan()
	except:
		pass

	# we can add one more node, for example:
	implicit_deps.append(self.generator.path.make_node('wscript'))

	for x in self.inputs + self.dep_nodes + implicit_deps:
		for y in self.outputs:
			try:
				if os.stat(x.abspath()).st_mtime > os.stat(y.abspath()).st_mtime:
					return Task.RUN_ME
			except:
				return Task.RUN_ME

	return Task.SKIP_ME
Task.Task.runnable_status = status

# the post build execution does not need to deal with signatures or anything else
Task.Task.post_run = Utils.nada

