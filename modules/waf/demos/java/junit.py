#! /usr/bin/env python
# encoding: utf-8

"""
JUnit test system

 - executes all junit tests in the specified subtree (junitsrc)
   - only if --junit is given on the commandline
   - method:
	 - add task to compile junitsrc after compiling srcdir
	   - additional junit_classpath specifiable
		 - defaults to classpath + destdir
	 - add task to run junit tests after they're compiled.
"""

import os
from waflib import Task, TaskGen, Utils, Options
from waflib.TaskGen import feature, before, after
from waflib.Configure import conf

JUNIT_RUNNER = 'org.junit.runner.JUnitCore'

def options(opt):
	opt.add_option('--junit', action='store_true', default=False,
		help='Run all junit tests', dest='junit')
	opt.add_option('--junitpath', action='store', default='',
		help='Give a path to the junit jar')

def configure(ctx):
	cp = ctx.options.junitpath
	val = ctx.env.JUNIT_RUNNER = ctx.env.JUNIT_RUNNER or JUNIT_RUNNER
	if ctx.check_java_class(val, with_classpath=cp):
		ctx.fatal('Could not run junit from %r' % val)
	ctx.env.CLASSPATH_JUNIT = cp

#@feature('junit')
#@after('apply_java', 'use_javac_files')
def make_test(self):
	"""make the unit test task"""
	if not getattr(self, 'junitsrc', None):
		return
	junit_task = self.create_task('junit_test')
	try:
		junit_task.set_run_after(self.javac_task)
	except AttributeError:
		pass
feature('junit')(make_test)
after('apply_java', 'use_javac_files')(make_test)

class junit_test(Task.Task):
	color = 'YELLOW'
	vars = ['JUNIT_EXEC_FLAGS', 'JUNIT_RUNNER']

	def runnable_status(self):
		"""
		Only run if --junit was set as an option
		"""
		for t in self.run_after:
			if not t.hasrun:
				return Task.ASK_LATER

		n = self.generator.path.find_dir(self.generator.junitsrc)
		if not n:
			self.generator.bld.fatal('no such junit directory %r' % self.generator.junitsrc)
		self.base = n

		# make sure the tests are executed whenever the .class files change
		self.inputs = n.ant_glob('**/*.java')

		ret = super(junit_test, self).runnable_status()
		if ret == Task.SKIP_ME:
			if getattr(Options.options, 'junit', False):
				ret = Task.RUN_ME
		return ret

	def run(self):
		cmd = []
		cmd.extend(self.env.JAVA)
		cmd.append('-classpath')
		cmd.append(self.generator.javac_task.env.CLASSPATH + os.pathsep + self.generator.javac_task.env.OUTDIR)
		cmd.extend(self.env.JUNIT_EXEC_FLAGS)
		cmd.append(self.env.JUNIT_RUNNER)
		cmd.extend([x.path_from(self.base).replace('.java', '').replace(os.sep, '.') for x in self.inputs])
		return self.exec_command(cmd)

