#! /usr/bin/env python
# encoding: utf-8
# Federico Pellegrin, 2019 (fedepell)

"""
Provides Java Unit test support using :py:class:`waflib.Tools.waf_unit_test.utest`
task via the **javatest** feature.

This gives the possibility to run unit test and have them integrated into the
standard waf unit test environment. It has been tested with TestNG and JUnit
but should be easily expandable to other frameworks given the flexibility of
ut_str provided by the standard waf unit test environment.

The extra takes care also of managing non-java dependencies (ie. C/C++ libraries
using JNI or Python modules via JEP) and setting up the environment needed to run
them.

Example usage:

def options(opt):
	opt.load('java waf_unit_test javatest')

def configure(conf):
	conf.load('java javatest')

def build(bld):

	[ ... mainprog is built here ... ]

	bld(features = 'javac javatest',
		srcdir     = 'test/',
		outdir     = 'test',
		sourcepath = ['test'],
		classpath  = [ 'src' ],
		basedir    = 'test',
		use = ['JAVATEST', 'mainprog'], # mainprog is the program being tested in src/
		ut_str = 'java -cp ${CLASSPATH} ${JTRUNNER} ${SRC}',
		jtest_source = bld.path.ant_glob('test/*.xml'),
	)


At command line the CLASSPATH where to find the testing environment and the
test runner (default TestNG) that will then be seen in the environment as
CLASSPATH_JAVATEST (then used for use) and JTRUNNER and can be used for
dependencies and ut_str generation.

Example configure for TestNG:
	waf configure --jtpath=/tmp/testng-6.12.jar:/tmp/jcommander-1.71.jar --jtrunner=org.testng.TestNG
		 or as default runner is TestNG:
	waf configure --jtpath=/tmp/testng-6.12.jar:/tmp/jcommander-1.71.jar

Example configure for JUnit:
	waf configure --jtpath=/tmp/junit.jar --jtrunner=org.junit.runner.JUnitCore

The runner class presence on the system is checked for at configuration stage.

"""

import os
from waflib import Task, TaskGen, Options, Errors, Utils, Logs
from waflib.Tools import ccroot

JAR_RE = '**/*'

def _process_use_rec(self, name):
	"""
	Recursively process ``use`` for task generator with name ``name``..
	Used by javatest_process_use.
	"""
	if name in self.javatest_use_not or name in self.javatest_use_seen:
		return
	try:
		tg = self.bld.get_tgen_by_name(name)
	except Errors.WafError:
		self.javatest_use_not.add(name)
		return

	self.javatest_use_seen.append(name)
	tg.post()

	for n in self.to_list(getattr(tg, 'use', [])):
		_process_use_rec(self, n)

@TaskGen.feature('javatest')
@TaskGen.after_method('process_source', 'apply_link', 'use_javac_files')
def javatest_process_use(self):
	"""
	Process the ``use`` attribute which contains a list of task generator names and store
	paths that later is used to populate the unit test runtime environment.
	"""
	self.javatest_use_not = set()
	self.javatest_use_seen = []
	self.javatest_libpaths = [] # strings or Nodes
	self.javatest_pypaths = [] # strings or Nodes
	self.javatest_dep_nodes = []

	names = self.to_list(getattr(self, 'use', []))
	for name in names:
		_process_use_rec(self, name)

	def extend_unique(lst, varlst):
		ext = []
		for x in varlst:
			if x not in lst:
				ext.append(x)
		lst.extend(ext)

	# Collect type specific info needed to construct a valid runtime environment
	# for the test.
	for name in self.javatest_use_seen:
		tg = self.bld.get_tgen_by_name(name)

		# Python-Java embedding crosstools such as JEP
		if 'py' in tg.features:
			# Python dependencies are added to PYTHONPATH
			pypath = getattr(tg, 'install_from', tg.path)

			if 'buildcopy' in tg.features:
				# Since buildcopy is used we assume that PYTHONPATH in build should be used,
				# not source
				extend_unique(self.javatest_pypaths, [pypath.get_bld().abspath()])

				# Add buildcopy output nodes to dependencies
				extend_unique(self.javatest_dep_nodes, [o for task in getattr(tg, 'tasks', []) for o in getattr(task, 'outputs', [])])
			else:
				# If buildcopy is not used, depend on sources instead
				extend_unique(self.javatest_dep_nodes, tg.source)
				extend_unique(self.javatest_pypaths, [pypath.abspath()])


		if getattr(tg, 'link_task', None):
			# For tasks with a link_task (C, C++, D et.c.) include their library paths:
			if not isinstance(tg.link_task, ccroot.stlink_task):
				extend_unique(self.javatest_dep_nodes, tg.link_task.outputs)
				extend_unique(self.javatest_libpaths, tg.link_task.env.LIBPATH)

				if 'pyext' in tg.features:
					# If the taskgen is extending Python we also want to add the interpreter libpath.
					extend_unique(self.javatest_libpaths, tg.link_task.env.LIBPATH_PYEXT)
				else:
					# Only add to libpath if the link task is not a Python extension
					extend_unique(self.javatest_libpaths, [tg.link_task.outputs[0].parent.abspath()])

		if 'javac' in tg.features or 'jar' in tg.features:
			if hasattr(tg, 'jar_task'):
				# For Java JAR tasks depend on generated JAR
				extend_unique(self.javatest_dep_nodes, tg.jar_task.outputs)
			else:
				# For Java non-JAR ones we need to glob generated files (Java output files are not predictable)
				if hasattr(tg, 'outdir'):
					base_node = tg.outdir
				else:
					base_node = tg.path.get_bld()

				self.javatest_dep_nodes.extend([dx for dx in base_node.ant_glob(JAR_RE, remove=False, quiet=True)])



@TaskGen.feature('javatest')
@TaskGen.after_method('apply_java', 'use_javac_files', 'set_classpath', 'javatest_process_use')
def make_javatest(self):
	"""
	Creates a ``utest`` task with a populated environment for Java Unit test execution

	"""
	tsk = self.create_task('utest')
	tsk.set_run_after(self.javac_task)

	# Dependencies from recursive use analysis
	tsk.dep_nodes.extend(self.javatest_dep_nodes)

	# Put test input files as waf_unit_test relies on that for some prints and log generation
	# If jtest_source is there, this is specially useful for passing XML for TestNG
	# that contain test specification, use that as inputs, otherwise test sources
	if getattr(self, 'jtest_source', None):
		tsk.inputs = self.to_nodes(self.jtest_source)
	else:
		if self.javac_task.srcdir[0].exists():
			tsk.inputs = self.javac_task.srcdir[0].ant_glob('**/*.java', remove=False)

	if getattr(self, 'ut_str', None):
		self.ut_run, lst = Task.compile_fun(self.ut_str, shell=getattr(self, 'ut_shell', False))
		tsk.vars = lst + tsk.vars

	if getattr(self, 'ut_cwd', None):
		if isinstance(self.ut_cwd, str):
			# we want a Node instance
			if os.path.isabs(self.ut_cwd):
				self.ut_cwd = self.bld.root.make_node(self.ut_cwd)
			else:
				self.ut_cwd = self.path.make_node(self.ut_cwd)
	else:
		self.ut_cwd = self.bld.bldnode

	# Get parent CLASSPATH and add output dir of test, we run from wscript dir
	# We have to change it from list to the standard java -cp format (: separated)
	tsk.env.CLASSPATH = ':'.join(self.env.CLASSPATH) + ':' + self.outdir.abspath()

	if not self.ut_cwd.exists():
		self.ut_cwd.mkdir()

	if not hasattr(self, 'ut_env'):
		self.ut_env = dict(os.environ)
		def add_paths(var, lst):
			# Add list of paths to a variable, lst can contain strings or nodes
			lst = [ str(n) for n in lst ]
			Logs.debug("ut: %s: Adding paths %s=%s", self, var, lst)
			self.ut_env[var] = os.pathsep.join(lst) + os.pathsep + self.ut_env.get(var, '')

		add_paths('PYTHONPATH', self.javatest_pypaths)

		if Utils.is_win32:
			add_paths('PATH', self.javatest_libpaths)
		elif Utils.unversioned_sys_platform() == 'darwin':
			add_paths('DYLD_LIBRARY_PATH', self.javatest_libpaths)
			add_paths('LD_LIBRARY_PATH', self.javatest_libpaths)
		else:
			add_paths('LD_LIBRARY_PATH', self.javatest_libpaths)

def configure(ctx):
	cp = ctx.env.CLASSPATH or '.'
	if getattr(Options.options, 'jtpath', None):
		ctx.env.CLASSPATH_JAVATEST = getattr(Options.options, 'jtpath').split(':')
		cp += ':' + getattr(Options.options, 'jtpath')

	if getattr(Options.options, 'jtrunner', None):
		ctx.env.JTRUNNER = getattr(Options.options, 'jtrunner')

	if ctx.check_java_class(ctx.env.JTRUNNER, with_classpath=cp):
		ctx.fatal('Could not run test class %r' % ctx.env.JTRUNNER)

def options(opt):
	opt.add_option('--jtpath', action='store', default='', dest='jtpath',
		help='Path to jar(s) needed for javatest execution, colon separated, if not in the system CLASSPATH')
	opt.add_option('--jtrunner', action='store', default='org.testng.TestNG', dest='jtrunner',
		help='Class to run javatest test [default: org.testng.TestNG]')

