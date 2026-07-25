Waf tutorial
============

Waf is a piece of software used to help building software projects.
The goal of this tutorial is to provide a quick overview of how to set up
the scripts for a project using Waf.

Waf scripts and commands
------------------------

A software typically has *source files* which are kept in a version control system (git, subversion, etc),
and *build scripts* (Makefiles, ..) which describe what to do with those files. A few *build files* are usually
obtained after transforming the *source files*, but they are optional. The build scripts in Waf are files named 'wscript'.

In general, a project will consist of several phases:

* configure: configure the project, find the location of the prerequisites
* build: transform the source files into build files
* install: install the build files
* uninstall: uninstall the build files
* dist: create an archive of the source files
* clean: remove the build files

Each phase is modelled in the wscript file as a python function which takes as argument an instance of :py:class:`waflib.Context.Context`.
Let's start with a new wscript file in the directory '/tmp/myproject'::

	def configure(cnf):
		print("configure!")

	def build(bld):
		print("build!")

We will also use a Waf binary file, for example waf-2.0.0, which we will copy in the project directory::

	$ cd /tmp/myproject
	$ wget https://waf.io/waf-2.0.0

To execute the project, we will simply call the command as an argument to ``waf``::

	$ python ./waf-2.0.0 configure build
	configure!
	build!

Targets
-------

An important part of the build system is to declare the creation of targets. Here is a very simple example::

	def build(bld):
		tg = bld(rule='cp ${SRC} ${TGT}', source='wscript', target='foo.txt')
		bld(rule='cp ${SRC} ${TGT}', source='foo.txt', target='bar.txt')

The call ``bld(..)`` creates an object called *task generator*, which is used to create *tasks* which will actually
call the command ``cp``. The commands are not executed unless all the scripts have been read, which is important
for computing the build order.

The expressions *${SRC}* and *${TGT}* are shortcuts to avoid repeating the file names. More shortcuts can be defined
by using the *${}* symbol, which reads the values from the attribute bld.env::

	def build(bld):
		bld.env.MESSAGE = 'Hello, world!'
		bld(rule='echo ${MESSAGE}', always=True)

The bld object is an instance of :py:class:`waflib.Build.BuildContext`, its *env* attribute is an instance :py:class:`waflib.ConfigSet.ConfigSet`.
This object is also accessible as an attribute on the `configure()` method's `cnf` parameter. Therefore, values can be shared/stored/loaded easily:

	def configure(cnf):
		cnf.env.MESSAGE = 'Hello, world!'

	def build(bld):
		bld(rule='echo ${MESSAGE}', always=True)

Scripts and Tools
-----------------

To let a script use a script from a subdirectory, the method :py:meth:`waflib.Context.Context.recurse` has to be used with
the relative path to the folder containing the wscript file. For example, to call the function *build* in the script ``src/wscript``,
one should write::

	def build(bld):
		bld.recurse('src')

The support for specific languages and compilers is provided through specific modules called *Waf tools*. The tools are
similar to wscript files and provide functions such as *configure* or *build*. Here is a simple project for the C programming language::

	def options(opt):
		opt.load('compiler_c')
	def configure(cnf):
		cnf.load('compiler_c')
	def build(bld):
		bld(features='c cprogram', source='main.c', target='app')

The function *options* is another predefined command used for setting command-line options. Its argument is an instance of :py:meth:`waflib.Options.OptionsContext`. The tool *compiler_c* is provided for detecting if a C compiler is present and to set various variables such as ``cnf.env.CFLAGS``.

The task generator declared in *bld* does not have a *rule* keyword, but a list of *features* which is used to reference methods that will call the appropriate rules. In this case, a rule is called for compiling the file, and another is used for linking the object files into the binary *app*. Other tool-dependent features exist such as *javac*, *cs*, or *tex*.

A C and C++ project
-------------------

Here is a script for a more complicated project::

	def options(opt):
		opt.load('compiler_c compiler_cxx')
	def configure(cnf):
		cnf.load('compiler_c compiler_cxx')
		cnf.check(features='cxx cxxprogram', lib=['m'], cflags=['-Wall'], defines=['var=foo'], uselib_store='M')
	def build(bld):
		bld(features='c cshlib', source='b.c', target='mylib')
		bld(features='c cxx cxxprogram', source='a.c main.cpp', target='app', use=['M','mylib'], lib=['dl'])

The method :py:func:`waflib.Tools.c_config.check` executes a build internally to check if the library ``libm`` is present on the operating system.
It will then define variables such as:

* ``cnf.env.LIB_M = ['m']``
* ``cnf.env.CFLAGS_M = ['-Wall']``
* ``cnf.env.DEFINES_M = ['var=foo']``

By stating ``use=['M', 'mylib']``, the program *app* is going to inherit all the *M* variables defined
during the configuration. The program will also use the library *mylib* and both the build order and the dependencies
will be modified so that *mylib* is linked before *app*.

The ``use`` attribute is also working for other languages such as Java (dependencies between jar files) or C# (dependencies between assemblies).

Project-specific extensions
---------------------------

The *feature* keyword is a high-level reference to existing Waf methods.
For example, the **c** feature will add the method :py:func:`waflib.Tools.ccroot.apply_incpaths` for execution.
To add a new method that will add the task generator path to the include path for all C targets,
one may use such a declaration::

	from waflib import Utils
	from waflib.TaskGen import feature, before_method
	@feature('c')
	@before_method('apply_incpaths')
	def add_current_dir_to_includes(self):
		self.includes = Utils.to_list(self.includes)
		self.includes.append(self.path)

	def build(bld):
		tg = bld(features='c', source='main.c', target='app')

The *feature* methods are bound to the :py:class:`waflib.TaskGen.task_gen` class, which is the class of the
object *tg* in the example. New features can be declared in the same manner::

	from waflib.TaskGen import feature, after_method
	@feature('debug_tasks')
	@after_method('apply_link')
	def print_debug(self):
		print('tasks created %r' % self.tasks)

	def build(bld):
		tg = bld(features='c cprogram debug_tasks', source='main.c', target='app')

The declaration can be made more user-friendly by binding new methods to the context classes::

	from waflib.Build import BuildContext
	def enterprise_program(self, *k, **kw):
		kw['features'] = 'c cprogram debug_tasks'
		return self(*k, **kw)
	BuildContext.enterprise_program = enterprise_program

	def build(bld):
		# no feature line
		bld.enterprise_program(source='main.c', target='app')

The support code may be turned into a Waf tool by moving it to a separate file.
To ease the deployment, the new Waf tool can even be added to the waf file (see https://gitlab.com/ita1024/waf/blob/master/README.md#L20).

Conclusion
----------

This concludes the tutorial. For more information consult the apis, the Waf book and the examples.

