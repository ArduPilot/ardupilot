Waf Tools
=========

Special python modules called Waf **tools** provide functions and classes to help
using compilers, libraries or programs. The typical usage from a user script is::

	def function(ctx):
		# ...
		ctx.load('toolname')

Where the function is usually:

* options: add command-line options used by the tool
* configure: modify ``conf.env``, raise a configuration error if a prerequisite is not met

The tools will usually enhance the application by adding:

* new commands deriving from :py:class:`waflib.Context.Context`
* new task classes deriving from :py:class:`waflib.Task.Task`
* new methods to :py:class:`waflib.Configure.ConfigurationContext` and :py:class:`waflib.Build.BuildContext` through :py:func:`waflib.Configure.conf`
* new task generator methods to :py:class:`waflib.TaskGen.task_gen` through :py:func:`waflib.TaskGen.taskgen_method`, :py:func:`waflib.TaskGen.after`

As a general rule, existing methods or classes are hardly ever replaced.

C/C++ compiler detection
------------------------

The following Waf tools are used for loading specific C or C++ compilers. They may
be used directly, for example::

	def options(opt):
		opt.load('compiler_c')
	def configure(conf):
		conf.load('compiler_c')

.. toctree::

	tools/clang
	tools/clangxx
	tools/compiler_c
	tools/compiler_cxx
	tools/ar
	tools/gcc
	tools/gxx
	tools/icc
	tools/icpc
	tools/suncc
	tools/suncxx
	tools/xlc
	tools/xlcxx
	tools/msvc
	tools/winres
	tools/irixcc

C/C++ support
-------------

The following modules contain the functions and classes required for building C and C++ applications. They
are almost always loaded by other Waf tools. Among these, the most important from a user point of view
is :py:mod:`waflib.Tools.c_config` which provides the :py:func:`waflib.Tools.c_config.check` and
:py:func:`waflib.Tools.c_config.check_cfg` functions.

.. toctree::

	tools/ccroot
	tools/c
	tools/cxx
	tools/c_config
	tools/c_osx
	tools/c_preproc
	tools/c_tests
	tools/c_aliases

Assembly
--------

The following tools provide support for assembly. The module :py:mod:`waflib.Tools.asm` is loaded automatically by :py:mod:`waflib.Tools.nasm` or :py:mod:`waflib.Tools.gas`.

.. toctree::

	tools/gas
	tools/nasm
	tools/asm

D language and compilers
------------------------

The first three tools in the following list may be used for detecting D compilers. The remaining contain the support functions and classes.

.. toctree::

	tools/compiler_d
	tools/dmd
	tools/ldc2
	tools/gdc
	tools/d_config
	tools/d
	tools/d_scan

Fortran support
---------------

The first four tools in the following list are used for detecting fortran compilers. The three remaining contain the routines for compiling fortran applications.

.. toctree::

	tools/compiler_fc
	tools/g95
	tools/gfortran
	tools/ifort
	tools/fc
	tools/fc_config
	tools/fc_scan

C/C++-related applications
--------------------------

The next tools provide support for code generators used in C and C++ projects.

.. toctree::

	tools/bison
	tools/flex
	tools/dbus
	tools/vala
	tools/glib2
	tools/qt5
	tools/perl
	tools/python
	tools/ruby

Other compilers and tools
-------------------------

.. _extras: https://gitlab.com/ita1024/waf/tree/master/waflib/extras

The following tools provide support for specific compilers or configurations. More tools are present in the extras_ folder, although they are not documented and as stable as the default tools.

.. toctree::

	tools/waf_unit_test
	tools/tex
	tools/javaw
	tools/cs
	tools/gnu_dirs
	tools/intltool
	tools/lua
	tools/md5_tstamp
	tools/nobuild

