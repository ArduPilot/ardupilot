#!/usr/bin/env python
# encoding: utf-8
# Copyright Garmin International or its subsidiaries, 2012-2013

'''
Off-load dependency scanning from Python code to MSVC compiler

This tool is safe to load in any environment; it will only activate the
MSVC exploits when it finds that a particular taskgen uses MSVC to
compile.

Empirical testing shows about a 10% execution time savings from using
this tool as compared to c_preproc.

The technique of gutting scan() and pushing the dependency calculation
down to post_run() is cribbed from gccdeps.py.

This affects the cxx class, so make sure to load Qt5 after this tool.

Usage::

	def options(opt):
		opt.load('compiler_cxx')
	def configure(conf):
		conf.load('compiler_cxx msvcdeps')
'''

import os, sys, tempfile, threading

from waflib import Context, Errors, Logs, Task, Utils
from waflib.Tools import c_preproc, c, cxx, msvc
from waflib.TaskGen import feature, before_method

lock = threading.Lock()

PREPROCESSOR_FLAG = '/showIncludes'
INCLUDE_PATTERN = 'Note: including file:'

# Extensible by outside tools
supported_compilers = ['msvc']

@feature('c', 'cxx')
@before_method('process_source')
def apply_msvcdeps_flags(taskgen):
	if taskgen.env.CC_NAME not in supported_compilers:
		return

	for flag in ('CFLAGS', 'CXXFLAGS'):
		if taskgen.env.get_flat(flag).find(PREPROCESSOR_FLAG) < 0:
			taskgen.env.append_value(flag, PREPROCESSOR_FLAG)


def get_correct_path_case(base_path, path):
	'''
	Return a case-corrected version of ``path`` by searching the filesystem for
	``path``, relative to ``base_path``, using the case returned by the filesystem.
	'''
	components = Utils.split_path(path)

	corrected_path = ''
	if os.path.isabs(path):
		corrected_path = components.pop(0).upper() + os.sep

	for part in components:
		part = part.lower()
		search_path = os.path.join(base_path, corrected_path)
		if part == '..':
			corrected_path = os.path.join(corrected_path, part)
			search_path = os.path.normpath(search_path)
			continue

		for item in sorted(os.listdir(search_path)):
			if item.lower() == part:
				corrected_path = os.path.join(corrected_path, item)
				break
		else:
			raise ValueError("Can't find %r in %r" % (part, search_path))

	return corrected_path


def path_to_node(base_node, path, cached_nodes):
	'''
	Take the base node and the path and return a node
	Results are cached because searching the node tree is expensive
	The following code is executed by threads, it is not safe, so a lock is needed...
	'''
	# normalize the path to remove parent path components (..)
	path = os.path.normpath(path)

	# normalize the path case to increase likelihood of a cache hit
	node_lookup_key = (base_node, os.path.normcase(path))

	try:
		node = cached_nodes[node_lookup_key]
	except KeyError:
		# retry with lock on cache miss
		with lock:
			try:
				node = cached_nodes[node_lookup_key]
			except KeyError:
				path = get_correct_path_case(base_node.abspath(), path)
				node = cached_nodes[node_lookup_key] = base_node.find_node(path)

	return node

def post_run(self):
	if self.env.CC_NAME not in supported_compilers:
		return super(self.derived_msvcdeps, self).post_run()

	# TODO this is unlikely to work with netcache
	if getattr(self, 'cached', None):
		return Task.Task.post_run(self)

	resolved_nodes = []
	unresolved_names = []
	bld = self.generator.bld

	# Dynamically bind to the cache
	try:
		cached_nodes = bld.cached_nodes
	except AttributeError:
		cached_nodes = bld.cached_nodes = {}

	for path in self.msvcdeps_paths:
		node = None
		if os.path.isabs(path):
			node = path_to_node(bld.root, path, cached_nodes)
		else:
			# when calling find_resource, make sure the path does not begin with '..'
			base_node = bld.bldnode
			path = [k for k in Utils.split_path(path) if k and k != '.']
			while path[0] == '..':
				path.pop(0)
				base_node = base_node.parent
			path = os.sep.join(path)

			node = path_to_node(base_node, path, cached_nodes)

		if not node:
			raise ValueError('could not find %r for %r' % (path, self))
		else:
			if not c_preproc.go_absolute:
				if not (node.is_child_of(bld.srcnode) or node.is_child_of(bld.bldnode)):
					# System library
					Logs.debug('msvcdeps: Ignoring system include %r', node)
					continue

			if id(node) == id(self.inputs[0]):
				# ignore the source file, it is already in the dependencies
				# this way, successful config tests may be retrieved from the cache
				continue

			resolved_nodes.append(node)

	Logs.debug('deps: msvcdeps for %s returned %s', self, resolved_nodes)

	bld.node_deps[self.uid()] = resolved_nodes
	bld.raw_deps[self.uid()] = unresolved_names

	try:
		del self.cache_sig
	except AttributeError:
		pass

	Task.Task.post_run(self)

def scan(self):
	if self.env.CC_NAME not in supported_compilers:
		return super(self.derived_msvcdeps, self).scan()

	resolved_nodes = self.generator.bld.node_deps.get(self.uid(), [])
	unresolved_names = []
	return (resolved_nodes, unresolved_names)

def sig_implicit_deps(self):
	if self.env.CC_NAME not in supported_compilers:
		return super(self.derived_msvcdeps, self).sig_implicit_deps()
	bld = self.generator.bld

	try:
		return self.compute_sig_implicit_deps()
	except Errors.TaskNotReady:
		raise ValueError("Please specify the build order precisely with msvcdeps (c/c++ tasks)")
	except EnvironmentError:
		# If a file is renamed, assume the dependencies are stale and must be recalculated
		for x in bld.node_deps.get(self.uid(), []):
			if not x.is_bld() and not x.exists():
				try:
					del x.parent.children[x.name]
				except KeyError:
					pass

	key = self.uid()
	bld.node_deps[key] = []
	bld.raw_deps[key] = []
	return Utils.SIG_NIL

def exec_command(self, cmd, **kw):
	if self.env.CC_NAME not in supported_compilers:
		return super(self.derived_msvcdeps, self).exec_command(cmd, **kw)

	if not 'cwd' in kw:
		kw['cwd'] = self.get_cwd()

	if self.env.PATH:
		env = kw['env'] = dict(kw.get('env') or self.env.env or os.environ)
		env['PATH'] = self.env.PATH if isinstance(self.env.PATH, str) else os.pathsep.join(self.env.PATH)

	# The Visual Studio IDE adds an environment variable that causes
	# the MS compiler to send its textual output directly to the
	# debugging window rather than normal stdout/stderr.
	#
	# This is unrecoverably bad for this tool because it will cause
	# all the dependency scanning to see an empty stdout stream and
	# assume that the file being compiled uses no headers.
	#
	# See http://blogs.msdn.com/b/freik/archive/2006/04/05/569025.aspx
	#
	# Attempting to repair the situation by deleting the offending
	# envvar at this point in tool execution will not be good enough--
	# its presence poisons the 'waf configure' step earlier. We just
	# want to put a sanity check here in order to help developers
	# quickly diagnose the issue if an otherwise-good Waf tree
	# is then executed inside the MSVS IDE.
	assert 'VS_UNICODE_OUTPUT' not in kw['env']

	cmd, args = self.split_argfile(cmd)
	try:
		(fd, tmp) = tempfile.mkstemp()
		os.write(fd, '\r\n'.join(args).encode())
		os.close(fd)

		self.msvcdeps_paths = []
		kw['env'] = kw.get('env', os.environ.copy())
		kw['cwd'] = kw.get('cwd', os.getcwd())
		kw['quiet'] = Context.STDOUT
		kw['output'] = Context.STDOUT

		out = []
		if Logs.verbose:
			Logs.debug('argfile: @%r -> %r', tmp, args)
		try:
			raw_out = self.generator.bld.cmd_and_log(cmd + ['@' + tmp], **kw)
			ret = 0
		except Errors.WafError as e:
			# Use e.msg if e.stdout is not set
			raw_out = getattr(e, 'stdout', e.msg)

			# Return non-zero error code even if we didn't
			# get one from the exception object
			ret = getattr(e, 'returncode', 1)

		Logs.debug('msvcdeps: Running for: %s' % self.inputs[0])
		for line in raw_out.splitlines():
			if line.startswith(INCLUDE_PATTERN):
				# Only strip whitespace after log to preserve
				# dependency structure in debug output
				inc_path = line[len(INCLUDE_PATTERN):]
				Logs.debug('msvcdeps: Regex matched %s', inc_path)
				self.msvcdeps_paths.append(inc_path.strip())
			else:
				out.append(line)

		# Pipe through the remaining stdout content (not related to /showIncludes)
		if self.generator.bld.logger:
			self.generator.bld.logger.debug('out: %s' % os.linesep.join(out))
		elif len(out) > 1:
			# msvc will output the input file name by default, which is not useful
			# in the single-file case as waf will already print task. For multi-file
			# inputs or other messages, allow the full message to be forwarded.
			Logs.info(os.linesep.join(out), extra={'stream':sys.stdout, 'c1': ''})

		return ret
	finally:
		try:
			os.remove(tmp)
		except OSError:
			# anti-virus and indexers can keep files open -_-
			pass


def wrap_compiled_task(classname):
	derived_class = type(classname, (Task.classes[classname],), {})
	derived_class.derived_msvcdeps = derived_class
	derived_class.post_run = post_run
	derived_class.scan = scan
	derived_class.sig_implicit_deps = sig_implicit_deps
	derived_class.exec_command = exec_command

for k in ('c', 'cxx'):
	if k in Task.classes:
		wrap_compiled_task(k)

def options(opt):
	raise ValueError('Do not load msvcdeps options')
